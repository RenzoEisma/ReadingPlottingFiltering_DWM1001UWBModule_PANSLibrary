# ===================== PROGRAM_INFO ==================================================================================
""" Author: Renzo Eisma
    Date: 03/19/2026
    Description: UWB Sensor Measurement with Dynamic Data Routing"""

# =====================================================================================================================
# IMPORTS
# =====================================================================================================================

import serial
import time
import csv
import os
import numpy as np
from datetime import datetime
import re

try:
    import matlab.engine
except ImportError:
    matlab = None
    print("[UWB] WARNING: matlab.engine module not found. MATLAB routing will be disabled.")

# =====================================================================================================================
# UWB KALMAN FILTER & OUTLIER REJECTION (Low Pass Filter)
# =====================================================================================================================
class UWBSmoother:
    def __init__(self):
        self.state = np.zeros(6)
        self.P = np.eye(6) * 500.0
        self.R = np.eye(3) * 0.1
        self.Q = np.eye(6) * 0.01
        self.last_time = None
        self.last_accepted_pos = None

    def process(self, current_time, raw_x, raw_y, raw_z):
        current_pos = np.array([raw_x, raw_y, raw_z])

        if self.last_time is None:
            self.state[0:3] = current_pos
            self.last_time = current_time
            self.last_accepted_pos = current_pos
            return self.state[0], self.state[1], self.state[2]

        dt = current_time - self.last_time
        if dt <= 0: dt = 0.001

        dist_from_last = np.linalg.norm(current_pos - self.last_accepted_pos)
        if dist_from_last > 1.5:
            current_pos = self.state[0:3] + (self.state[3:6] * dt)
        else:
            self.last_accepted_pos = current_pos

        F = np.eye(6)
        F[0, 3], F[1, 4], F[2, 5] = dt, dt, dt

        pred_state = F @ self.state
        pred_P = F @ self.P @ F.T + self.Q

        H = np.zeros((3, 6))
        H[0, 0], H[1, 1], H[2, 2] = 1, 1, 1

        S = H @ pred_P @ H.T + self.R
        K = pred_P @ H.T @ np.linalg.inv(S)

        y = current_pos - (H @ pred_state)
        self.state = pred_state + (K @ y)
        self.P = (np.eye(6) - K @ H) @ pred_P
        self.last_time = current_time

        return self.state[0], self.state[1], self.state[2]


def update_anchor_list(ser, save_dir):
    print(f"[UWB] Requesting anchor positions from {ser.port}...")
    ser.write(b'\r\r')
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(b'la\r')
    time.sleep(0.8)

    anchors_found = []
    lines = ser.readlines()
    coord_pattern = re.compile(r"pos=(-?\d+\.\d+):(-?\d+\.\d+):(-?\d+\.\d+)")

    for line_raw in lines:
        line = line_raw.decode('utf-8', errors='ignore').strip()
        if "id=" in line and "pos=" in line:
            match = coord_pattern.search(line)
            if match:
                x, y, z = match.groups()
                anchors_found.append([float(x), float(y), float(z)])
                print(f" -> Found Anchor: X={x}, Y={y}, Z={z}")

    ser.write(b'\r')
    return anchors_found


def parse_distances(line):
    """
    Extracts distance measurements from the DWM1001 'lec' output string.
    Example DWM1001 output: POS,1,0A,0.00,0.00,0.00,100,bT,4,AN0,2.10,AN1,2.20...
    Returns a list of distances.
    """
    distances = []
    parts = line.split(',')
    # Scan through the parts to find 'ANx' tags and grab the number immediately following it
    for i in range(len(parts) - 1):
        if parts[i].startswith('AN'):
            try:
                dist = float(parts[i + 1])
                distances.append(dist)
            except ValueError:
                pass
    return distances


# ========================================================
# UWB DRIVER LOOP
# ========================================================
def run_uwb(stop_event, config, save_dir, data_queue=None):
    # 1. Unpack Master Configuration
    port1 = config.get('port1')
    port2 = config.get('port2')
    baud = config.get('baud', 115200)

    send_matlab = config.get('send_matlab', False)
    filter_type = config.get('filter_type', 'Python Filter')
    read_type = config.get('read_type', 'Tag Position')
    anchor_count = config.get('anchor_count', '4 Anchors (1 Listener)')

    print(f"[UWB] Routing: MATLAB={send_matlab}, Filter={filter_type}, Mode={read_type}")

    # 2. Determine number of listeners
    ports_to_open = [port1]
    if "8 Anchors" in anchor_count and port2:
        ports_to_open.append(port2)
        print(f"[UWB] Dual listener mode activated. Ports: {port1}, {port2}")

    # 3. Connect to MATLAB Engine
    eng = None
    if send_matlab and matlab is not None:
        print("[UWB] Starting MATLAB Engine (this may take a few seconds)...")
        try:
            eng = matlab.engine.start_matlab()
            eng.workspace['uwb_filter_type'] = filter_type
            eng.workspace['uwb_read_type'] = read_type
            eng.workspace['uwb_anchor_count'] = anchor_count
            print("[UWB] MATLAB Engine Connected.")
        except Exception as e:
            print(f"[UWB] MATLAB Engine failed to start: {e}")
            send_matlab = False

    session_name = datetime.now().strftime("%Y%m%d_%H%M%S")

    active_serials = []
    filters = []
    raw_writers = []
    filt_writers = []
    file_handles = []

    # We create a text buffer for each port to catch fragmented serial lines
    buffers = []

    try:
        # 4. Setup Serial Ports and Output Files
        all_anchors = []
        for i, port in enumerate(ports_to_open):
            # Timeout is mostly ignored now due to the buffer, but kept low to prevent blocking
            ser = serial.Serial(port, baud, timeout=0.05)

            time.sleep(0.5)
            ser.write(b'\r')
            time.sleep(0.1)
            ser.write(b'\r\r')
            time.sleep(0.5)

            active_serials.append(ser)
            filters.append(UWBSmoother())
            buffers.append("")

            # Fetch Anchors
            anchors = update_anchor_list(ser, save_dir)
            all_anchors.extend(anchors)

            # Put module into continuous reporting mode
            ser.write(b'\r\r')
            time.sleep(1.0)  # Increased back to 1.0s to ensure the module wakes up properly
            ser.write(b'lec\r')

            # Create CSVs for this specific listener
            f_raw = open(os.path.join(save_dir, f"[Log]_uwb_listener{i + 1}_{session_name}.csv"), 'w', newline='')
            raw_w = csv.writer(f_raw)

            f_filt = None
            filt_w = None

            if read_type == 'Tag Position':
                raw_w.writerow(['Time', 'POSX', 'POSY', 'POSZ'])
                if filter_type == 'Python Filter':
                    f_filt = open(os.path.join(save_dir, f"[Log]_uwbFiltered_listener{i + 1}_{session_name}.csv"), 'w',
                                  newline='')
                    filt_w = csv.writer(f_filt)
                    filt_w.writerow(['Time', 'POSX', 'POSY', 'POSZ'])
            else:
                raw_w.writerow(['Time', 'Dist1', 'Dist2', 'Dist3', 'Dist4', 'Dist5', 'Dist6', 'Dist7', 'Dist8'])

            raw_writers.append(raw_w)
            filt_writers.append(filt_w)
            file_handles.extend([f_raw, f_filt])

        if all_anchors:
            anchor_file_path = os.path.join(save_dir, "[Log]_anchor_positions.csv")
            with open(anchor_file_path, 'w') as file:
                file.write("X,Y,Z\n")
                unique_anchors = []
                for a in all_anchors:
                    if a not in unique_anchors:
                        unique_anchors.append(a)
                        file.write(f"{a[0]},{a[1]},{a[2]}\n")

        print("[UWB] Listening for data...")

        # 5. Main Processing Loop
        while not stop_event.is_set():
            for i, ser in enumerate(active_serials):
                if ser.in_waiting > 0:
                    # Read all available bytes and dump them into this port's holding buffer
                    new_data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    buffers[i] += new_data

                    # If we have a complete line (signified by a newline character)
                    if '\n' in buffers[i]:
                        lines = buffers[i].split('\n')

                        # The very last item in the split list will be whatever incomplete chunk
                        # arrived after the last \n. We pop it off and keep it in the buffer for next time.
                        buffers[i] = lines.pop()

                        for line in lines:
                            line = line.strip()
                            if not line:
                                continue

                            arr_time = time.time()

                            # ==========================================
                            # MODE A: TAG POSITION
                            # ==========================================
                            if read_type == 'Tag Position' and line.startswith('POS'):
                                parts = [p for p in line.split(',') if p]
                                if len(parts) >= 6:
                                    raw_x, raw_y, raw_z = float(parts[3]), float(parts[4]), float(parts[5])

                                    raw_writers[i].writerow([arr_time, raw_x, raw_y, raw_z])
                                    file_handles[i * 2].flush()

                                    if filter_type == 'Python Filter':
                                        filt_x, filt_y, filt_z = filters[i].process(arr_time, raw_x, raw_y, raw_z)
                                        filt_writers[i].writerow(
                                            [arr_time, round(filt_x, 4), round(filt_y, 4), round(filt_z, 4)])
                                        file_handles[(i * 2) + 1].flush()

                                        if data_queue is not None and i == 0:
                                            data_queue.put(('UWB', filt_x, filt_y, filt_z))

                                        if send_matlab and eng is not None:
                                            eng.eval(f"process_uwb_data({filt_x}, {filt_y}, {filt_z});", nargout=0)

                                    else:
                                        if data_queue is not None and i == 0:
                                            data_queue.put(('UWB', raw_x, raw_y, raw_z))

                                        if send_matlab and eng is not None:
                                            eng.eval(f"process_uwb_data({raw_x}, {raw_y}, {raw_z});", nargout=0)

                            # ==========================================
                            # MODE B: TAG DISTANCES
                            # ==========================================
                            elif read_type == 'Tag Distances' and ('DIST' in line or 'POS' in line):
                                distances = parse_distances(line)
                                if distances:
                                    padded_dist = distances + [0.0] * (8 - len(distances))
                                    raw_writers[i].writerow([arr_time] + padded_dist[:8])
                                    file_handles[i * 2].flush()

                                    if send_matlab and eng is not None:
                                        mat_dist = matlab.double(distances)
                                        eng.workspace['current_distances'] = mat_dist
                                        eng.eval("process_uwb_distances(current_distances);", nargout=0)

            time.sleep(0.001)

    except Exception as e:
        print(f"[UWB Error] {e}")
    finally:
        for ser in active_serials:
            if ser.is_open:
                ser.write(b'\r')
                ser.close()

        for f in file_handles:
            if f is not None:
                f.close()

        if eng is not None:
            eng.quit()
            print("[UWB] MATLAB Engine closed.")

        print("[UWB] Drivers Closed.")
