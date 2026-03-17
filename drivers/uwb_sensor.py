# import serial
# import time
# import csv
# import os
# import numpy as np
# from datetime import datetime
# import re
#
# # ===================== PROGRAM_INFO =====================
# """
#     Author: Renzo Eisma
#     Version: 2.0
#     Date: 02/27/2026
#     Description: UWB Sensor Measurement
# """
#
# # ========================================================
# # UWB KALMAN FILTER & OUTLIER REJECTION (Low Pass Filter)
# # ========================================================
# class UWBSmoother:
#     def __init__(self):
#         # State vector: [X, Y, Z, Vx, Vy, Vz]
#         self.state = np.zeros(6)
#
#         # Uncertainty Matrix (P)
#         self.P = np.eye(6) * 500.0
#
#         # Measurement Noise (R) - Increase this (e.g., 0.5) if UWB is very jittery
#         self.R = np.eye(3) * 0.1
#
#         # Process Noise (Q) - Decrease this (e.g., 0.001) if drone moves slowly
#         self.Q = np.eye(6) * 0.01
#
#         self.last_time = None
#         self.last_accepted_pos = None
#
#     def process(self, current_time, raw_x, raw_y, raw_z):
#         current_pos = np.array([raw_x, raw_y, raw_z])
#
#         # Initialization step
#         if self.last_time is None:
#             self.state[0:3] = current_pos
#             self.last_time = current_time
#             self.last_accepted_pos = current_pos
#             return self.state[0], self.state[1], self.state[2]
#
#         dt = current_time - self.last_time
#         if dt <= 0: dt = 0.001
#
#         # --- STAGE 1: OUTLIER REJECTION ---
#         dist_from_last = np.linalg.norm(current_pos - self.last_accepted_pos)
#         if dist_from_last > 1.5:  # If it jumps > 1.5m instantly, ignore it
#             current_pos = self.state[0:3] + (self.state[3:6] * dt)
#         else:
#             self.last_accepted_pos = current_pos
#
#         # --- STAGE 2: KALMAN PREDICT & UPDATE ---
#         F = np.eye(6)
#         F[0, 3], F[1, 4], F[2, 5] = dt, dt, dt
#
#         pred_state = F @ self.state
#         pred_P = F @ self.P @ F.T + self.Q
#
#         H = np.zeros((3, 6))
#         H[0, 0], H[1, 1], H[2, 2] = 1, 1, 1
#
#         S = H @ pred_P @ H.T + self.R
#         K = pred_P @ H.T @ np.linalg.inv(S)
#
#         y = current_pos - (H @ pred_state)
#         self.state = pred_state + (K @ y)
#         self.P = (np.eye(6) - K @ H) @ pred_P
#
#         self.last_time = current_time
#
#         return self.state[0], self.state[1], self.state[2]
#
#
# def update_anchor_list(ser, save_dir):
#     """
#     Sends 'la' to the UWB listener, parses the anchor positions,
#     and saves them to a text file.
#     """
#     print("[UWB] Requesting anchor positions...")
#     ser.write(b'\r\r')  # Wake up shell
#     time.sleep(0.2)
#     ser.reset_input_buffer()  # Clear any 'lec' or 'lep' data left in the pipe
#     ser.write(b'la\r')
#     time.sleep(0.8)  # Give the system time to dump all anchor lines
#
#     anchors_found = []
#     lines = ser.readlines() # Read the buffer
#
#     # Regex explained:
#     # pos= matches the literal string
#     # (-?\d+\.\d+) matches a positive or negative decimal number
#     # : is the separator used by PANS firmware
#     coord_pattern = re.compile(r"pos=(-?\d+\.\d+):(-?\d+\.\d+):(-?\d+\.\d+)")
#
#     for line_raw in lines:
#         line = line_raw.decode('utf-8', errors='ignore').strip()
#
#         # Look for lines containing "id=" and "pos="
#         if "id=" in line and "pos=" in line:
#             match = coord_pattern.search(line)
#             if match:
#                 x, y, z = match.groups()
#                 anchors_found.append([float(x), float(y), float(z)])
#                 print(f" -> Found Anchor: X={x}, Y={y}, Z={z}")
#
#     if anchors_found:
#         anchor_file_path = os.path.join(save_dir, "anchor_positions.csv")
#         with open(anchor_file_path, 'w') as file:
#             file.write("X,Y,Z\n")
#             for a in anchors_found:
#                 file.write(f"{a[0]},{a[1]},{a[2]}\n")
#         print(f"[UWB] Success: {len(anchors_found)} anchors saved to {anchor_file_path}")
#     else:
#         print("[UWB] WARNING: No anchors detected in the 'la' output.")
#
#     ser.write(b'\r')  # Clean up shell
#     return anchors_found
#
# def process_serial_data(ser, kf, raw_writer, filt_writer, f_raw, f_filt):
#     line = ser.readline().decode('utf-8', errors='ignore').strip()
#     if line.startswith('POS'):
#         parts = [p for p in line.split(',') if p]
#         if len(parts) >= 6:
#             arr_time = time.time()
#             raw_x, raw_y, raw_z = float(parts[3]), float(parts[4]), float(parts[5])
#
#             # Write Raw Data
#             raw_writer.writerow([arr_time, raw_x, raw_y, raw_z])
#             f_raw.flush()
#
#             # Pass through Filter
#             filt_x, filt_y, filt_z = kf.process(arr_time, raw_x, raw_y, raw_z)
#
#             # Write Filtered Data
#             filt_writer.writerow([arr_time, round(filt_x, 4), round(filt_y, 4), round(filt_z, 4)])
#             f_filt.flush()
#
#
# # ========================================================
# # UWB DRIVER LOOP
# # ========================================================
# def run_uwb(stop_event, config, save_dir):
#     port1 = config['port1']
#     port2 = config['port2']  # This will be COM4 from your master logger
#     baud = config['baud']
#
#     session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
#
#     # Filenames for Listener 1
#     raw_filename1 = os.path.join(save_dir, f"uwb_1_log_{session_name}.csv")
#     filt_filename1 = os.path.join(save_dir, f"uwbFiltered_1_log_{session_name}.csv")
#
#     # Filenames for Listener 2
#     raw_filename2 = os.path.join(save_dir, f"uwb_2_log_{session_name}.csv")
#     filt_filename2 = os.path.join(save_dir, f"uwbFiltered_2_log_{session_name}.csv")
#
#     # Initialize two separate filters so the states do not mix
#     kf1 = UWBSmoother()
#     kf2 = UWBSmoother()
#
#     print(f"[UWB] Opening {port1} and {port2}...")
#     try:
#         # Lower the timeout so the loop doesn't hang waiting for one port
#         ser1 = serial.Serial(port1, baud, timeout=0.05)
#         ser2 = serial.Serial(port2, baud, timeout=0.05)
#
#         # Optional: You can run update_anchor_list on ser1 here if needed
#
#         for ser in [ser1, ser2]:
#             ser.write(b'\r\r')
#             time.sleep(0.5)
#             ser.write(b'lec\r')
#
#         # Open all four CSV files simultaneously
#         with open(raw_filename1, mode='w', newline='') as f_raw1, \
#                 open(filt_filename1, mode='w', newline='') as f_filt1, \
#                 open(raw_filename2, mode='w', newline='') as f_raw2, \
#                 open(filt_filename2, mode='w', newline='') as f_filt2:
#
#             raw_writer1, filt_writer1 = csv.writer(f_raw1), csv.writer(f_filt1)
#             raw_writer2, filt_writer2 = csv.writer(f_raw2), csv.writer(f_filt2)
#
#             header = ['Time', 'POSX', 'POSY', 'POSZ']
#             for w in [raw_writer1, filt_writer1, raw_writer2, filt_writer2]:
#                 w.writerow(header)
#
#             while not stop_event.is_set():
#                 # Check and process Listener 1
#                 if ser1.in_waiting > 0:
#                     process_serial_data(ser1, kf1, raw_writer1, filt_writer1, f_raw1, f_filt1)
#
#                 # Check and process Listener 2
#                 if ser2.in_waiting > 0:
#                     process_serial_data(ser2, kf2, raw_writer2, filt_writer2, f_raw2, f_filt2)
#
#                 # Tiny sleep to prevent the while loop from maxing out a CPU core
#                 time.sleep(0.001)
#
#     except Exception as e:
#         print(f"[UWB Error] {e}")
#     finally:
#         if 'ser1' in locals() and ser1.is_open:
#             ser1.write(b'\r')
#             ser1.close()
#         if 'ser2' in locals() and ser2.is_open:
#             ser2.write(b'\r')
#             ser2.close()
#         print("[UWB] Closed both ports.")


# ===================================================================================================================
# Old code with one serial port
# ===================================================================================================================

import serial
import time
import csv
import os
import numpy as np
from datetime import datetime
import re

# ===================== PROGRAM_INFO =====================
"""
    Author: Renzo Eisma
    Version: 2.0
    Date: 02/27/2026
    Description: UWB Sensor Measurement
"""

# ========================================================
# UWB KALMAN FILTER & OUTLIER REJECTION (Low Pass Filter)
# ========================================================
class UWBSmoother:
    def __init__(self):
        # State vector: [X, Y, Z, Vx, Vy, Vz]
        self.state = np.zeros(6)

        # Uncertainty Matrix (P)
        self.P = np.eye(6) * 500.0

        # Measurement Noise (R) - Increase this (e.g., 0.5) if UWB is very jittery
        self.R = np.eye(3) * 0.1

        # Process Noise (Q) - Decrease this (e.g., 0.001) if drone moves slowly
        self.Q = np.eye(6) * 0.01

        self.last_time = None
        self.last_accepted_pos = None

    def process(self, current_time, raw_x, raw_y, raw_z):
        current_pos = np.array([raw_x, raw_y, raw_z])

        # Initialization step
        if self.last_time is None:
            self.state[0:3] = current_pos
            self.last_time = current_time
            self.last_accepted_pos = current_pos
            return self.state[0], self.state[1], self.state[2]

        dt = current_time - self.last_time
        if dt <= 0: dt = 0.001

        # --- STAGE 1: OUTLIER REJECTION ---
        dist_from_last = np.linalg.norm(current_pos - self.last_accepted_pos)
        if dist_from_last > 1.5:  # If it jumps > 1.5m instantly, ignore it
            current_pos = self.state[0:3] + (self.state[3:6] * dt)
        else:
            self.last_accepted_pos = current_pos

        # --- STAGE 2: KALMAN PREDICT & UPDATE ---
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
    """
    Sends 'la' to the UWB listener, parses the anchor positions,
    and saves them to a text file.
    """
    print("[UWB] Requesting anchor positions...")
    ser.write(b'\r\r')  # Wake up shell
    time.sleep(0.2)
    ser.reset_input_buffer()  # Clear any 'lec' or 'lep' data left in the pipe
    ser.write(b'la\r')
    time.sleep(0.8)  # Give the system time to dump all anchor lines

    anchors_found = []
    lines = ser.readlines() # Read the buffer

    # Regex explained:
    # pos= matches the literal string
    # (-?\d+\.\d+) matches a positive or negative decimal number
    # : is the separator used by PANS firmware
    coord_pattern = re.compile(r"pos=(-?\d+\.\d+):(-?\d+\.\d+):(-?\d+\.\d+)")

    for line_raw in lines:
        line = line_raw.decode('utf-8', errors='ignore').strip()

        # Look for lines containing "id=" and "pos="
        if "id=" in line and "pos=" in line:
            match = coord_pattern.search(line)
            if match:
                x, y, z = match.groups()
                anchors_found.append([float(x), float(y), float(z)])
                print(f" -> Found Anchor: X={x}, Y={y}, Z={z}")

    if anchors_found:
        anchor_file_path = os.path.join(save_dir, "anchor_positions.csv")
        with open(anchor_file_path, 'w') as file:
            file.write("X,Y,Z\n")
            for a in anchors_found:
                file.write(f"{a[0]},{a[1]},{a[2]}\n")
        print(f"[UWB] Success: {len(anchors_found)} anchors saved to {anchor_file_path}")
    else:
        print("[UWB] WARNING: No anchors detected in the 'la' output.")

    ser.write(b'\r')  # Clean up shell
    return anchors_found

# ========================================================
# UWB DRIVER LOOP
# ========================================================
def run_uwb(stop_event, config, save_dir):
    port, baud = config['port1'], config['baud']

    # 1. Define both filenames in the session directory
    session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_filename = os.path.join(save_dir, f"uwb_log_{session_name}.csv")
    filt_filename = os.path.join(save_dir, f"uwbFiltered_log_{session_name}.csv")

    # 2. Initialize the filter
    kf = UWBSmoother()

    print(f"[UWB] Opening {port}...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
        update_anchor_list(ser, save_dir)
        ser.write(b'\r\r')
        time.sleep(1)
        ser.write(b'lec\r')

        # 3. Open both CSV files simultaneously
        with open(raw_filename, mode='w', newline='') as f_raw, \
                open(filt_filename, mode='w', newline='') as f_filt:

            raw_writer = csv.writer(f_raw)
            filt_writer = csv.writer(f_filt)

            # Write identical headers so main.py reads them perfectly
            header = ['Time', 'POSX', 'POSY', 'POSZ']
            raw_writer.writerow(header)
            filt_writer.writerow(header)

            while not stop_event.is_set():
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith('POS'):
                        parts = [p for p in line.split(',') if p]
                        if len(parts) >= 6:
                            arr_time = time.time()
                            raw_x, raw_y, raw_z = float(parts[3]), float(parts[4]), float(parts[5])

                            # A) Write Raw Data
                            raw_writer.writerow([arr_time, raw_x, raw_y, raw_z])
                            f_raw.flush()

                            # B) Pass through Filter
                            filt_x, filt_y, filt_z = kf.process(arr_time, raw_x, raw_y, raw_z)

                            # C) Write Filtered Data
                            filt_writer.writerow([arr_time, round(filt_x, 4), round(filt_y, 4), round(filt_z, 4)])
                            f_filt.flush()

    except Exception as e:
        print(f"[UWB Error] {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.write(b'\r')
            ser.close()
            print("[UWB] Closed.")



# # ===================================================================================================================
# Old old code
# # ===================================================================================================================

# import time
# import csv
# import os
# import serial
# from datetime import datetime
#
# def run_uwb(stop_event, config, save_dir):
#     port, baud = config['port'], config['baud']
#     session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
#     filename = os.path.join(save_dir, f"uwb_log_{session_name}.csv")
#
#     print(f"[UWB] Opening {port}...")
#     try:
#         ser = serial.Serial(port, baud, timeout=1)
#         ser.write(b'\r\r')
#         time.sleep(1)
#         ser.write(b'lec\r')
#
#         with open(filename, mode='w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(['Time', 'POSX', 'POSY', 'POSZ'])
#
#             while not stop_event.is_set():
#                 if ser.in_waiting > 0:
#                     line = ser.readline().decode('utf-8', errors='ignore').strip()
#                     if line.startswith('POS'):
#                         parts = [p for p in line.split(',') if p]
#                         if len(parts) >= 6:
#                             # Use PC time for synchronization with OptiTrack
#                             row = [time.time(), float(parts[3]), float(parts[4]), float(parts[5])]
#                             writer.writerow(row)
#                             f.flush()
#
#     except Exception as e:
#         print(f"[UWB Error] {e}")
#     finally:
#         if 'ser' in locals() and ser.is_open:
#             ser.write(b'\r')
#             ser.close()
#             print("[UWB] Closed.")