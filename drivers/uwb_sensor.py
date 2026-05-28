# ===================== PROGRAM_INFO ==================================================================================
"""
Author: Renzo Eisma
Date: 04/2026
Description: UWB Sensor Measurement with Dynamic Data Routing

This script is responsible for reading UWB data from one or two listener modules.
It writes the final UWB position to a CSV file, sends live UWB data to MATLAB over UDP,
and sends live UWB data to MasterControlStation for visualization.

MasterControlStation is responsible for session/settings packets.
This script is responsible only for UWB measurement data.
"""
# =====================================================================================================================


# =====================================================================================================================
# IMPORTS
# =====================================================================================================================
import serial
import time
import csv
import os
import re
import socket
from datetime import datetime

import numpy as np


# =====================================================================================================================
# UDP CONFIGURATION
# =====================================================================================================================
UDP_IP = "127.0.0.1"      # Default localhost. Can be overwritten by config from MasterControlStation.
UDP_PORT_UWB = 5005       # UWB live data port for MatlabMasterControl.


# =====================================================================================================================
# HELPER FUNCTIONS
# =====================================================================================================================

# Checks whether a value can safely be converted to a float.
# ---------------------------------------------------------------------------------------------------------------------
def is_float(value):
    try:
        float(value)
        return True
    except (TypeError, ValueError):
        return False


# Converts a quality value to a weight. This is used for combining two networks or weighting distance measurements.
# The DWM1001 quality number is normally between 0 and 100. If no quality is available, weight 1 is used.
# ---------------------------------------------------------------------------------------------------------------------
def quality_to_weight(quality):
    if quality is None:
        return 1.0

    try:
        quality = float(quality)
    except (TypeError, ValueError):
        return 1.0

    # Keep the weight from becoming zero. This avoids divide-by-zero problems and still gives bad data a low weight.
    return max(quality, 1.0) / 100.0


# Sends the final UWB position packet to MATLAB.
# Packet format:
# timestamp,x,y,z,quality,listener_id,network_id,position_type
# ---------------------------------------------------------------------------------------------------------------------
def send_uwb_udp(sock, matlab_host, matlab_port, result):
    quality = "" if result.get("quality") is None else result.get("quality")

    msg = (
        f"{result['timestamp']},"
        f"{result['position'][0]},"
        f"{result['position'][1]},"
        f"{result['position'][2]},"
        f"{quality},"
        f"{result.get('listener_id', '')},"
        f"{result.get('network_id', '')},"
        f"{result.get('position_type', '')}\n"
    )

    sock.sendto(msg.encode("utf-8"), (matlab_host, matlab_port))


# Sends the final UWB position to MasterControlStation for the live plot.
# This uses the new dictionary format. MasterControlStation still supports the old tuple format too.
# ---------------------------------------------------------------------------------------------------------------------
def send_to_master_queue(data_queue, result):
    if data_queue is None:
        return

    x, y, z = result["position"]

    data_queue.put({
        "source": "uwb",
        "source_type": "listener_serial",
        "data_type": "position",
        "timestamp": result["timestamp"],
        "pc_timestamp": result["timestamp"],
        "position": {
            "x": x,
            "y": y,
            "z": z
        },
        "quality": {
            "valid": True,
            "accuracy": result.get("quality")
        },
        "metadata": {
            "listener_id": result.get("listener_id"),
            "network_id": result.get("network_id"),
            "position_type": result.get("position_type")
        }
    })


# Logs parser or measurement errors to a separate error CSV.
# ---------------------------------------------------------------------------------------------------------------------
def log_error(error_writer, error_file, timestamp, port, listener_id, network_id, line, message):
    print(f"[UWB ERROR] {message}")

    if error_writer is not None:
        error_writer.writerow([timestamp, port, listener_id, network_id, line, message])
        error_file.flush()


# =====================================================================================================================
# PARSING FUNCTIONS
# =====================================================================================================================

# Extracts the estimated tag position from a DWM1001 'les' style line.
# Example from PANS / DWM1001 shell output:
# CD37[0.00,0.00,0.00]=2.80 ... est[1.90,1.96,0.15,91]
# ---------------------------------------------------------------------------------------------------------------------
def parse_est_position(line):
    match = re.search(
        r"est\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\]",
        line
    )

    if not match:
        return None

    x, y, z, quality = match.groups()

    return {
        "position": [float(x), float(y), float(z)],
        "quality": float(quality),
        "position_type": "tag_position_est"
    }


# Extracts the tag position from the CSV-style PANS output.
# This parser is intentionally flexible because different firmware/settings can slightly change the exact output.
# Example that the previous script handled:
# POS,1,0A,1.23,2.34,0.85,91,...
# ---------------------------------------------------------------------------------------------------------------------
def parse_pos_csv_position(line):
    if "POS" not in line:
        return None

    try:
        # Keep only the part after POS. This also handles command echoes before POS.
        data_part = line.split("POS", 1)[-1]
        parts = [p.strip() for p in data_part.split(',') if p.strip()]

        # Candidate layouts. The first candidate matches the current old script:
        # parts = [index, tag_id, x, y, z, quality, ...]
        candidates = [
            (2, 3, 4, 5),
            (3, 4, 5, 6),
            (0, 1, 2, 3),
            (1, 2, 3, 4)
        ]

        for ix, iy, iz, iq in candidates:
            if len(parts) > iz and is_float(parts[ix]) and is_float(parts[iy]) and is_float(parts[iz]):
                x = float(parts[ix])
                y = float(parts[iy])
                z = float(parts[iz])

                quality = None
                if len(parts) > iq and is_float(parts[iq]):
                    quality = float(parts[iq])

                return {
                    "position": [x, y, z],
                    "quality": quality,
                    "position_type": "tag_position"
                }

    except Exception:
        return None

    return None


# Parses anchor distance measurements from a DWM1001 'les' style line.
# Example:
# CD37[0.00,0.00,0.00]=2.80 1495[0.00,3.99,0.00]=2.74 ... est[1.90,1.96,0.15,91]
#
# Each returned measurement contains:
# id, anchor_position, distance, weight
# ---------------------------------------------------------------------------------------------------------------------
def parse_anchor_distances_with_positions(line, quality=None):
    measurements = []

    # This matches anchor_id[x,y,z]=distance. Anchor IDs sometimes contain a space in copied terminal output.
    pattern = re.compile(
        r"(?P<id>[0-9A-Fa-f ]{1,10})\[\s*"
        r"(?P<x>-?\d+(?:\.\d+)?)\s*,\s*"
        r"(?P<y>-?\d+(?:\.\d+)?)\s*,\s*"
        r"(?P<z>-?\d+(?:\.\d+)?)\s*\]\s*=\s*"
        r"(?P<dist>-?\d+(?:\.\d+)?)"
    )

    for match in pattern.finditer(line):
        anchor_id = match.group("id").replace(" ", "").strip()

        # Avoid accidentally treating an empty/invalid id as a real anchor.
        if not anchor_id:
            anchor_id = f"anchor_{len(measurements) + 1}"

        measurements.append({
            "id": anchor_id,
            "anchor_position": [
                float(match.group("x")),
                float(match.group("y")),
                float(match.group("z"))
            ],
            "distance": float(match.group("dist")),
            "weight": quality_to_weight(quality)
        })

    return measurements


# Parses simple ANx,distance style values when they exist in the output. These do not contain anchor coordinates,
# so they are not enough for custom triangulation. They are only used to detect why distance mode failed.
# ---------------------------------------------------------------------------------------------------------------------
def parse_distances_without_positions(line):
    distances = []
    parts = [p.strip() for p in line.split(',') if p.strip()]

    for i in range(len(parts) - 1):
        if parts[i].upper().startswith('AN') and is_float(parts[i + 1]):
            distances.append({
                "id": parts[i],
                "distance": float(parts[i + 1])
            })

    return distances


# Parses one UWB line in Tag Position mode.
# ---------------------------------------------------------------------------------------------------------------------
def parse_tag_position_line(line):
    # Prefer the CSV-style POS output if available.
    pos_result = parse_pos_csv_position(line)
    if pos_result is not None:
        return pos_result

    # Also support the 'les' style est[x,y,z,q] position.
    est_result = parse_est_position(line)
    if est_result is not None:
        return est_result

    return None


# Parses one UWB line in Tag Distances mode.
# This mode requires anchor coordinates and distances, otherwise custom triangulation is not possible.
# ---------------------------------------------------------------------------------------------------------------------
def parse_distance_line(line):
    est_result = parse_est_position(line)
    quality = est_result.get("quality") if est_result else None

    measurements = parse_anchor_distances_with_positions(line, quality=quality)

    return {
        "measurements": measurements,
        "quality": quality,
        "fallback_position": est_result
    }


# =====================================================================================================================
# 3D DISTANCE TRIANGULATION
# =====================================================================================================================

# Calculates a 3D position from anchor positions and distances using iterative weighted least squares.
# At least four anchor distances are required for a useful 3D solution.
# ---------------------------------------------------------------------------------------------------------------------
def triangulate_3d_from_distances(measurements, initial_position=None, max_iterations=25, tolerance=0.0005):
    if len(measurements) < 4:
        raise ValueError("At least 4 anchor distances are needed for 3D triangulation.")

    anchors = np.array([m["anchor_position"] for m in measurements], dtype=float)
    ranges = np.array([m["distance"] for m in measurements], dtype=float)
    weights = np.array([m.get("weight", 1.0) for m in measurements], dtype=float)

    # Avoid zero weights because they make the weighted least-squares matrix unstable.
    weights[weights <= 0] = 0.01

    if initial_position is not None:
        x = np.array(initial_position, dtype=float)
    else:
        x = np.mean(anchors, axis=0)

    for _ in range(max_iterations):
        diff = x - anchors
        predicted_ranges = np.linalg.norm(diff, axis=1)
        predicted_ranges[predicted_ranges < 1e-6] = 1e-6

        residual = ranges - predicted_ranges
        h_matrix = -diff / predicted_ranges[:, None]

        w_matrix = np.diag(weights)
        lhs = h_matrix.T @ w_matrix @ h_matrix
        rhs = h_matrix.T @ w_matrix @ residual

        # pinv is used instead of inv because poor anchor geometry can make the matrix singular or near-singular.
        delta = np.linalg.pinv(lhs) @ rhs
        x = x + delta

        if np.linalg.norm(delta) < tolerance:
            break

    return [float(x[0]), float(x[1]), float(x[2])]


# Combines two already calculated positions by weighted averaging.
# This is used for Tag Position mode when two listener networks are active.
# ---------------------------------------------------------------------------------------------------------------------
def weighted_average_positions(results):
    valid_results = [r for r in results if r is not None and r.get("position") is not None]

    if not valid_results:
        return None

    if len(valid_results) == 1:
        return valid_results[0]

    weights = np.array([quality_to_weight(r.get("quality")) for r in valid_results], dtype=float)
    positions = np.array([r["position"] for r in valid_results], dtype=float)

    combined_position = np.average(positions, axis=0, weights=weights)

    quality_values = [r.get("quality") for r in valid_results if r.get("quality") is not None]
    combined_quality = None
    if quality_values:
        combined_quality = float(np.average(quality_values, weights=weights[:len(quality_values)]))

    return {
        "timestamp": max(r["timestamp"] for r in valid_results),
        "position": [float(combined_position[0]), float(combined_position[1]), float(combined_position[2])],
        "quality": combined_quality,
        "listener_id": 0,
        "network_id": 0,
        "position_type": "weighted_two_network_position"
    }


# =====================================================================================================================
# SERIAL / PANS SHELL FUNCTIONS
# =====================================================================================================================

# Opens a serial connection to the listener module.
# ---------------------------------------------------------------------------------------------------------------------
def open_serial_port(port, baud):
    ser = serial.Serial(port, baud, timeout=1, dsrdtr=False, rtscts=False)
    ser.dtr = False
    ser.rts = False
    return ser


# Wakes the DWM1001 shell and clears old serial data.
# This version is intentionally close to the old working startup sequence.
# ---------------------------------------------------------------------------------------------------------------------
def wake_shell(ser):
    print(f"[UWB] Waking shell on {ser.port}...")

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    # Stop/settle existing stream
    ser.write(b'\r')
    time.sleep(1.0)

    # Clear old output
    ser.read_all()

    # Wake shell
    ser.write(b'\r\r')
    time.sleep(1.0)


# Starts the UWB stream.
# This version first checks whether data is already streaming, similar to the old script.
# ---------------------------------------------------------------------------------------------------------------------
def start_stream(ser, command):
    # Give the module a moment after previous shell commands such as 'la'
    time.sleep(0.5)

    if ser.in_waiting > 15:
        print(f"[UWB] {ser.port} is already streaming data. Skipping '{command}' command.")
        ser.reset_input_buffer()
        return

    print(f"[UWB] {ser.port} state unclear or not streaming. Sending '{command}' command...")
    ser.write((command + '\r').encode('utf-8'))
    time.sleep(1.0)

    # If still no data, try waking the shell once more and send the command again.
    if ser.in_waiting == 0:
        print(f"[UWB] No stream detected after first '{command}'. Retrying...")
        ser.write(b'\r\r')
        time.sleep(0.5)
        ser.write((command + '\r').encode('utf-8'))
        time.sleep(1.0)


# Stops the UWB stream before closing the port.
# ---------------------------------------------------------------------------------------------------------------------
def stop_stream(ser):
    try:
        if ser is not None and ser.is_open:
            ser.write(b'\r')
            time.sleep(0.2)
    except Exception:
        pass


# Requests anchor positions using the 'la' command and writes them to [Log]_anchor_positions.csv.
# This is mostly useful for documentation and report plotting.
# ---------------------------------------------------------------------------------------------------------------------
def update_anchor_list(ser, save_dir):
    print(f"[UWB] Requesting anchor positions from {ser.port}...")

    try:
        ser.reset_input_buffer()
        ser.write(b'la\r')
        time.sleep(0.5)

        raw_text = ser.read_all().decode('utf-8', errors='ignore')
        lines = raw_text.split('\n')

        anchors_found = []

        # Supports pos=x:y:z style output.
        coord_pattern = re.compile(r"pos=(-?\d+(?:\.\d+)?):(-?\d+(?:\.\d+)?):(-?\d+(?:\.\d+)?)")

        for line in lines:
            line = line.strip()
            if "id=" in line and "pos=" in line:
                match = coord_pattern.search(line)
                if match:
                    x, y, z = match.groups()
                    anchor_position = [float(x), float(y), float(z)]
                    anchors_found.append(anchor_position)
                    print(f" -> Found Anchor: X={x}, Y={y}, Z={z}")

        if anchors_found:
            anchor_file_path = os.path.join(save_dir, "[Log]_anchor_positions.csv")

            # If the file already exists, append only new unique positions.
            existing = []
            if os.path.exists(anchor_file_path):
                try:
                    with open(anchor_file_path, 'r') as file:
                        next(file, None)
                        for row in file:
                            values = [float(v) for v in row.strip().split(',')]
                            if len(values) == 3:
                                existing.append(values)
                except Exception:
                    existing = []

            with open(anchor_file_path, 'w') as file:
                file.write("X,Y,Z\n")
                unique_anchors = []

                for anchor in existing + anchors_found:
                    if anchor not in unique_anchors:
                        unique_anchors.append(anchor)
                        file.write(f"{anchor[0]},{anchor[1]},{anchor[2]}\n")

            print(f"[UWB] Success: {len(anchors_found)} anchors detected on {ser.port}.")

        else:
            print("[UWB] WARNING: No anchors detected in the 'la' output.")

        return anchors_found

    except Exception as e:
        print(f"[UWB] WARNING: Could not update anchor list on {ser.port}: {e}")
        return []


# =====================================================================================================================
# FINAL POSITION BUILDING
# =====================================================================================================================

# Builds a final position result in Tag Position mode. If two networks are selected, the latest fresh positions from
# both networks are combined using weighted averaging based on the quality value.
# ---------------------------------------------------------------------------------------------------------------------
def build_final_position_from_tag_positions(latest_positions, current_result, two_network_mode, combine_window):
    if not two_network_mode:
        return current_result

    now = current_result["timestamp"]

    fresh_results = []
    for result in latest_positions.values():
        if result is not None and abs(now - result["timestamp"]) <= combine_window:
            fresh_results.append(result)

    if not fresh_results:
        return current_result

    return weighted_average_positions(fresh_results)


# Builds a final position result in Tag Distance mode. If two networks are active, fresh measurements from both networks
# are combined into one larger anchor-distance set before triangulation.
# ---------------------------------------------------------------------------------------------------------------------
def build_final_position_from_distances(latest_distances, current_result, two_network_mode, combine_window, previous_position):
    now = current_result["timestamp"]

    distance_sets = []

    if two_network_mode:
        for result in latest_distances.values():
            if result is not None and abs(now - result["timestamp"]) <= combine_window:
                distance_sets.append(result)
    else:
        distance_sets.append(current_result)

    all_measurements = []
    quality_values = []

    for result in distance_sets:
        all_measurements.extend(result.get("measurements", []))
        if result.get("quality") is not None:
            quality_values.append(result.get("quality"))

    if len(all_measurements) < 4:
        raise ValueError(f"Not enough anchor distances for 3D triangulation. Got {len(all_measurements)}, need at least 4.")

    position = triangulate_3d_from_distances(all_measurements, initial_position=previous_position)

    quality = None
    if quality_values:
        quality = float(np.mean(quality_values))

    return {
        "timestamp": now,
        "position": position,
        "quality": quality,
        "listener_id": 0 if two_network_mode else current_result.get("listener_id"),
        "network_id": 0 if two_network_mode else current_result.get("network_id"),
        "position_type": "triangulated_distances_combined" if two_network_mode else "triangulated_distances"
    }


# =====================================================================================================================
# UWB DRIVER LOOP
# =====================================================================================================================

def run_uwb(stop_event, config, save_dir, data_queue=None):
    # 1. Unpack Master Configuration
    # -----------------------------------------------------------------------------------------------------------------
    port1 = config.get('port1')
    port2 = config.get('port2')
    baud = config.get('baud', 115200)

    read_type = config.get('read_type', 'Tag Position')
    network_scale = config.get('network_scale', config.get('anchor_count', '1 Network / 1 Listener'))
    two_network_mode = "2" in str(network_scale) or "8 Anchors" in str(network_scale)

    send_matlab = config.get('send_matlab', False)
    matlab_host = config.get('matlab_host', UDP_IP)
    matlab_port = config.get('matlab_uwb_port', UDP_PORT_UWB)

    session_name = config.get('session_name', datetime.now().strftime("%Y%m%d_%H%M%S"))
    abs_save_dir = os.path.abspath(save_dir)

    # Shell commands. These can be overwritten from MasterControlStation later if needed.
    position_command = config.get('position_command', 'lec')
    distance_command = config.get('distance_command', 'les')

    combine_window = config.get('combine_window', 0.5)

    print(f"[UWB] Mode: {read_type}")
    print(f"[UWB] Network Scale: {network_scale}")
    print(f"[UWB] MATLAB Live UDP: {send_matlab} -> {matlab_host}:{matlab_port}")

    # 2. Determine number of listeners
    # -----------------------------------------------------------------------------------------------------------------
    ports_to_open = []

    if port1:
        ports_to_open.append(port1)

    if two_network_mode and port2:
        ports_to_open.append(port2)
        print(f"[UWB] Dual listener mode activated. Ports: {port1}, {port2}")

    if not ports_to_open:
        print("[UWB] No UWB ports configured. UWB reader will not start.")
        return

    # 3. Create files and runtime objects
    # -----------------------------------------------------------------------------------------------------------------
    active_serials = []
    buffers = []
    listener_info = []

    latest_positions = {}
    latest_distances = {}
    previous_distance_position = None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    data_file = None
    data_writer = None
    error_file = None
    error_writer = None

    try:
        # The report maker currently expects four columns. Extra diagnostic data is sent to MATLAB but not stored here.
        # This file contains the final UWB position no matter whether it comes from tag position mode, custom
        # triangulation, or two-network weighted combination.
        data_path = os.path.join(abs_save_dir, f"[Log]_uwb_listener1_{session_name}.csv")
        data_file = open(data_path, 'w', newline='')
        data_writer = csv.writer(data_file)
        data_writer.writerow(['Time', 'POSX', 'POSY', 'POSZ'])

        error_path = os.path.join(abs_save_dir, f"[Log]_errors_uwb_{session_name}.csv")
        error_file = open(error_path, 'w', newline='')
        error_writer = csv.writer(error_file)
        error_writer.writerow(['Time', 'Port', 'ListenerID', 'NetworkID', 'Line', 'Error'])

        # 4. Setup Serial Ports
        # -------------------------------------------------------------------------------------------------------------
        for index, port in enumerate(ports_to_open):
            listener_id = index + 1
            network_id = index + 1

            print(f"[UWB] Opening listener {listener_id} on {port}...")
            ser = open_serial_port(port, baud)

            active_serials.append(ser)
            buffers.append("")
            listener_info.append({
                "listener_id": listener_id,
                "network_id": network_id,
                "port": port
            })

            wake_shell(ser)
            update_anchor_list(ser, abs_save_dir)

            if read_type == 'Tag Distances':
                start_stream(ser, distance_command)
            else:
                start_stream(ser, position_command)

        print("[UWB] Listening for data...")

        # 5. Main Processing Loop
        # -------------------------------------------------------------------------------------------------------------
        while not stop_event.is_set():
            for i, ser in enumerate(active_serials):
                info = listener_info[i]
                listener_id = info["listener_id"]
                network_id = info["network_id"]
                port = info["port"]

                if ser.in_waiting <= 0:
                    continue

                new_data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffers[i] += new_data

                if '\n' not in buffers[i]:
                    continue

                lines = buffers[i].split('\n')
                buffers[i] = lines.pop()

                for line in lines:
                    line = line.strip()
                    if not line:
                        continue

                    # Ignore pure prompt lines.
                    if line.strip() == "dwm>":
                        continue

                    arr_time = time.time()
                    print(f"[UWB_RAW][L{listener_id}] {line}")

                    try:
                        # ==========================================
                        # MODE A: TAG POSITION
                        # ==========================================
                        if read_type == 'Tag Position':
                            parsed = parse_tag_position_line(line)

                            if parsed is None:
                                log_error(error_writer, error_file, arr_time, port, listener_id, network_id, line,
                                          "Could not parse tag position line.")
                                continue

                            current_result = {
                                "timestamp": arr_time,
                                "position": parsed["position"],
                                "quality": parsed.get("quality"),
                                "listener_id": listener_id,
                                "network_id": network_id,
                                "position_type": parsed.get("position_type", "tag_position")
                            }

                            latest_positions[listener_id] = current_result
                            final_result = build_final_position_from_tag_positions(
                                latest_positions,
                                current_result,
                                two_network_mode,
                                combine_window
                            )

                        # ==========================================
                        # MODE B: TAG DISTANCES
                        # ==========================================
                        elif read_type == 'Tag Distances':
                            parsed = parse_distance_line(line)
                            measurements = parsed.get("measurements", [])

                            if not measurements:
                                simple_distances = parse_distances_without_positions(line)
                                if simple_distances:
                                    log_error(error_writer, error_file, arr_time, port, listener_id, network_id, line,
                                              "Distances found, but no anchor coordinates were included. Cannot triangulate 3D position.")
                                else:
                                    log_error(error_writer, error_file, arr_time, port, listener_id, network_id, line,
                                              "Could not parse anchor distances.")
                                continue

                            current_distance_result = {
                                "timestamp": arr_time,
                                "measurements": measurements,
                                "quality": parsed.get("quality"),
                                "listener_id": listener_id,
                                "network_id": network_id
                            }

                            latest_distances[listener_id] = current_distance_result
                            final_result = build_final_position_from_distances(
                                latest_distances,
                                current_distance_result,
                                two_network_mode,
                                combine_window,
                                previous_distance_position
                            )
                            previous_distance_position = final_result["position"]

                        else:
                            log_error(error_writer, error_file, arr_time, port, listener_id, network_id, line,
                                      f"Unknown read_type selected: {read_type}")
                            continue

                        # ==========================================
                        # Log and route the final UWB position
                        # ==========================================
                        x, y, z = final_result["position"]

                        data_writer.writerow([final_result["timestamp"], x, y, z])
                        data_file.flush()

                        send_to_master_queue(data_queue, final_result)

                        if send_matlab:
                            send_uwb_udp(sock, matlab_host, matlab_port, final_result)

                    except Exception as e:
                        log_error(error_writer, error_file, arr_time, port, listener_id, network_id, line, str(e))

            time.sleep(0.001)

    # =================================================================================================================
    # End Program
    # =================================================================================================================
    except Exception as e:
        print(f"[UWB Error] {e}")

    finally:
        print("[UWB] Closing UWB drivers...")

        for ser in active_serials:
            try:
                if ser.is_open:
                    stop_stream(ser)
                    ser.close()
            except Exception:
                pass

        if data_file is not None:
            data_file.close()

        if error_file is not None:
            error_file.close()

        try:
            sock.close()
        except Exception:
            pass

        print("[UWB] Drivers Closed.")
