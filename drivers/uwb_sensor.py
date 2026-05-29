# ===================== PROGRAM_INFO ==================================================================================
"""
Author: Renzo Eisma
Date: 05/2026
Description:
    UWB listener logger for the DWM1001 / MDEK1001 PANS setup.

    Current supported modes:
    - One listener, tag position reading
    - Two listeners, two physical tags, quality-weighted fusion of both tag positions

    Future reserved modes:
    - One listener, distance/range reading after firmware is reprogrammed
    - Two listeners, distance/range reading after firmware is reprogrammed

    MasterControlStation is responsible for session/settings packets.
    This script is responsible for UWB measurement data only:
    - reading UWB listener serial data
    - writing final UWB position CSVs
    - writing diagnostic/error CSVs
    - sending live UWB data to MATLAB
    - sending live UWB data to MasterControlStation for the live plot
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
import math
from datetime import datetime

import numpy as np


# =====================================================================================================================
# DEFAULT SETTINGS
# =====================================================================================================================
UDP_IP = "127.0.0.1"
UDP_PORT_UWB = 5005

DEFAULT_BAUD = 115200
DEFAULT_POSITION_COMMAND = "lec"
DEFAULT_COMBINE_WINDOW = 0.5

# Translation offsets used to align both UWB network coordinate frames.
# aligned_position = raw_position + listener_offset
# If both networks already use the same origin and axes, keep these at zero.
DEFAULT_LISTENER_OFFSETS = {
    1: [0.0, 0.0, 0.0],
    2: [-5.624, -3.116, -1.256]
}

# Position of each physical tag relative to the wanted centerpoint between the two tags.
# Centerpoint is the middle of the tag holder and the top point of the two MDEK's is the Z centerpoint.
# Unit: meters
# Conversion used:
# estimated_center = measured_tag_position - rotated_tag_offset_from_center

    # For Two tag holder V3
# TAG_OFFSET_A = [-0.085, -0.125, 0.013]
# TAG_OFFSET_B = [0.085, 0.125, 0.013]

    # For Two tag holder V4
TAG_OFFSET_A = [-0.0185, -0.125, 0.013]
TAG_OFFSET_B = [0.0185, 0.125, 0.013]

TWO_TAGS_SWAPPED_ON_HOLDER = False

DEFAULT_TAG_OFFSETS_FROM_CENTER = {1: TAG_OFFSET_A, 2: TAG_OFFSET_B}

# Listener/tag role definition.
# Change these if listener 1 and listener 2 are physically swapped.
FRONT_TAG_LISTENER_ID = 1
BACK_TAG_LISTENER_ID = 2

# If true, the tag-to-center offsets are rotated using the current angle between the front and back tag.
USE_ROTATED_TAG_OFFSETS = True

# If only one tag is available, the script can use the latest known angle.
# If the angle is older than this, fallback becomes less reliable.
MAX_YAW_AGE_FOR_SINGLE_TAG_FALLBACK = 1.0

DEFAULT_MIN_VALID_QUALITY = 1.0
DEFAULT_ALLOW_SINGLE_LISTENER_FALLBACK = True


# =====================================================================================================================
# BASIC HELPERS
# =====================================================================================================================
# Checks whether a value can safely be converted to a float.
# ---------------------------------------------------------------------------------------------------------------------
def is_float(value):
    try:
        value = float(value)
        return not math.isnan(value) and not math.isinf(value)
    except (TypeError, ValueError):
        return False


# Converts a quality value to a fusion weight.
# PANS quality is normally higher = better. Values <= 0 are treated as invalid/very weak.
# ---------------------------------------------------------------------------------------------------------------------
def quality_to_weight(quality):
    if quality is None:
        return 1.0

    try:
        quality = float(quality)
    except (TypeError, ValueError):
        return 1.0

    if math.isnan(quality) or math.isinf(quality):
        return 0.0

    return max(quality, 0.0) / 100.0


# Safely converts [x, y, z] to a numpy vector.
# ---------------------------------------------------------------------------------------------------------------------
def to_vector(position):
    return np.array(position, dtype=float)


# Checks whether a parsed UWB position can be used.
# ---------------------------------------------------------------------------------------------------------------------
def is_valid_position(position, quality=None, min_quality=DEFAULT_MIN_VALID_QUALITY):
    if position is None or len(position) < 3:
        return False

    try:
        values = [float(position[0]), float(position[1]), float(position[2])]
    except (TypeError, ValueError):
        return False

    if any(math.isnan(v) or math.isinf(v) for v in values):
        return False

    if quality is not None:
        try:
            q = float(quality)
            if math.isnan(q) or math.isinf(q):
                return False
            if q < min_quality:
                return False
        except (TypeError, ValueError):
            return False

    return True


# =====================================================================================================================
# MATLAB / GUI OUTPUT FUNCTIONS
# =====================================================================================================================
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
            "position_type": result.get("position_type"),
            "fusion_mode": result.get("fusion_mode"),
            "tag1_position": result.get("tag1_position"),
            "tag2_position": result.get("tag2_position"),
            "tag1_quality": result.get("tag1_quality"),
            "tag2_quality": result.get("tag2_quality"),
            "holder_yaw_deg": result.get("holder_yaw_deg"),
            "yaw_source": result.get("yaw_source")
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
# Parses the CSV-style PANS listener position output.
# Common example:
# POS,0,4D18,-1.31,0.17,0.39,47,x01
# ---------------------------------------------------------------------------------------------------------------------
def parse_pos_csv_position(line):
    # Find POS, anywhere in the line.
    # This handles dirty serial lines such as:
    # lePOS,0,4CAD,4.56,3.29,1.54,90,x08
    pos_index = line.find("POS,")

    if pos_index == -1:
        return None

    try:
        line = line[pos_index:].strip()
        parts = [p.strip() for p in line.split(",")]

        # Expected PANS lec listener format:
        # POS,index,tag_id,x,y,z,quality,checksum
        if len(parts) < 7:
            return None

        tag_id = parts[2]

        if not is_float(parts[3]) or not is_float(parts[4]) or not is_float(parts[5]):
            return None

        x = float(parts[3])
        y = float(parts[4])
        z = float(parts[5])

        quality = None
        if len(parts) > 6 and is_float(parts[6]):
            quality = float(parts[6])

        return {
            "tag_id": tag_id,
            "position": [x, y, z],
            "quality": quality,
            "position_type": "tag_position"
        }

    except Exception:
        return None


# Parses compact listener output.
# Example:
# 0) 4CAD[-1.31,0.17,0.39,47,x0C]
# ---------------------------------------------------------------------------------------------------------------------
def parse_compact_listener_position(line):
    match = re.search(
        r"\)\s*(?P<tag_id>[0-9A-Fa-f]+)\[\s*"
        r"(?P<x>-?\d+(?:\.\d+)?)\s*,\s*"
        r"(?P<y>-?\d+(?:\.\d+)?)\s*,\s*"
        r"(?P<z>-?\d+(?:\.\d+)?)\s*,\s*"
        r"(?P<quality>-?\d+(?:\.\d+)?)",
        line
    )

    if not match:
        return None

    return {
        "tag_id": match.group("tag_id"),
        "position": [
            float(match.group("x")),
            float(match.group("y")),
            float(match.group("z"))
        ],
        "quality": float(match.group("quality")),
        "position_type": "compact_listener_position"
    }


# Parses one UWB line in Tag Position mode.
# ---------------------------------------------------------------------------------------------------------------------
def parse_tag_position_line(line):
    pos_result = parse_pos_csv_position(line)
    if pos_result is not None:
        return pos_result

    compact_result = parse_compact_listener_position(line)
    if compact_result is not None:
        return compact_result

    return None


# Checks whether a serial line is only a shell prompt or command echo.
# ---------------------------------------------------------------------------------------------------------------------
def should_ignore_serial_line(line):
    clean = line.strip().lower()

    if clean in ["", "lec", "les", "lep", "c", "dwm>"]:
        return True

    if clean in ["dwm> lec", "dwm> les", "dwm> lep"]:
        return True

    # Do not ignore dirty lines that still contain useful POS data.
    # Example: lePOS,0,4CAD,...
    if "pos," in clean:
        return False

    return False


# =====================================================================================================================
# COORDINATE ALIGNMENT AND TWO-TAG FUSION
# =====================================================================================================================
# Applies a simple translation offset to align a listener/network coordinate frame.
# ---------------------------------------------------------------------------------------------------------------------
def apply_listener_offset(position, offset):
    return (to_vector(position) + to_vector(offset)).tolist()


# Keeps an angle inside the range -pi to pi.
# ---------------------------------------------------------------------------------------------------------------------
def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# Calculates the holder/drone yaw from the measured front and back tag positions.
# The measured world angle is compared to the configured local tag-offset angle.
# ---------------------------------------------------------------------------------------------------------------------
def calculate_holder_yaw(front_position, back_position, front_offset_from_center, back_offset_from_center):
    front_position = to_vector(front_position)
    back_position = to_vector(back_position)

    front_offset_from_center = to_vector(front_offset_from_center)
    back_offset_from_center = to_vector(back_offset_from_center)

    world_line = front_position - back_position
    local_line = front_offset_from_center - back_offset_from_center

    if np.linalg.norm(world_line[0:2]) < 1e-6:
        return None

    if np.linalg.norm(local_line[0:2]) < 1e-6:
        return None

    world_line_angle = math.atan2(world_line[1], world_line[0])
    local_line_angle = math.atan2(local_line[1], local_line[0])

    holder_yaw = normalize_angle(world_line_angle - local_line_angle)

    return holder_yaw


# Rotates a tag offset from holder/body coordinates into world coordinates.
# Only X/Y are rotated. Z is kept the same.
# ---------------------------------------------------------------------------------------------------------------------
def rotate_offset_to_world(tag_offset_from_center, holder_yaw):
    offset = to_vector(tag_offset_from_center)

    c = math.cos(holder_yaw)
    s = math.sin(holder_yaw)

    rotated_x = c * offset[0] - s * offset[1]
    rotated_y = s * offset[0] + c * offset[1]
    rotated_z = offset[2]

    return np.array([rotated_x, rotated_y, rotated_z], dtype=float)


# Converts a measured tag position to the estimated centerpoint position.
# ---------------------------------------------------------------------------------------------------------------------
def convert_tag_position_to_center_position(tag_position, tag_offset_from_center, holder_yaw=None):
    tag_position = to_vector(tag_position)

    if USE_ROTATED_TAG_OFFSETS and holder_yaw is not None:
        tag_offset_world = rotate_offset_to_world(tag_offset_from_center, holder_yaw)
    else:
        tag_offset_world = to_vector(tag_offset_from_center)

    center_position = tag_position - tag_offset_world

    return [
        float(center_position[0]),
        float(center_position[1]),
        float(center_position[2])
    ]


# Returns the freshest valid listener results inside the combine window.
# ---------------------------------------------------------------------------------------------------------------------
def get_fresh_listener_results(latest_positions, now, combine_window):
    fresh_results = {}

    for listener_id, result in latest_positions.items():
        if result is None:
            continue

        if abs(now - result["timestamp"]) <= combine_window:
            fresh_results[listener_id] = result

    return fresh_results


# Fuses two physical tags into one raw UWB centerpoint.
# ---------------------------------------------------------------------------------------------------------------------
def fuse_two_listener_tag_positions(
        latest_positions,
        current_timestamp,
        combine_window,
        fusion_state,
        allow_single_listener_fallback=DEFAULT_ALLOW_SINGLE_LISTENER_FALLBACK):

    fresh = get_fresh_listener_results(latest_positions, current_timestamp, combine_window)

    tag1 = fresh.get(1)
    tag2 = fresh.get(2)

    front_tag = fresh.get(FRONT_TAG_LISTENER_ID)
    back_tag = fresh.get(BACK_TAG_LISTENER_ID)

    holder_yaw = None
    yaw_source = "none"

    # -------------------------------------------------------------
    # Calculate current holder yaw when both front and back tags exist
    # -------------------------------------------------------------
    if front_tag is not None and back_tag is not None:
        front_offset = DEFAULT_TAG_OFFSETS_FROM_CENTER.get(FRONT_TAG_LISTENER_ID, [0.0, 0.0, 0.0])
        back_offset = DEFAULT_TAG_OFFSETS_FROM_CENTER.get(BACK_TAG_LISTENER_ID, [0.0, 0.0, 0.0])

        holder_yaw = calculate_holder_yaw(
            front_tag["aligned_position"],
            back_tag["aligned_position"],
            front_offset,
            back_offset
        )

        if holder_yaw is not None:
            fusion_state["latest_yaw"] = holder_yaw
            fusion_state["latest_yaw_time"] = current_timestamp
            yaw_source = "current_two_tags"

    # -------------------------------------------------------------
    # If only one tag is available, use latest known yaw if it is recent
    # -------------------------------------------------------------
    if holder_yaw is None:
        latest_yaw = fusion_state.get("latest_yaw")
        latest_yaw_time = fusion_state.get("latest_yaw_time")

        if latest_yaw is not None and latest_yaw_time is not None:
            yaw_age = current_timestamp - latest_yaw_time

            if yaw_age <= MAX_YAW_AGE_FOR_SINGLE_TAG_FALLBACK:
                holder_yaw = latest_yaw
                yaw_source = "latest_known_yaw"

    # -------------------------------------------------------------
    # Convert available tag positions to centerpoint estimates
    # -------------------------------------------------------------
    tag1_center_position = None
    tag2_center_position = None

    if tag1 is not None:
        tag1_center_position = convert_tag_position_to_center_position(
            tag1["aligned_position"],
            DEFAULT_TAG_OFFSETS_FROM_CENTER.get(1, [0.0, 0.0, 0.0]),
            holder_yaw
        )

    if tag2 is not None:
        tag2_center_position = convert_tag_position_to_center_position(
            tag2["aligned_position"],
            DEFAULT_TAG_OFFSETS_FROM_CENTER.get(2, [0.0, 0.0, 0.0]),
            holder_yaw
        )

    # -------------------------------------------------------------
    # Both listener/tag positions are available
    # -------------------------------------------------------------
    if tag1_center_position is not None and tag2_center_position is not None:
        p1 = to_vector(tag1_center_position)
        p2 = to_vector(tag2_center_position)

        q1 = tag1.get("quality")
        q2 = tag2.get("quality")

        w1 = quality_to_weight(q1)
        w2 = quality_to_weight(q2)

        if (w1 + w2) > 0:
            fused_position = ((w1 * p1) + (w2 * p2)) / (w1 + w2)
            fusion_mode = "two_tag_rotated_center_quality_weighted"
        else:
            fused_position = (p1 + p2) / 2.0
            fusion_mode = "two_tag_rotated_center_average_no_quality"

        quality_values = [q for q in [q1, q2] if q is not None]
        fused_quality = float(np.mean(quality_values)) if quality_values else None

        holder_yaw_deg = None
        if holder_yaw is not None:
            holder_yaw_deg = math.degrees(holder_yaw)

        return {
            "timestamp": current_timestamp,
            "position": [float(fused_position[0]), float(fused_position[1]), float(fused_position[2])],
            "quality": fused_quality,
            "listener_id": 0,
            "network_id": 0,
            "position_type": "two_listener_fused_center_position",
            "fusion_mode": fusion_mode,
            "holder_yaw_rad": holder_yaw,
            "holder_yaw_deg": holder_yaw_deg,
            "yaw_source": yaw_source,
            "tag1_position": tag1_center_position,
            "tag2_position": tag2_center_position,
            "tag1_aligned_position": tag1.get("aligned_position"),
            "tag2_aligned_position": tag2.get("aligned_position"),
            "tag1_raw_position": tag1.get("raw_position"),
            "tag2_raw_position": tag2.get("raw_position"),
            "tag1_quality": q1,
            "tag2_quality": q2,
            "tag1_id": tag1.get("tag_id"),
            "tag2_id": tag2.get("tag_id")
        }

    # -------------------------------------------------------------
    # Fallback if only one listener/tag is available
    # -------------------------------------------------------------
    if allow_single_listener_fallback:
        # When rotated tag offsets are enabled, one-tag fallback is only safe if a recent yaw is available.
        if USE_ROTATED_TAG_OFFSETS and holder_yaw is None:
            return None

        fallback = tag1 if tag1_center_position is not None else tag2
        fallback_center_position = tag1_center_position if tag1_center_position is not None else tag2_center_position

        if fallback is not None and fallback_center_position is not None:
            listener_id = fallback.get("listener_id")

            holder_yaw_deg = None
            if holder_yaw is not None:
                holder_yaw_deg = math.degrees(holder_yaw)

            return {
                "timestamp": current_timestamp,
                "position": fallback_center_position,
                "quality": fallback.get("quality"),
                "listener_id": listener_id,
                "network_id": fallback.get("network_id"),
                "position_type": f"two_listener_center_fallback_l{listener_id}",
                "fusion_mode": "single_listener_rotated_center_fallback",
                "holder_yaw_rad": holder_yaw,
                "holder_yaw_deg": holder_yaw_deg,
                "yaw_source": yaw_source,
                "tag1_position": tag1_center_position,
                "tag2_position": tag2_center_position,
                "tag1_aligned_position": tag1.get("aligned_position") if tag1 is not None else None,
                "tag2_aligned_position": tag2.get("aligned_position") if tag2 is not None else None,
                "tag1_raw_position": tag1.get("raw_position") if tag1 is not None else None,
                "tag2_raw_position": tag2.get("raw_position") if tag2 is not None else None,
                "tag1_quality": tag1.get("quality") if tag1 is not None else None,
                "tag2_quality": tag2.get("quality") if tag2 is not None else None,
                "tag1_id": tag1.get("tag_id") if tag1 is not None else None,
                "tag2_id": tag2.get("tag_id") if tag2 is not None else None
            }

    return None


# =====================================================================================================================
# FUTURE DISTANCE / RANGE MODE PLACEHOLDERS
# =====================================================================================================================
# Future mode for custom firmware:
# One listener reads individual anchor distances and triangulates one position.
# ---------------------------------------------------------------------------------------------------------------------
def run_single_listener_distance_mode(*args, **kwargs):
    # [Insert single-listener distance reading here when custom firmware exposes raw ranges.]
    # Expected future pipeline:
    # listener -> anchor ranges -> range validation -> 3D triangulation -> final UWB position
    raise NotImplementedError("Single-listener distance mode is reserved for future custom firmware work.")


# Future mode for custom firmware:
# Two listeners read individual anchor distances and combine/triangulate into one position.
# ---------------------------------------------------------------------------------------------------------------------
def run_two_listener_distance_mode(*args, **kwargs):
    # [Insert two-listener distance reading/fusion here when custom firmware exposes raw ranges.]
    # Expected future pipeline:
    # listener 1 ranges + listener 2 ranges -> coordinate alignment -> range fusion/triangulation -> final UWB position
    raise NotImplementedError("Two-listener distance mode is reserved for future custom firmware work.")


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
# ---------------------------------------------------------------------------------------------------------------------
def wake_shell(ser):
    print(f"[UWB] Waking shell on {ser.port}...")

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    ser.write(b'\r')
    time.sleep(1.0)
    ser.read_all()

    ser.write(b'\r\r')
    time.sleep(1.0)
    ser.read_all()


# Starts the PANS listener position stream.
# ---------------------------------------------------------------------------------------------------------------------
def start_position_stream(ser, command=DEFAULT_POSITION_COMMAND):
    try:
        if ser.in_waiting > 15:
            print(f"[UWB] {ser.port} is already streaming data. Skipping '{command}' command.")
            return
    except Exception:
        pass

    print(f"[UWB] Starting stream on {ser.port} with command: {command}")

    try:
        ser.write((command + '\r').encode('utf-8'))
        time.sleep(1.0)

        if ser.in_waiting <= 0:
            print(f"[UWB] No data after first '{command}' command on {ser.port}. Retrying...")
            ser.write((command + '\r').encode('utf-8'))
            time.sleep(1.0)

    except Exception as e:
        print(f"[UWB] Failed to start stream on {ser.port}: {e}")


# Stops the UWB stream before closing the port.
# ---------------------------------------------------------------------------------------------------------------------
def stop_stream(ser):
    try:
        if ser is not None and ser.is_open:
            print(f"[UWB] Stopping stream on {ser.port}...")

            # Send multiple enters to break out of a running stream and return to shell.
            ser.write(b'\r')
            time.sleep(0.2)
            ser.write(b'\r')
            time.sleep(0.2)

            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass

    except Exception:
        pass


# Requests anchor positions using the 'la' command and writes them to [Log]_anchor_positions.csv.
# This is useful for documentation and report plotting.
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
# CSV WRITING FUNCTIONS
# =====================================================================================================================
# Writes the final UWB position CSV used by the report maker.
# ---------------------------------------------------------------------------------------------------------------------
def write_final_position(data_writer, data_file, result):
    x, y, z = result["position"]
    data_writer.writerow([result["timestamp"], x, y, z])
    data_file.flush()


# Converts a vector for debug CSV writing.
# ---------------------------------------------------------------------------------------------------------------------
def debug_vec(result, key):
    vec = result.get(key)
    if vec is None:
        return ["", "", ""]
    return [vec[0], vec[1], vec[2]]


# Writes extra information for debugging two-listener fusion.
# The report maker can ignore this file.
# ---------------------------------------------------------------------------------------------------------------------
def write_two_listener_debug(debug_writer, debug_file, result):
    if debug_writer is None:
        return

    tag1_raw = debug_vec(result, "tag1_raw_position")
    tag1_aligned = debug_vec(result, "tag1_aligned_position")
    tag1_center = debug_vec(result, "tag1_position")

    tag2_raw = debug_vec(result, "tag2_raw_position")
    tag2_aligned = debug_vec(result, "tag2_aligned_position")
    tag2_center = debug_vec(result, "tag2_position")

    fused = result.get("position") or ["", "", ""]

    debug_writer.writerow([
        result.get("timestamp"),
        tag1_raw[0], tag1_raw[1], tag1_raw[2],
        tag1_aligned[0], tag1_aligned[1], tag1_aligned[2],
        tag1_center[0], tag1_center[1], tag1_center[2],
        result.get("tag1_quality"), result.get("tag1_id"),
        tag2_raw[0], tag2_raw[1], tag2_raw[2],
        tag2_aligned[0], tag2_aligned[1], tag2_aligned[2],
        tag2_center[0], tag2_center[1], tag2_center[2],
        result.get("tag2_quality"), result.get("tag2_id"),
        fused[0], fused[1], fused[2], result.get("quality"),
        result.get("holder_yaw_rad"),
        result.get("holder_yaw_deg"),
        result.get("yaw_source"),
        result.get("fusion_mode"),
        result.get("position_type")
    ])

    debug_file.flush()


# Prints the two-listener fusion steps to the GUI terminal.
# ---------------------------------------------------------------------------------------------------------------------
def print_two_listener_fusion_status(result):

    def fmt_vec(vec):
        if vec is None:
            return "None"

        try:
            return f"({float(vec[0]):.3f}, {float(vec[1]):.3f}, {float(vec[2]):.3f})"
        except Exception:
            return "Invalid"

    def fmt_quality(q):
        if q is None:
            return "N/A"

        try:
            return f"{float(q):.1f}"
        except Exception:
            return "N/A"

    def fmt_percent(value):
        try:
            return f"{100.0 * float(value):.1f}%"
        except Exception:
            return "N/A"

    tag1_aligned = result.get("tag1_aligned_position")
    tag2_aligned = result.get("tag2_aligned_position")

    tag1_center = result.get("tag1_position")
    tag2_center = result.get("tag2_position")

    calculated = result.get("position")

    q1 = result.get("tag1_quality")
    q2 = result.get("tag2_quality")
    q_calc = result.get("quality")

    w1 = quality_to_weight(q1)
    w2 = quality_to_weight(q2)

    if (w1 + w2) > 0:
        p1 = w1 / (w1 + w2)
        p2 = w2 / (w1 + w2)
    else:
        p1 = 0.5
        p2 = 0.5

    yaw_deg = result.get("holder_yaw_deg")
    yaw_text = f"{yaw_deg:.1f} deg" if yaw_deg is not None else "N/A"

    mode = result.get("fusion_mode")

    print(
        "[UWB_FUSION] "
        f"L1 raw={fmt_vec(tag1_aligned)} -> center={fmt_vec(tag1_center)} q={fmt_quality(q1)} pull={fmt_percent(p1)} | "
        f"L2 raw={fmt_vec(tag2_aligned)} -> center={fmt_vec(tag2_center)} q={fmt_quality(q2)} pull={fmt_percent(p2)} | "
        f"CALC={fmt_vec(calculated)} q={fmt_quality(q_calc)} yaw={yaw_text} mode={mode}"
    )


# =====================================================================================================================
# MAIN UWB DRIVER LOOP
# =====================================================================================================================
def run_uwb(stop_event, config, save_dir, data_queue=None):
    # 1. Unpack MasterControlStation configuration
    # -----------------------------------------------------------------------------------------------------------------
    port1 = config.get('port1')
    port2 = config.get('port2')
    baud = config.get('baud', DEFAULT_BAUD)

    read_type = config.get('read_type', 'Tag Position')
    network_scale = config.get('network_scale', config.get('anchor_count', '1 Network / 1 Listener'))
    two_listener_mode = "2" in str(network_scale) or "8 Anchors" in str(network_scale)

    send_matlab = config.get('send_matlab', False)
    matlab_host = config.get('matlab_host', UDP_IP)
    matlab_port = config.get('matlab_uwb_port', UDP_PORT_UWB)

    session_name = config.get('session_name', datetime.now().strftime("%Y%m%d_%H%M%S"))
    abs_save_dir = os.path.abspath(save_dir)

    position_command = config.get('position_command', DEFAULT_POSITION_COMMAND)
    combine_window = float(config.get('combine_window', DEFAULT_COMBINE_WINDOW))

    listener_offsets = config.get('listener_offsets', DEFAULT_LISTENER_OFFSETS)
    listener_1_offset = config.get('listener_1_offset', listener_offsets.get(1, DEFAULT_LISTENER_OFFSETS[1]))
    listener_2_offset = config.get('listener_2_offset', listener_offsets.get(2, DEFAULT_LISTENER_OFFSETS[2]))

    min_valid_quality = float(config.get('min_valid_quality', DEFAULT_MIN_VALID_QUALITY))
    allow_single_listener_fallback = bool(config.get('allow_single_listener_fallback', DEFAULT_ALLOW_SINGLE_LISTENER_FALLBACK))

    print(f"[UWB] Mode: {read_type}")
    print(f"[UWB] Network Scale: {network_scale}")
    print(f"[UWB] MATLAB Live UDP: {send_matlab} -> {matlab_host}:{matlab_port}")

    if read_type != 'Tag Position':
        print("[UWB] Distance/range modes are reserved for future custom firmware work.")
        print("[UWB] Current supported mode is: Tag Position")
        return

    # 2. Determine listener ports
    # -----------------------------------------------------------------------------------------------------------------
    ports_to_open = []

    if port1:
        ports_to_open.append({
            "port": port1,
            "listener_id": 1,
            "network_id": 1,
            "offset": listener_1_offset
        })

    if two_listener_mode:
        if port2:
            ports_to_open.append({
                "port": port2,
                "listener_id": 2,
                "network_id": 2,
                "offset": listener_2_offset
            })
            print(f"[UWB] Two-listener two-tag fusion enabled. Ports: {port1}, {port2}")
            print(f"[UWB] Listener 1 offset: {listener_1_offset}")
            print(f"[UWB] Listener 2 offset: {listener_2_offset}")
            print(f"[UWB] Combine window: {combine_window} s")
        else:
            print("[UWB] WARNING: Two-listener mode selected, but port2 is not configured.")
            print("[UWB] Falling back to one-listener mode.")
            two_listener_mode = False

    if not ports_to_open:
        print("[UWB] No UWB ports configured. UWB reader will not start.")
        return

    # 3. Create files and runtime objects
    # -----------------------------------------------------------------------------------------------------------------
    active_serials = []
    buffers = []
    listener_info = []
    latest_positions = {}

    fusion_state = {
        "latest_yaw": None,
        "latest_yaw_time": None
    }

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    data_file = None
    data_writer = None
    error_file = None
    error_writer = None
    debug_file = None
    debug_writer = None

    try:
        # Final report-compatible UWB position CSV.
        data_path = os.path.join(abs_save_dir, f"[Log]_uwb_listener1_{session_name}.csv")
        data_file = open(data_path, 'w', newline='')
        data_writer = csv.writer(data_file)
        data_writer.writerow(['Time', 'POSX', 'POSY', 'POSZ'])

        # Error/debug CSV.
        error_path = os.path.join(abs_save_dir, f"[Log]_uwb_errors_{session_name}.csv")
        error_file = open(error_path, 'w', newline='')
        error_writer = csv.writer(error_file)
        error_writer.writerow(['Time', 'Port', 'ListenerID', 'NetworkID', 'Line', 'Error'])

        # Two-listener fusion debug CSV.
        if two_listener_mode:
            debug_path = os.path.join(abs_save_dir, f"[Log]_uwb_two_listener_debug_{session_name}.csv")
            debug_file = open(debug_path, 'w', newline='')
            debug_writer = csv.writer(debug_file)
            debug_writer.writerow([
                'Time',
                'Tag1_Raw_X', 'Tag1_Raw_Y', 'Tag1_Raw_Z',
                'Tag1_Aligned_X', 'Tag1_Aligned_Y', 'Tag1_Aligned_Z',
                'Tag1_Center_X', 'Tag1_Center_Y', 'Tag1_Center_Z',
                'Tag1_Quality', 'Tag1_ID',
                'Tag2_Raw_X', 'Tag2_Raw_Y', 'Tag2_Raw_Z',
                'Tag2_Aligned_X', 'Tag2_Aligned_Y', 'Tag2_Aligned_Z',
                'Tag2_Center_X', 'Tag2_Center_Y', 'Tag2_Center_Z',
                'Tag2_Quality', 'Tag2_ID',
                'Fused_X', 'Fused_Y', 'Fused_Z', 'Fused_Quality',
                'HolderYawRad', 'HolderYawDeg', 'YawSource',
                'FusionMode', 'PositionType'
            ])

        # 4. Setup serial ports
        # -------------------------------------------------------------------------------------------------------------
        for info in ports_to_open:
            port = info["port"]
            listener_id = info["listener_id"]

            print(f"[UWB] Opening listener {listener_id} on {port}...")

            ser = open_serial_port(port, baud)
            active_serials.append(ser)
            buffers.append("")
            listener_info.append(info)

            wake_shell(ser)
            update_anchor_list(ser, abs_save_dir)

        for ser, info in zip(active_serials, listener_info):
            listener_id = info["listener_id"]

            print(f"[UWB] Starting listener {listener_id} stream...")
            start_position_stream(ser, position_command)

        print("[UWB] Listening for position data...")

        # 5. Main processing loop
        # -------------------------------------------------------------------------------------------------------------
        while not stop_event.is_set():
            for i, ser in enumerate(active_serials):
                info = listener_info[i]
                listener_id = info["listener_id"]
                network_id = info["network_id"]
                port = info["port"]
                offset = info["offset"]

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

                    if should_ignore_serial_line(line):
                        continue

                    arr_time = time.time()
                    if not two_listener_mode:
                        print(f"[UWB_RAW][L{listener_id}] {line}")

                    try:
                        parsed = parse_tag_position_line(line)

                        if parsed is None:
                            log_error(
                                error_writer, error_file, arr_time,
                                port, listener_id, network_id, line,
                                "Could not parse tag position line."
                            )
                            continue

                        raw_position = parsed.get("position")
                        quality = parsed.get("quality")

                        if not is_valid_position(raw_position, quality=quality, min_quality=min_valid_quality):
                            log_error(
                                error_writer, error_file, arr_time,
                                port, listener_id, network_id, line,
                                "Invalid UWB position or quality."
                            )
                            continue

                        aligned_position = apply_listener_offset(raw_position, offset)

                        current_result = {
                            "timestamp": arr_time,
                            "raw_position": raw_position,
                            "aligned_position": aligned_position,
                            "position": aligned_position,
                            "quality": quality,
                            "listener_id": listener_id,
                            "network_id": network_id,
                            "tag_id": parsed.get("tag_id"),
                            "position_type": parsed.get("position_type", "tag_position")
                        }

                        latest_positions[listener_id] = current_result

                        # ------------------------------------------
                        # Mode A: one listener position reading
                        # ------------------------------------------
                        if not two_listener_mode:
                            final_result = {
                                "timestamp": arr_time,
                                "position": aligned_position,
                                "quality": quality,
                                "listener_id": listener_id,
                                "network_id": network_id,
                                "position_type": "single_listener_position",
                                "fusion_mode": "single_listener"
                            }

                        # ------------------------------------------
                        # Mode B: two listeners, two physical tags
                        # ------------------------------------------
                        else:
                            final_result = fuse_two_listener_tag_positions(
                                latest_positions=latest_positions,
                                current_timestamp=arr_time,
                                combine_window=combine_window,
                                fusion_state=fusion_state,
                                allow_single_listener_fallback=allow_single_listener_fallback
                            )

                        if final_result is None:
                            continue

                        # ------------------------------------------
                        # Print useful debug information
                        # ------------------------------------------
                        if two_listener_mode:
                            print_two_listener_fusion_status(final_result)
                        else:
                            pos = final_result["position"]
                            quality = final_result.get("quality")
                            quality_text = f"{quality:.1f}" if quality is not None else "N/A"

                            print(
                                f"[UWB_SINGLE] "
                                f"X={pos[0]:.3f}, "
                                f"Y={pos[1]:.3f}, "
                                f"Z={pos[2]:.3f}, "
                                f"Q={quality_text}"
                            )

                        # ------------------------------------------
                        # Log and route final UWB position
                        # ------------------------------------------
                        write_final_position(data_writer, data_file, final_result)

                        if two_listener_mode:
                            write_two_listener_debug(debug_writer, debug_file, final_result)

                        send_to_master_queue(data_queue, final_result)

                        if send_matlab:
                            send_uwb_udp(sock, matlab_host, matlab_port, final_result)

                    except Exception as e:
                        log_error(error_writer, error_file, arr_time, port, listener_id, network_id, line, str(e))

            time.sleep(0.001)

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

        if debug_file is not None:
            debug_file.close()

        try:
            sock.close()
        except Exception:
            pass

        print("[UWB] Drivers Closed.")
