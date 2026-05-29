# ===================== PROGRAM_INFO ==================================================================================
"""
Author: Renzo Eisma
Date: 05/2026
Description:
    UWB listener logger for the DWM1001 / MDEK1001 PANS setup.

    Current supported modes:
    - One listener, tag position reading
    - Two listeners, two physical tags, midpoint/fusion of both tag positions

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
    2: [5.624, 3.116, 1.256]
}

# Position of each physical tag relative to the wanted centerpoint between the two tags.
# Centerpoint is the middle of the Tag holder and the top point the two MDEK's is the Z centerpoint
# Unit: meters
# Conversion used:
# estimated_center = measured_tag_position - tag_offset_from_center

# #For Two tag holder V3
# TAG_OFFSET_A = [-0.085, -0.125, 0.013]
# TAG_OFFSET_B = [0.085, 0.125, 0.013]

# For Two tag holder V4
TAG_OFFSET_A = [-0.0185, -0.125, 0.013]
TAG_OFFSET_B = [0.0185, 0.125, 0.013]

TWO_TAGS_SWAPPED_ON_HOLDER = True

if TWO_TAGS_SWAPPED_ON_HOLDER:
    DEFAULT_TAG_OFFSETS_FROM_CENTER = {
        1: TAG_OFFSET_B,
        2: TAG_OFFSET_A
    }
else:
    DEFAULT_TAG_OFFSETS_FROM_CENTER = {
        1: TAG_OFFSET_A,
        2: TAG_OFFSET_B
    }

# For two physical tags, midpoint is the geometrically correct default.
# Quality is still used for validity checks, output quality and optional fallback behaviour.
# Change to "weighted" only if you intentionally want the better-quality tag to pull the result toward itself.
DEFAULT_TWO_TAG_FUSION_METHOD = "midpoint"       # "midpoint" or "weighted"
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
            "tag2_quality": result.get("tag2_quality")
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

        # Ignore invalid PANS positions like:
        # POS,0,4D18,nan,nan,nan,0,x01
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            return None

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


# Parses the estimated tag position from a DWM1001 'les' style line.
# This is mostly kept for compatibility/debugging.
# Example:
# est[1.90,1.96,0.15,91]
# ---------------------------------------------------------------------------------------------------------------------
def parse_est_position(line):
    match = re.search(
        r"est\[\s*(-?\d+(?:\.\d+)?)\s*,\s*"
        r"(-?\d+(?:\.\d+)?)\s*,\s*"
        r"(-?\d+(?:\.\d+)?)\s*,\s*"
        r"(-?\d+(?:\.\d+)?)\s*\]",
        line
    )

    if not match:
        return None

    x, y, z, quality = match.groups()

    return {
        "tag_id": None,
        "position": [float(x), float(y), float(z)],
        "quality": float(quality),
        "position_type": "tag_position_est"
    }


# Parses one UWB line in Tag Position mode.
# ---------------------------------------------------------------------------------------------------------------------
def parse_tag_position_line(line):
    # Ignore command echoes and prompt-only lines.
    stripped = line.strip().lower()
    if stripped in ["lec", "les", "lep", "dwm>", "dwm> lec", "dwm> les", "dwm> lep"]:
        return "ignore"

    pos_result = parse_pos_csv_position(line)
    if pos_result is not None:
        return pos_result

    compact_result = parse_compact_listener_position(line)
    if compact_result is not None:
        return compact_result

    est_result = parse_est_position(line)
    if est_result is not None:
        return est_result

    return None


def should_ignore_serial_line(line):
    clean = line.strip().lower()

    if clean in ["", "lec", "les", "lep", "c"]:
        return True

    if clean == "dwm>":
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


# Converts a measured tag position to the estimated centerpoint position.
# tag_offset_from_center is the known physical position of the tag relative to the wanted centerpoint.
#
# Example:
# tag is 1 cm left of center: tag_offset_from_center = [-0.01, 0.0, 0.0]
# center_estimate = measured_tag_position - [-0.01, 0.0, 0.0]
# center_estimate = measured_tag_position + [0.01, 0.0, 0.0]
# ---------------------------------------------------------------------------------------------------------------------
def convert_tag_position_to_center_position(tag_position, tag_offset_from_center):
    tag_position = to_vector(tag_position)
    tag_offset_from_center = to_vector(tag_offset_from_center)

    center_position = tag_position - tag_offset_from_center

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
#
# Method:
# - each listener/tag position is first converted to an estimated centerpoint position
# - if both tags are valid, the two centerpoint estimates are fused
# - the fusion is quality-weighted, so the tag with better quality has more influence
# - if only one tag is valid, its converted centerpoint estimate is used
# - if no tags are valid, no output is produced
# ---------------------------------------------------------------------------------------------------------------------
def fuse_two_listener_tag_positions(
        latest_positions,
        current_timestamp,
        combine_window,
        fusion_method=DEFAULT_TWO_TAG_FUSION_METHOD,
        allow_single_listener_fallback=DEFAULT_ALLOW_SINGLE_LISTENER_FALLBACK):

    fresh = get_fresh_listener_results(latest_positions, current_timestamp, combine_window)

    tag1 = fresh.get(1)
    tag2 = fresh.get(2)

    # -------------------------------------------------------------
    # Convert available tag positions to centerpoint estimates
    # -------------------------------------------------------------
    tag1_center_position = None
    tag2_center_position = None

    if tag1 is not None:
        tag1_center_position = convert_tag_position_to_center_position(
            tag1["aligned_position"],
            DEFAULT_TAG_OFFSETS_FROM_CENTER.get(1, [0.0, 0.0, 0.0])
        )

    if tag2 is not None:
        tag2_center_position = convert_tag_position_to_center_position(
            tag2["aligned_position"],
            DEFAULT_TAG_OFFSETS_FROM_CENTER.get(2, [0.0, 0.0, 0.0])
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

        # Quality-weighted centerpoint fusion.
        # Example:
        # q1 = 50, q2 = 100
        # result = 33% tag1 center estimate + 66% tag2 center estimate
        if (w1 + w2) > 0:
            fused_position = ((w1 * p1) + (w2 * p2)) / (w1 + w2)
            fusion_mode = "two_tag_center_quality_weighted"
        else:
            fused_position = (p1 + p2) / 2.0
            fusion_mode = "two_tag_center_midpoint_no_quality"

        quality_values = [q for q in [q1, q2] if q is not None]
        fused_quality = float(np.mean(quality_values)) if quality_values else None

        return {
            "timestamp": current_timestamp,
            "position": [float(fused_position[0]), float(fused_position[1]), float(fused_position[2])],
            "quality": fused_quality,
            "listener_id": 0,
            "network_id": 0,
            "position_type": "two_listener_fused_center_position",
            "fusion_mode": fusion_mode,

            # Centerpoint estimates after correcting for physical tag placement
            "tag1_position": tag1_center_position,
            "tag2_position": tag2_center_position,

            # Original aligned UWB tag positions before physical tag offset correction
            "tag1_aligned_position": tag1.get("aligned_position"),
            "tag2_aligned_position": tag2.get("aligned_position"),

            # Original raw listener positions before network/listener coordinate offset
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
        fallback = tag1 if tag1_center_position is not None else tag2
        fallback_center_position = tag1_center_position if tag1_center_position is not None else tag2_center_position

        if fallback is not None and fallback_center_position is not None:
            listener_id = fallback.get("listener_id")

            return {
                "timestamp": current_timestamp,
                "position": fallback_center_position,
                "quality": fallback.get("quality"),
                "listener_id": listener_id,
                "network_id": fallback.get("network_id"),
                "position_type": f"two_listener_center_fallback_l{listener_id}",
                "fusion_mode": "single_listener_center_fallback",

                # Centerpoint estimates after correcting for physical tag placement
                "tag1_position": tag1_center_position,
                "tag2_position": tag2_center_position,

                # Original aligned UWB tag positions before physical tag offset correction
                "tag1_aligned_position": tag1.get("aligned_position") if tag1 is not None else None,
                "tag2_aligned_position": tag2.get("aligned_position") if tag2 is not None else None,

                # Original raw listener positions before network/listener coordinate offset
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
            ser.write(b'\r')
            time.sleep(0.2)
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


# Writes extra information for debugging two-listener fusion.
# The report maker can ignore this file.
# ---------------------------------------------------------------------------------------------------------------------
def write_two_listener_debug(debug_writer, debug_file, result):
    if debug_writer is None:
        return

    tag1 = result.get("tag1_position") or ["", "", ""]
    tag2 = result.get("tag2_position") or ["", "", ""]
    fused = result.get("position") or ["", "", ""]

    debug_writer.writerow([
        result.get("timestamp"),
        tag1[0], tag1[1], tag1[2], result.get("tag1_quality"), result.get("tag1_id"),
        tag2[0], tag2[1], tag2[2], result.get("tag2_quality"), result.get("tag2_id"),
        fused[0], fused[1], fused[2], result.get("quality"),
        result.get("fusion_mode"), result.get("position_type")
    ])

    debug_file.flush()


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

    fusion_method = config.get('two_tag_fusion_method', DEFAULT_TWO_TAG_FUSION_METHOD)
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
            print(f"[UWB] Fusion method: {fusion_method}")
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
                'Tag1_X', 'Tag1_Y', 'Tag1_Z', 'Tag1_Quality', 'Tag1_ID',
                'Tag2_X', 'Tag2_Y', 'Tag2_Z', 'Tag2_Quality', 'Tag2_ID',
                'Fused_X', 'Fused_Y', 'Fused_Z', 'Fused_Quality',
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
                    print(f"[UWB_RAW][L{listener_id}] {line}")

                    try:
                        parsed = parse_tag_position_line(line)

                        if parsed == "ignore":
                            continue

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
                                fusion_method=fusion_method,
                                allow_single_listener_fallback=allow_single_listener_fallback
                            )

                            if final_result is not None:
                                pos = final_result["position"]
                                quality = final_result.get("quality")
                                mode = final_result.get("fusion_mode")

                                quality_text = f"{quality:.1f}" if quality is not None else "N/A"

                                print(
                                    f"[UWB_CALC] "
                                    f"X={pos[0]:.3f}, "
                                    f"Y={pos[1]:.3f}, "
                                    f"Z={pos[2]:.3f}, "
                                    f"Q={quality_text}"
                                )

                                continue

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
