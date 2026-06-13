# !!!!!!!!!!!!!!!!!!!!!
# !!!NOTE!!!, this script is made up of two scripts. One permanent one for reading MDEK listeners and one temporary one for
# reading data from the custom UWB PCB. It isn't very clean, so in the future it should be in two seperate scripts.
# !!!!!!!!!!!!!!!!!!!!!
#
# # ===================== PROGRAM_INFO ==================================================================================
# """
# Author: Renzo Eisma
# Date: 05/2026
# Description:
#     UWB listener logger for the DWM1001 / MDEK1001 PANS setup.
#
#     Current supported modes:
#     - One listener, tag position reading
#     - Two listeners, two physical tags, quality-weighted fusion of both tag positions
#
#     Future reserved modes:
#     - One listener, distance/range reading after firmware is reprogrammed
#     - Two listeners, distance/range reading after firmware is reprogrammed
#
#     MasterControlStation is responsible for session/settings packets.
#     This script is responsible for UWB measurement data only:
#     - reading UWB listener serial data
#     - writing final UWB position CSVs
#     - writing diagnostic/error CSVs
#     - sending live UWB data to MATLAB
#     - sending live UWB data to MasterControlStation for the live plot
#
# Assistance note: ChatGPT Pro 5.5 Thinking Extended was used to clean up variable names, comments, line spacing, and
# added in depth logging in the terminal. The rewritten version was checked by a human. The original concept, code logic,
# and project structure were created by Renzo Eisma.
# """
# # =====================================================================================================================
#
#
# # =====================================================================================================================
# # IMPORTS
# # =====================================================================================================================
# import serial
# import time
# import csv
# import os
# import re
# import socket
# import math
# from datetime import datetime
#
# import numpy as np
#
#
# # =====================================================================================================================
# # DEFAULT SETTINGS
# # =====================================================================================================================
# UDP_IP = "127.0.0.1"
# UDP_PORT_UWB = 5005
#
# DEFAULT_BAUD = 115200
# DEFAULT_POSITION_COMMAND = "lec"
# DEFAULT_COMBINE_WINDOW = 0.5
#
# # Translation offsets used to align both UWB network coordinate frames.
# # aligned_position = raw_position + listener_offset
# # If both networks already use the same origin and axes, keep these at zero.
#
# DEFAULT_LISTENER_OFFSETS = {
#     1: [0.0, 0.0, 0.0],
#     2: [0,0,0] #-5.624, -3.116, -1.256
# }
#
# # Position of each physical tag relative to the wanted centerpoint between the two tags.
# # Centerpoint is the middle of the tag holder and the top point of the two MDEK's is the Z centerpoint.
# # Unit: meters
# # Conversion used:
# # estimated_center = measured_tag_position - rotated_tag_offset_from_center
#
#     # For Two tag holder V3
# # TAG_OFFSET_A = [-0.085, -0.125, 0.013]
# # TAG_OFFSET_B = [0.085, 0.125, 0.013]
#
#     # For Two tag holder V4
# TAG_OFFSET_A = [-0.0185, -0.125, 0.013]
# TAG_OFFSET_B = [0.0185, 0.125, 0.013]
#
# TWO_TAGS_SWAPPED_ON_HOLDER = False
#
# DEFAULT_TAG_OFFSETS_FROM_CENTER = {1: TAG_OFFSET_A, 2: TAG_OFFSET_B}
#
# # Listener/tag role definition.
# # Change these if listener 1 and listener 2 are physically swapped.
# FRONT_TAG_LISTENER_ID = 1
# BACK_TAG_LISTENER_ID = 2
#
# # If true, the tag-to-center offsets are rotated using the current angle between the front and back tag.
# USE_ROTATED_TAG_OFFSETS = True
#
# # If only one tag is available, the script can use the latest known angle.
# # If the angle is older than this, fallback becomes less reliable.
# MAX_YAW_AGE_FOR_SINGLE_TAG_FALLBACK = 1.0
#
# DEFAULT_MIN_VALID_QUALITY = 1.0
# DEFAULT_ALLOW_SINGLE_LISTENER_FALLBACK = True
#
#
# # =====================================================================================================================
# # BASIC HELPERS
# # =====================================================================================================================
# # Checks whether a value can safely be converted to a float.
# # ---------------------------------------------------------------------------------------------------------------------
# def is_float(value):
#     try:
#         value = float(value)
#         return not math.isnan(value) and not math.isinf(value)
#     except (TypeError, ValueError):
#         return False
#
#
# # Converts a quality value to a fusion weight.
# # PANS quality is normally higher = better. Values <= 0 are treated as invalid/very weak.
# # ---------------------------------------------------------------------------------------------------------------------
# def quality_to_weight(quality):
#     if quality is None:
#         return 1.0
#
#     try:
#         quality = float(quality)
#     except (TypeError, ValueError):
#         return 1.0
#
#     if math.isnan(quality) or math.isinf(quality):
#         return 0.0
#
#     return max(quality, 0.0) / 100.0
#
#
# # Safely converts [x, y, z] to a numpy vector.
# # ---------------------------------------------------------------------------------------------------------------------
# def to_vector(position):
#     return np.array(position, dtype=float)
#
#
# # Checks whether a parsed UWB position can be used.
# # ---------------------------------------------------------------------------------------------------------------------
# def is_valid_position(position, quality=None, min_quality=DEFAULT_MIN_VALID_QUALITY):
#     if position is None or len(position) < 3:
#         return False
#
#     try:
#         values = [float(position[0]), float(position[1]), float(position[2])]
#     except (TypeError, ValueError):
#         return False
#
#     if any(math.isnan(v) or math.isinf(v) for v in values):
#         return False
#
#     if quality is not None:
#         try:
#             q = float(quality)
#             if math.isnan(q) or math.isinf(q):
#                 return False
#             if q < min_quality:
#                 return False
#         except (TypeError, ValueError):
#             return False
#
#     return True
#
#
# # =====================================================================================================================
# # MATLAB / GUI OUTPUT FUNCTIONS
# # =====================================================================================================================
# # Sends the final UWB position packet to MATLAB.
# # Packet format:
# # timestamp,x,y,z,quality,listener_id,network_id,position_type
# # ---------------------------------------------------------------------------------------------------------------------
# def send_uwb_udp(sock, matlab_host, matlab_port, result):
#     quality = "" if result.get("quality") is None else result.get("quality")
#
#     msg = (
#         f"{result['timestamp']},"
#         f"{result['position'][0]},"
#         f"{result['position'][1]},"
#         f"{result['position'][2]},"
#         f"{quality},"
#         f"{result.get('listener_id', '')},"
#         f"{result.get('network_id', '')},"
#         f"{result.get('position_type', '')}\n"
#     )
#
#     sock.sendto(msg.encode("utf-8"), (matlab_host, matlab_port))
#
#
# # Sends the final UWB position to MasterControlStation for the live plot.
# # ---------------------------------------------------------------------------------------------------------------------
# def send_to_master_queue(data_queue, result):
#     if data_queue is None:
#         return
#
#     x, y, z = result["position"]
#
#     data_queue.put({
#         "source": "uwb",
#         "source_type": "listener_serial",
#         "data_type": "position",
#         "timestamp": result["timestamp"],
#         "pc_timestamp": result["timestamp"],
#         "position": {
#             "x": x,
#             "y": y,
#             "z": z
#         },
#         "quality": {
#             "valid": True,
#             "accuracy": result.get("quality")
#         },
#         "metadata": {
#             "listener_id": result.get("listener_id"),
#             "network_id": result.get("network_id"),
#             "position_type": result.get("position_type"),
#             "fusion_mode": result.get("fusion_mode"),
#             "tag1_position": result.get("tag1_position"),
#             "tag2_position": result.get("tag2_position"),
#             "tag1_quality": result.get("tag1_quality"),
#             "tag2_quality": result.get("tag2_quality"),
#             "holder_yaw_deg": result.get("holder_yaw_deg"),
#             "yaw_source": result.get("yaw_source")
#         }
#     })
#
#
# # Logs parser or measurement errors to a separate error CSV.
# # ---------------------------------------------------------------------------------------------------------------------
# def log_error(error_writer, error_file, timestamp, port, listener_id, network_id, line, message):
#     print(f"[UWB ERROR] {message}")
#
#     if error_writer is not None:
#         error_writer.writerow([timestamp, port, listener_id, network_id, line, message])
#         error_file.flush()
#
#
# # =====================================================================================================================
# # PARSING FUNCTIONS
# # =====================================================================================================================
# # Parses the CSV-style PANS listener position output.
# # Common example:
# # POS,0,4D18,-1.31,0.17,0.39,47,x01
# # ---------------------------------------------------------------------------------------------------------------------
# def parse_pos_csv_position(line):
#     # Find POS, anywhere in the line.
#     # This handles dirty serial lines such as:
#     # lePOS,0,4CAD,4.56,3.29,1.54,90,x08
#     pos_index = line.find("POS,")
#
#     if pos_index == -1:
#         return None
#
#     try:
#         line = line[pos_index:].strip()
#         parts = [p.strip() for p in line.split(",")]
#
#         # Expected PANS lec listener format:
#         # POS,index,tag_id,x,y,z,quality,checksum
#         if len(parts) < 7:
#             return None
#
#         tag_id = parts[2]
#
#         if not is_float(parts[3]) or not is_float(parts[4]) or not is_float(parts[5]):
#             return None
#
#         x = float(parts[3])
#         y = float(parts[4])
#         z = float(parts[5])
#
#         quality = None
#         if len(parts) > 6 and is_float(parts[6]):
#             quality = float(parts[6])
#
#         return {
#             "tag_id": tag_id,
#             "position": [x, y, z],
#             "quality": quality,
#             "position_type": "tag_position"
#         }
#
#     except Exception:
#         return None
#
#
# # Parses compact listener output.
# # Example:
# # 0) 4CAD[-1.31,0.17,0.39,47,x0C]
# # ---------------------------------------------------------------------------------------------------------------------
# def parse_compact_listener_position(line):
#     match = re.search(
#         r"\)\s*(?P<tag_id>[0-9A-Fa-f]+)\[\s*"
#         r"(?P<x>-?\d+(?:\.\d+)?)\s*,\s*"
#         r"(?P<y>-?\d+(?:\.\d+)?)\s*,\s*"
#         r"(?P<z>-?\d+(?:\.\d+)?)\s*,\s*"
#         r"(?P<quality>-?\d+(?:\.\d+)?)",
#         line
#     )
#
#     if not match:
#         return None
#
#     return {
#         "tag_id": match.group("tag_id"),
#         "position": [
#             float(match.group("x")),
#             float(match.group("y")),
#             float(match.group("z"))
#         ],
#         "quality": float(match.group("quality")),
#         "position_type": "compact_listener_position"
#     }
#
#
# # Parses one UWB line in Tag Position mode.
# # ---------------------------------------------------------------------------------------------------------------------
# def parse_tag_position_line(line):
#     pos_result = parse_pos_csv_position(line)
#     if pos_result is not None:
#         return pos_result
#
#     compact_result = parse_compact_listener_position(line)
#     if compact_result is not None:
#         return compact_result
#
#     return None
#
#
# # Checks whether a serial line is only a shell prompt or command echo.
# # ---------------------------------------------------------------------------------------------------------------------
# def should_ignore_serial_line(line):
#     clean = line.strip().lower()
#
#     if clean in ["", "lec", "les", "lep", "c", "dwm>"]:
#         return True
#
#     if clean in ["dwm> lec", "dwm> les", "dwm> lep"]:
#         return True
#
#     # Do not ignore dirty lines that still contain useful POS data.
#     # Example: lePOS,0,4CAD,...
#     if "pos," in clean:
#         return False
#
#     return False
#
#
# # =====================================================================================================================
# # COORDINATE ALIGNMENT AND TWO-TAG FUSION
# # =====================================================================================================================
# # Applies a simple translation offset to align a listener/network coordinate frame.
# # ---------------------------------------------------------------------------------------------------------------------
# def apply_listener_offset(position, offset):
#     return (to_vector(position) + to_vector(offset)).tolist()
#
#
# # Keeps an angle inside the range -pi to pi.
# # ---------------------------------------------------------------------------------------------------------------------
# def normalize_angle(angle):
#     return math.atan2(math.sin(angle), math.cos(angle))
#
#
# # Calculates the holder/drone yaw from the measured front and back tag positions.
# # The measured world angle is compared to the configured local tag-offset angle.
# # ---------------------------------------------------------------------------------------------------------------------
# def calculate_holder_yaw(front_position, back_position, front_offset_from_center, back_offset_from_center):
#     front_position = to_vector(front_position)
#     back_position = to_vector(back_position)
#
#     front_offset_from_center = to_vector(front_offset_from_center)
#     back_offset_from_center = to_vector(back_offset_from_center)
#
#     world_line = front_position - back_position
#     local_line = front_offset_from_center - back_offset_from_center
#
#     if np.linalg.norm(world_line[0:2]) < 1e-6:
#         return None
#
#     if np.linalg.norm(local_line[0:2]) < 1e-6:
#         return None
#
#     world_line_angle = math.atan2(world_line[1], world_line[0])
#     local_line_angle = math.atan2(local_line[1], local_line[0])
#
#     holder_yaw = normalize_angle(world_line_angle - local_line_angle)
#
#     return holder_yaw
#
#
# # Rotates a tag offset from holder/body coordinates into world coordinates.
# # Only X/Y are rotated. Z is kept the same.
# # ---------------------------------------------------------------------------------------------------------------------
# def rotate_offset_to_world(tag_offset_from_center, holder_yaw):
#     offset = to_vector(tag_offset_from_center)
#
#     c = math.cos(holder_yaw)
#     s = math.sin(holder_yaw)
#
#     rotated_x = c * offset[0] - s * offset[1]
#     rotated_y = s * offset[0] + c * offset[1]
#     rotated_z = offset[2]
#
#     return np.array([rotated_x, rotated_y, rotated_z], dtype=float)
#
#
# # Converts a measured tag position to the estimated centerpoint position.
# # ---------------------------------------------------------------------------------------------------------------------
# def convert_tag_position_to_center_position(tag_position, tag_offset_from_center, holder_yaw=None):
#     tag_position = to_vector(tag_position)
#
#     if USE_ROTATED_TAG_OFFSETS and holder_yaw is not None:
#         tag_offset_world = rotate_offset_to_world(tag_offset_from_center, holder_yaw)
#     else:
#         tag_offset_world = to_vector(tag_offset_from_center)
#
#     center_position = tag_position - tag_offset_world
#
#     return [
#         float(center_position[0]),
#         float(center_position[1]),
#         float(center_position[2])
#     ]
#
#
# # Returns the freshest valid listener results inside the combine window.
# # ---------------------------------------------------------------------------------------------------------------------
# def get_fresh_listener_results(latest_positions, now, combine_window):
#     fresh_results = {}
#
#     for listener_id, result in latest_positions.items():
#         if result is None:
#             continue
#
#         if abs(now - result["timestamp"]) <= combine_window:
#             fresh_results[listener_id] = result
#
#     return fresh_results
#
#
# # Fuses two physical tags into one raw UWB centerpoint.
# # ---------------------------------------------------------------------------------------------------------------------
# def fuse_two_listener_tag_positions(
#         latest_positions,
#         current_timestamp,
#         combine_window,
#         fusion_state,
#         allow_single_listener_fallback=DEFAULT_ALLOW_SINGLE_LISTENER_FALLBACK):
#
#     fresh = get_fresh_listener_results(latest_positions, current_timestamp, combine_window)
#
#     tag1 = fresh.get(1)
#     tag2 = fresh.get(2)
#
#     front_tag = fresh.get(FRONT_TAG_LISTENER_ID)
#     back_tag = fresh.get(BACK_TAG_LISTENER_ID)
#
#     holder_yaw = None
#     yaw_source = "none"
#
#     # -------------------------------------------------------------
#     # Calculate current holder yaw when both front and back tags exist
#     # -------------------------------------------------------------
#     if front_tag is not None and back_tag is not None:
#         front_offset = DEFAULT_TAG_OFFSETS_FROM_CENTER.get(FRONT_TAG_LISTENER_ID, [0.0, 0.0, 0.0])
#         back_offset = DEFAULT_TAG_OFFSETS_FROM_CENTER.get(BACK_TAG_LISTENER_ID, [0.0, 0.0, 0.0])
#
#         holder_yaw = calculate_holder_yaw(
#             front_tag["aligned_position"],
#             back_tag["aligned_position"],
#             front_offset,
#             back_offset
#         )
#
#         if holder_yaw is not None:
#             fusion_state["latest_yaw"] = holder_yaw
#             fusion_state["latest_yaw_time"] = current_timestamp
#             yaw_source = "current_two_tags"
#
#     # -------------------------------------------------------------
#     # If only one tag is available, use latest known yaw if it is recent
#     # -------------------------------------------------------------
#     if holder_yaw is None:
#         latest_yaw = fusion_state.get("latest_yaw")
#         latest_yaw_time = fusion_state.get("latest_yaw_time")
#
#         if latest_yaw is not None and latest_yaw_time is not None:
#             yaw_age = current_timestamp - latest_yaw_time
#
#             if yaw_age <= MAX_YAW_AGE_FOR_SINGLE_TAG_FALLBACK:
#                 holder_yaw = latest_yaw
#                 yaw_source = "latest_known_yaw"
#
#     # -------------------------------------------------------------
#     # Convert available tag positions to centerpoint estimates
#     # -------------------------------------------------------------
#     tag1_center_position = None
#     tag2_center_position = None
#
#     if tag1 is not None:
#         tag1_center_position = convert_tag_position_to_center_position(
#             tag1["aligned_position"],
#             DEFAULT_TAG_OFFSETS_FROM_CENTER.get(1, [0.0, 0.0, 0.0]),
#             holder_yaw
#         )
#
#     if tag2 is not None:
#         tag2_center_position = convert_tag_position_to_center_position(
#             tag2["aligned_position"],
#             DEFAULT_TAG_OFFSETS_FROM_CENTER.get(2, [0.0, 0.0, 0.0]),
#             holder_yaw
#         )
#
#     # -------------------------------------------------------------
#     # Both listener/tag positions are available
#     # -------------------------------------------------------------
#     if tag1_center_position is not None and tag2_center_position is not None:
#         p1 = to_vector(tag1_center_position)
#         p2 = to_vector(tag2_center_position)
#
#         q1 = tag1.get("quality")
#         q2 = tag2.get("quality")
#
#         w1 = quality_to_weight(q1)
#         w2 = quality_to_weight(q2)
#
#         if (w1 + w2) > 0:
#             fused_position = ((w1 * p1) + (w2 * p2)) / (w1 + w2)
#             fusion_mode = "two_tag_rotated_center_quality_weighted"
#         else:
#             fused_position = (p1 + p2) / 2.0
#             fusion_mode = "two_tag_rotated_center_average_no_quality"
#
#         quality_values = [q for q in [q1, q2] if q is not None]
#         fused_quality = float(np.mean(quality_values)) if quality_values else None
#
#         holder_yaw_deg = None
#         if holder_yaw is not None:
#             holder_yaw_deg = math.degrees(holder_yaw)
#
#         return {
#             "timestamp": current_timestamp,
#             "position": [float(fused_position[0]), float(fused_position[1]), float(fused_position[2])],
#             "quality": fused_quality,
#             "listener_id": 0,
#             "network_id": 0,
#             "position_type": "two_listener_fused_center_position",
#             "fusion_mode": fusion_mode,
#             "holder_yaw_rad": holder_yaw,
#             "holder_yaw_deg": holder_yaw_deg,
#             "yaw_source": yaw_source,
#             "tag1_position": tag1_center_position,
#             "tag2_position": tag2_center_position,
#             "tag1_aligned_position": tag1.get("aligned_position"),
#             "tag2_aligned_position": tag2.get("aligned_position"),
#             "tag1_raw_position": tag1.get("raw_position"),
#             "tag2_raw_position": tag2.get("raw_position"),
#             "tag1_quality": q1,
#             "tag2_quality": q2,
#             "tag1_id": tag1.get("tag_id"),
#             "tag2_id": tag2.get("tag_id")
#         }
#
#     # -------------------------------------------------------------
#     # Fallback if only one listener/tag is available
#     # -------------------------------------------------------------
#     if allow_single_listener_fallback:
#         # When rotated tag offsets are enabled, one-tag fallback is only safe if a recent yaw is available.
#         if USE_ROTATED_TAG_OFFSETS and holder_yaw is None:
#             return None
#
#         fallback = tag1 if tag1_center_position is not None else tag2
#         fallback_center_position = tag1_center_position if tag1_center_position is not None else tag2_center_position
#
#         if fallback is not None and fallback_center_position is not None:
#             listener_id = fallback.get("listener_id")
#
#             holder_yaw_deg = None
#             if holder_yaw is not None:
#                 holder_yaw_deg = math.degrees(holder_yaw)
#
#             return {
#                 "timestamp": current_timestamp,
#                 "position": fallback_center_position,
#                 "quality": fallback.get("quality"),
#                 "listener_id": listener_id,
#                 "network_id": fallback.get("network_id"),
#                 "position_type": f"two_listener_center_fallback_l{listener_id}",
#                 "fusion_mode": "single_listener_rotated_center_fallback",
#                 "holder_yaw_rad": holder_yaw,
#                 "holder_yaw_deg": holder_yaw_deg,
#                 "yaw_source": yaw_source,
#                 "tag1_position": tag1_center_position,
#                 "tag2_position": tag2_center_position,
#                 "tag1_aligned_position": tag1.get("aligned_position") if tag1 is not None else None,
#                 "tag2_aligned_position": tag2.get("aligned_position") if tag2 is not None else None,
#                 "tag1_raw_position": tag1.get("raw_position") if tag1 is not None else None,
#                 "tag2_raw_position": tag2.get("raw_position") if tag2 is not None else None,
#                 "tag1_quality": tag1.get("quality") if tag1 is not None else None,
#                 "tag2_quality": tag2.get("quality") if tag2 is not None else None,
#                 "tag1_id": tag1.get("tag_id") if tag1 is not None else None,
#                 "tag2_id": tag2.get("tag_id") if tag2 is not None else None
#             }
#
#     return None
#
#
# # =====================================================================================================================
# # FUTURE DISTANCE / RANGE MODE PLACEHOLDERS
# # =====================================================================================================================
# # Future mode for custom firmware:
# # One listener reads individual anchor distances and triangulates one position.
# # ---------------------------------------------------------------------------------------------------------------------
# def run_single_listener_distance_mode(*args, **kwargs):
#     # [Insert single-listener distance reading here when custom firmware exposes raw ranges.]
#     # Expected future pipeline:
#     # listener -> anchor ranges -> range validation -> 3D triangulation -> final UWB position
#     raise NotImplementedError("Single-listener distance mode is reserved for future custom firmware work.")
#
#
# # Future mode for custom firmware:
# # Two listeners read individual anchor distances and combine/triangulate into one position.
# # ---------------------------------------------------------------------------------------------------------------------
# def run_two_listener_distance_mode(*args, **kwargs):
#     # [Insert two-listener distance reading/fusion here when custom firmware exposes raw ranges.]
#     # Expected future pipeline:
#     # listener 1 ranges + listener 2 ranges -> coordinate alignment -> range fusion/triangulation -> final UWB position
#     raise NotImplementedError("Two-listener distance mode is reserved for future custom firmware work.")
#
#
# # =====================================================================================================================
# # SERIAL / PANS SHELL FUNCTIONS
# # =====================================================================================================================
# # Opens a serial connection to the listener module.
# # ---------------------------------------------------------------------------------------------------------------------
# def open_serial_port(port, baud):
#     ser = serial.Serial(port, baud, timeout=1, dsrdtr=False, rtscts=False)
#     ser.dtr = False
#     ser.rts = False
#     return ser
#
#
# # Wakes the DWM1001 shell and clears old serial data.
# # ---------------------------------------------------------------------------------------------------------------------
# def wake_shell(ser):
#     print(f"[UWB] Waking shell on {ser.port}...")
#
#     try:
#         ser.reset_input_buffer()
#         ser.reset_output_buffer()
#     except Exception:
#         pass
#
#     ser.write(b'\r')
#     time.sleep(1.0)
#     ser.read_all()
#
#     ser.write(b'\r\r')
#     time.sleep(1.0)
#     ser.read_all()
#
#
# # Starts the PANS listener position stream.
# # ---------------------------------------------------------------------------------------------------------------------
# def start_position_stream(ser, command=DEFAULT_POSITION_COMMAND):
#     try:
#         if ser.in_waiting > 15:
#             print(f"[UWB] {ser.port} is already streaming data. Skipping '{command}' command.")
#             return
#     except Exception:
#         pass
#
#     print(f"[UWB] Starting stream on {ser.port} with command: {command}")
#
#     try:
#         ser.write((command + '\r').encode('utf-8'))
#         time.sleep(1.0)
#
#         if ser.in_waiting <= 0:
#             print(f"[UWB] No data after first '{command}' command on {ser.port}. Retrying...")
#             ser.write((command + '\r').encode('utf-8'))
#             time.sleep(1.0)
#
#     except Exception as e:
#         print(f"[UWB] Failed to start stream on {ser.port}: {e}")
#
#
# # Stops the UWB stream before closing the port.
# # ---------------------------------------------------------------------------------------------------------------------
# def stop_stream(ser):
#     try:
#         if ser is not None and ser.is_open:
#             print(f"[UWB] Stopping stream on {ser.port}...")
#
#             # Send multiple enters to break out of a running stream and return to shell.
#             ser.write(b'\r')
#             time.sleep(0.2)
#             ser.write(b'\r')
#             time.sleep(0.2)
#
#             try:
#                 ser.reset_input_buffer()
#                 ser.reset_output_buffer()
#             except Exception:
#                 pass
#
#     except Exception:
#         pass
#
#
# # Requests anchor positions using the 'la' command and writes them to [Log]_anchor_positions.csv.
# # This is useful for documentation and report plotting.
# # ---------------------------------------------------------------------------------------------------------------------
# def update_anchor_list(ser, save_dir):
#     print(f"[UWB] Requesting anchor positions from {ser.port}...")
#
#     try:
#         ser.reset_input_buffer()
#         ser.write(b'la\r')
#         time.sleep(0.5)
#         raw_text = ser.read_all().decode('utf-8', errors='ignore')
#         lines = raw_text.split('\n')
#         anchors_found = []
#
#         coord_pattern = re.compile(r"pos=(-?\d+(?:\.\d+)?):(-?\d+(?:\.\d+)?):(-?\d+(?:\.\d+)?)")
#
#         for line in lines:
#             line = line.strip()
#             if "id=" in line and "pos=" in line:
#                 match = coord_pattern.search(line)
#                 if match:
#                     x, y, z = match.groups()
#                     anchor_position = [float(x), float(y), float(z)]
#                     anchors_found.append(anchor_position)
#                     print(f" -> Found Anchor: X={x}, Y={y}, Z={z}")
#
#         if anchors_found:
#             anchor_file_path = os.path.join(save_dir, "[Log]_anchor_positions.csv")
#
#             existing = []
#             if os.path.exists(anchor_file_path):
#                 try:
#                     with open(anchor_file_path, 'r') as file:
#                         next(file, None)
#                         for row in file:
#                             values = [float(v) for v in row.strip().split(',')]
#                             if len(values) == 3:
#                                 existing.append(values)
#                 except Exception:
#                     existing = []
#
#             with open(anchor_file_path, 'w') as file:
#                 file.write("X,Y,Z\n")
#
#                 unique_anchors = []
#                 for anchor in existing + anchors_found:
#                     if anchor not in unique_anchors:
#                         unique_anchors.append(anchor)
#                         file.write(f"{anchor[0]},{anchor[1]},{anchor[2]}\n")
#
#             print(f"[UWB] Success: {len(anchors_found)} anchors detected on {ser.port}.")
#
#         else:
#             print("[UWB] WARNING: No anchors detected in the 'la' output.")
#
#         return anchors_found
#
#     except Exception as e:
#         print(f"[UWB] WARNING: Could not update anchor list on {ser.port}: {e}")
#         return []
#
#
# # =====================================================================================================================
# # CSV WRITING FUNCTIONS
# # =====================================================================================================================
# # Writes the final UWB position CSV used by the report maker.
# # ---------------------------------------------------------------------------------------------------------------------
# def write_final_position(data_writer, data_file, result):
#     x, y, z = result["position"]
#     data_writer.writerow([result["timestamp"], x, y, z])
#     data_file.flush()
#
#
# # Converts a vector for debug CSV writing.
# # ---------------------------------------------------------------------------------------------------------------------
# def debug_vec(result, key):
#     vec = result.get(key)
#     if vec is None:
#         return ["", "", ""]
#     return [vec[0], vec[1], vec[2]]
#
#
# # Writes extra information for debugging two-listener fusion.
# # The report maker can ignore this file.
# # ---------------------------------------------------------------------------------------------------------------------
# def write_two_listener_debug(debug_writer, debug_file, result):
#     if debug_writer is None:
#         return
#
#     tag1_raw = debug_vec(result, "tag1_raw_position")
#     tag1_aligned = debug_vec(result, "tag1_aligned_position")
#     tag1_center = debug_vec(result, "tag1_position")
#
#     tag2_raw = debug_vec(result, "tag2_raw_position")
#     tag2_aligned = debug_vec(result, "tag2_aligned_position")
#     tag2_center = debug_vec(result, "tag2_position")
#
#     fused = result.get("position") or ["", "", ""]
#
#     debug_writer.writerow([
#         result.get("timestamp"),
#         tag1_raw[0], tag1_raw[1], tag1_raw[2],
#         tag1_aligned[0], tag1_aligned[1], tag1_aligned[2],
#         tag1_center[0], tag1_center[1], tag1_center[2],
#         result.get("tag1_quality"), result.get("tag1_id"),
#         tag2_raw[0], tag2_raw[1], tag2_raw[2],
#         tag2_aligned[0], tag2_aligned[1], tag2_aligned[2],
#         tag2_center[0], tag2_center[1], tag2_center[2],
#         result.get("tag2_quality"), result.get("tag2_id"),
#         fused[0], fused[1], fused[2], result.get("quality"),
#         result.get("holder_yaw_rad"),
#         result.get("holder_yaw_deg"),
#         result.get("yaw_source"),
#         result.get("fusion_mode"),
#         result.get("position_type")
#     ])
#
#     debug_file.flush()
#
#
# # Prints the two-listener fusion steps to the GUI terminal.
# # ---------------------------------------------------------------------------------------------------------------------
# def print_two_listener_fusion_status(result):
#
#     def fmt_vec(vec):
#         if vec is None:
#             return "None"
#
#         try:
#             return f"({float(vec[0]):.3f}, {float(vec[1]):.3f}, {float(vec[2]):.3f})"
#         except Exception:
#             return "Invalid"
#
#     def fmt_quality(q):
#         if q is None:
#             return "N/A"
#
#         try:
#             return f"{float(q):.1f}"
#         except Exception:
#             return "N/A"
#
#     def fmt_percent(value):
#         try:
#             return f"{100.0 * float(value):.1f}%"
#         except Exception:
#             return "N/A"
#
#     tag1_aligned = result.get("tag1_aligned_position")
#     tag2_aligned = result.get("tag2_aligned_position")
#
#     tag1_center = result.get("tag1_position")
#     tag2_center = result.get("tag2_position")
#
#     calculated = result.get("position")
#
#     q1 = result.get("tag1_quality")
#     q2 = result.get("tag2_quality")
#     q_calc = result.get("quality")
#
#     w1 = quality_to_weight(q1)
#     w2 = quality_to_weight(q2)
#
#     if (w1 + w2) > 0:
#         p1 = w1 / (w1 + w2)
#         p2 = w2 / (w1 + w2)
#     else:
#         p1 = 0.5
#         p2 = 0.5
#
#     yaw_deg = result.get("holder_yaw_deg")
#     yaw_text = f"{yaw_deg:.1f} deg" if yaw_deg is not None else "N/A"
#
#     mode = result.get("fusion_mode")
#
#     print(
#         "[UWB_FUSION] "
#         f"L1 raw={fmt_vec(tag1_aligned)} -> center={fmt_vec(tag1_center)} q={fmt_quality(q1)} pull={fmt_percent(p1)} | "
#         f"L2 raw={fmt_vec(tag2_aligned)} -> center={fmt_vec(tag2_center)} q={fmt_quality(q2)} pull={fmt_percent(p2)} | "
#         f"CALC={fmt_vec(calculated)} q={fmt_quality(q_calc)} yaw={yaw_text} mode={mode}"
#     )
#
#
# # =====================================================================================================================
# # MAIN UWB DRIVER LOOP
# # =====================================================================================================================
# def run_uwb(stop_event, config, save_dir, data_queue=None):
#     # 1. Unpack MasterControlStation configuration
#     # -----------------------------------------------------------------------------------------------------------------
#     port1 = config.get('port1')
#     port2 = config.get('port2')
#     baud = config.get('baud', DEFAULT_BAUD)
#
#     read_type = config.get('read_type', 'Tag Position')
#     network_scale = config.get('network_scale', config.get('anchor_count', '1 Network / 1 Listener'))
#     two_listener_mode = "2" in str(network_scale) or "8 Anchors" in str(network_scale)
#
#     send_matlab = config.get('send_matlab', False)
#     matlab_host = config.get('matlab_host', UDP_IP)
#     matlab_port = config.get('matlab_uwb_port', UDP_PORT_UWB)
#
#     session_name = config.get('session_name', datetime.now().strftime("%Y%m%d_%H%M%S"))
#     abs_save_dir = os.path.abspath(save_dir)
#
#     position_command = config.get('position_command', DEFAULT_POSITION_COMMAND)
#     combine_window = float(config.get('combine_window', DEFAULT_COMBINE_WINDOW))
#
#     listener_offsets = config.get('listener_offsets', DEFAULT_LISTENER_OFFSETS)
#     listener_1_offset = config.get('listener_1_offset', listener_offsets.get(1, DEFAULT_LISTENER_OFFSETS[1]))
#     listener_2_offset = config.get('listener_2_offset', listener_offsets.get(2, DEFAULT_LISTENER_OFFSETS[2]))
#
#     min_valid_quality = float(config.get('min_valid_quality', DEFAULT_MIN_VALID_QUALITY))
#     allow_single_listener_fallback = bool(config.get('allow_single_listener_fallback', DEFAULT_ALLOW_SINGLE_LISTENER_FALLBACK))
#
#     print(f"[UWB] Mode: {read_type}")
#     print(f"[UWB] Network Scale: {network_scale}")
#     print(f"[UWB] MATLAB Live UDP: {send_matlab} -> {matlab_host}:{matlab_port}")
#
#     if read_type != 'Tag Position':
#         print("[UWB] Distance/range modes are reserved for future custom firmware work.")
#         print("[UWB] Current supported mode is: Tag Position")
#         return
#
#     # 2. Determine listener ports
#     # -----------------------------------------------------------------------------------------------------------------
#     ports_to_open = []
#
#     if port1:
#         ports_to_open.append({
#             "port": port1,
#             "listener_id": 1,
#             "network_id": 1,
#             "offset": listener_1_offset
#         })
#
#     if two_listener_mode:
#         if port2:
#             ports_to_open.append({
#                 "port": port2,
#                 "listener_id": 2,
#                 "network_id": 2,
#                 "offset": listener_2_offset
#             })
#             print(f"[UWB] Two-listener two-tag fusion enabled. Ports: {port1}, {port2}")
#             print(f"[UWB] Listener 1 offset: {listener_1_offset}")
#             print(f"[UWB] Listener 2 offset: {listener_2_offset}")
#             print(f"[UWB] Combine window: {combine_window} s")
#         else:
#             print("[UWB] WARNING: Two-listener mode selected, but port2 is not configured.")
#             print("[UWB] Falling back to one-listener mode.")
#             two_listener_mode = False
#
#     if not ports_to_open:
#         print("[UWB] No UWB ports configured. UWB reader will not start.")
#         return
#
#     # 3. Create files and runtime objects
#     # -----------------------------------------------------------------------------------------------------------------
#     active_serials = []
#     buffers = []
#     listener_info = []
#     latest_positions = {}
#
#     fusion_state = {
#         "latest_yaw": None,
#         "latest_yaw_time": None
#     }
#
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#
#     data_file = None
#     data_writer = None
#     error_file = None
#     error_writer = None
#     debug_file = None
#     debug_writer = None
#
#     try:
#         # Final report-compatible UWB position CSV.
#         data_path = os.path.join(abs_save_dir, f"[Log]_uwb_listener1_{session_name}.csv")
#         data_file = open(data_path, 'w', newline='')
#         data_writer = csv.writer(data_file)
#         data_writer.writerow(['Time', 'POSX', 'POSY', 'POSZ'])
#
#         # Error/debug CSV.
#         error_path = os.path.join(abs_save_dir, f"[Log]_uwb_errors_{session_name}.csv")
#         error_file = open(error_path, 'w', newline='')
#         error_writer = csv.writer(error_file)
#         error_writer.writerow(['Time', 'Port', 'ListenerID', 'NetworkID', 'Line', 'Error'])
#
#         # Two-listener fusion debug CSV.
#         if two_listener_mode:
#             debug_path = os.path.join(abs_save_dir, f"[Log]_uwb_two_listener_debug_{session_name}.csv")
#             debug_file = open(debug_path, 'w', newline='')
#             debug_writer = csv.writer(debug_file)
#             debug_writer.writerow([
#                 'Time',
#                 'Tag1_Raw_X', 'Tag1_Raw_Y', 'Tag1_Raw_Z',
#                 'Tag1_Aligned_X', 'Tag1_Aligned_Y', 'Tag1_Aligned_Z',
#                 'Tag1_Center_X', 'Tag1_Center_Y', 'Tag1_Center_Z',
#                 'Tag1_Quality', 'Tag1_ID',
#                 'Tag2_Raw_X', 'Tag2_Raw_Y', 'Tag2_Raw_Z',
#                 'Tag2_Aligned_X', 'Tag2_Aligned_Y', 'Tag2_Aligned_Z',
#                 'Tag2_Center_X', 'Tag2_Center_Y', 'Tag2_Center_Z',
#                 'Tag2_Quality', 'Tag2_ID',
#                 'Fused_X', 'Fused_Y', 'Fused_Z', 'Fused_Quality',
#                 'HolderYawRad', 'HolderYawDeg', 'YawSource',
#                 'FusionMode', 'PositionType'
#             ])
#
#         # 4. Setup serial ports
#         # -------------------------------------------------------------------------------------------------------------
#         for info in ports_to_open:
#             port = info["port"]
#             listener_id = info["listener_id"]
#
#             print(f"[UWB] Opening listener {listener_id} on {port}...")
#
#             ser = open_serial_port(port, baud)
#             active_serials.append(ser)
#             buffers.append("")
#             listener_info.append(info)
#
#             wake_shell(ser)
#             update_anchor_list(ser, abs_save_dir)
#
#         for ser, info in zip(active_serials, listener_info):
#             listener_id = info["listener_id"]
#
#             print(f"[UWB] Starting listener {listener_id} stream...")
#             start_position_stream(ser, position_command)
#
#         print("[UWB] Listening for position data...")
#
#         # 5. Main processing loop
#         # -------------------------------------------------------------------------------------------------------------
#         while not stop_event.is_set():
#             for i, ser in enumerate(active_serials):
#                 info = listener_info[i]
#                 listener_id = info["listener_id"]
#                 network_id = info["network_id"]
#                 port = info["port"]
#                 offset = info["offset"]
#
#                 if ser.in_waiting <= 0:
#                     continue
#
#                 new_data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
#                 buffers[i] += new_data
#
#                 if '\n' not in buffers[i]:
#                     continue
#
#                 lines = buffers[i].split('\n')
#                 buffers[i] = lines.pop()
#
#                 for line in lines:
#                     line = line.strip()
#
#                     if should_ignore_serial_line(line):
#                         continue
#
#                     arr_time = time.time()
#                     if not two_listener_mode:
#                         print(f"[UWB_RAW][L{listener_id}] {line}")
#
#                     try:
#                         parsed = parse_tag_position_line(line)
#
#                         if parsed is None:
#                             log_error(
#                                 error_writer, error_file, arr_time,
#                                 port, listener_id, network_id, line,
#                                 "Could not parse tag position line."
#                             )
#                             continue
#
#                         raw_position = parsed.get("position")
#                         quality = parsed.get("quality")
#
#                         if not is_valid_position(raw_position, quality=quality, min_quality=min_valid_quality):
#                             log_error(
#                                 error_writer, error_file, arr_time,
#                                 port, listener_id, network_id, line,
#                                 "Invalid UWB position or quality."
#                             )
#                             continue
#
#                         aligned_position = apply_listener_offset(raw_position, offset)
#
#                         current_result = {
#                             "timestamp": arr_time,
#                             "raw_position": raw_position,
#                             "aligned_position": aligned_position,
#                             "position": aligned_position,
#                             "quality": quality,
#                             "listener_id": listener_id,
#                             "network_id": network_id,
#                             "tag_id": parsed.get("tag_id"),
#                             "position_type": parsed.get("position_type", "tag_position")
#                         }
#
#                         latest_positions[listener_id] = current_result
#
#                         # ------------------------------------------
#                         # Mode A: one listener position reading
#                         # ------------------------------------------
#                         if not two_listener_mode:
#                             final_result = {
#                                 "timestamp": arr_time,
#                                 "position": aligned_position,
#                                 "quality": quality,
#                                 "listener_id": listener_id,
#                                 "network_id": network_id,
#                                 "position_type": "single_listener_position",
#                                 "fusion_mode": "single_listener"
#                             }
#
#                         # ------------------------------------------
#                         # Mode B: two listeners, two physical tags
#                         # ------------------------------------------
#                         else:
#                             final_result = fuse_two_listener_tag_positions(
#                                 latest_positions=latest_positions,
#                                 current_timestamp=arr_time,
#                                 combine_window=combine_window,
#                                 fusion_state=fusion_state,
#                                 allow_single_listener_fallback=allow_single_listener_fallback
#                             )
#
#                         if final_result is None:
#                             continue
#
#                         # ------------------------------------------
#                         # Print useful debug information
#                         # ------------------------------------------
#                         if two_listener_mode:
#                             print_two_listener_fusion_status(final_result)
#                         else:
#                             pos = final_result["position"]
#                             quality = final_result.get("quality")
#                             quality_text = f"{quality:.1f}" if quality is not None else "N/A"
#
#                             print(
#                                 f"[UWB_SINGLE] "
#                                 f"X={pos[0]:.3f}, "
#                                 f"Y={pos[1]:.3f}, "
#                                 f"Z={pos[2]:.3f}, "
#                                 f"Q={quality_text}"
#                             )
#
#                         # ------------------------------------------
#                         # Log and route final UWB position
#                         # ------------------------------------------
#                         write_final_position(data_writer, data_file, final_result)
#
#                         if two_listener_mode:
#                             write_two_listener_debug(debug_writer, debug_file, final_result)
#
#                         send_to_master_queue(data_queue, final_result)
#
#                         if send_matlab:
#                             send_uwb_udp(sock, matlab_host, matlab_port, final_result)
#
#                     except Exception as e:
#                         log_error(error_writer, error_file, arr_time, port, listener_id, network_id, line, str(e))
#
#             time.sleep(0.001)
#
#     except Exception as e:
#         print(f"[UWB Error] {e}")
#
#     finally:
#         print("[UWB] Closing UWB drivers...")
#
#         for ser in active_serials:
#             try:
#                 if ser.is_open:
#                     stop_stream(ser)
#                     ser.close()
#             except Exception:
#                 pass
#
#         if data_file is not None:
#             data_file.close()
#
#         if error_file is not None:
#             error_file.close()
#
#         if debug_file is not None:
#             debug_file.close()
#
#         try:
#             sock.close()
#         except Exception:
#             pass
#
#         print("[UWB] Drivers Closed.")
#
#
#
#
#
#
#
#










# ===================== PROGRAM_INFO ==================================================================================
"""
Author: Renzo Eisma
Date: 06/2026
Description:
    UDP reader/logger for the custom Mini UWB PCB ESP32-C6 streamer.

Purpose:
    - Listen to UDP packets sent by the ESP32-C6 custom PCB.
    - Send a small HELLO packet to the ESP first, so the ESP learns the real laptop IP/port.
    - Save a reduced high-rate CSV containing the requested useful fields:
      UWB position, individual anchor distances, IMU and battery voltage.
    - Save a normal report-compatible UWB position CSV with only:
      Time, POSX, POSY, POSZ
    - Send valid new UWB position updates to MATLAB using the same format as uwb_sensor.py.
    - Send valid new UWB position updates back to MasterControlStation through data_queue.

Expected ESP packet source:
    ESP32-C6 Mini UWB PCB UDP Sensor Streamer with handleUdpIncoming() support.

Expected ESP UDP packet types:
    FAST    = high-rate packet with latest known UWB + latest IMU + battery.
    UWB     = packet sent immediately when a new DWM/SPI UWB result is received.
    ESP_ACK = ESP reply to HELLO_FROM_PYTHON. This script ignores it.

Important UDP method:
    - Python binds to 0.0.0.0:4210.
    - Python sends HELLO_FROM_PYTHON to ESP at 192.168.4.1:4211.
    - ESP stores the sender address and replies/sends data back to the same Python socket.
    - This avoids unreliable broadcast behavior on Windows.

Assistance note:
    ChatGPT was used to create/refactor this version based on the existing custom PCB UDP logger.
"""
# =====================================================================================================================

import argparse
import csv
import math
import os
import socket
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Any

# =====================================================================================================================
# DEFAULT SETTINGS
# =====================================================================================================================

DEFAULT_UDP_BIND_IP = "0.0.0.0"
DEFAULT_UDP_PORT = 4210
DEFAULT_SOCKET_TIMEOUT_S = 0.2

# ESP AP settings.
DEFAULT_ESP_IP = "192.168.4.1"
DEFAULT_ESP_UDP_PORT = 4211
DEFAULT_SEND_HELLO_TO_ESP = True
DEFAULT_HELLO_INTERVAL_S = 1.0

DEFAULT_MATLAB_HOST = "127.0.0.1"
DEFAULT_MATLAB_UWB_PORT = 5005

MAX_ANCHORS = 8

# How often files are flushed to disk. Do not flush every row unless debugging.
DEFAULT_FLUSH_INTERVAL_S = 1.0

# Print live statistics this often.
DEFAULT_STATUS_INTERVAL_S = 1.0

# If true, the position CSV / MATLAB / GUI only get new UWB updates.
# The full CSV still logs all FAST packets.
DEFAULT_ONLY_OUTPUT_NEW_UWB = True

# =====================================================================================================================
# EXPECTED HEADER FROM THE ESP32-C6 UDP STREAMER
# =====================================================================================================================

def build_expected_header(max_anchors: int = MAX_ANCHORS) -> List[str]:
    header = [
        "packet_type",
        "seq",
        "time_ms",
        "time_us",
        "bmi_found",
        "battery_v",
        "battery_status",
        "dwm_status",
        "uwb_update_counter",
        "uwb_new",
        "uwb_valid",
        "uwb_age_ms",
        "uwb_x",
        "uwb_y",
        "uwb_z",
        "uwb_quality",
        "anchor_count",
    ]

    for i in range(max_anchors):
        header.extend([
            f"a{i}_valid",
            f"a{i}_id",
            f"a{i}_x",
            f"a{i}_y",
            f"a{i}_z",
            f"a{i}_distance",
            f"a{i}_dist_q",
            f"a{i}_pos_q",
        ])

    header.extend([
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
    ])

    return header


EXPECTED_HEADER = build_expected_header(MAX_ANCHORS)

# =====================================================================================================================
# REDUCED FULL CSV HEADER
# =====================================================================================================================

# This is the intentionally reduced high-rate CSV.
# It keeps only the fields requested for normal measurements.
# The ESP may send more fields, but they are not written to the full CSV.
MINIMAL_FULL_CSV_HEADER = [
    "pc_timestamp",
    "time_ms",
    "packet_type",
    "uwb_new",
    "uwb_valid",
    "uwb_x",
    "uwb_y",
    "uwb_z",
    "uwb_quality",
    "a0_id", "a0_distance",
    "a1_id", "a1_distance",
    "a2_id", "a2_distance",
    "a3_id", "a3_distance",
    "a4_id", "a4_distance",
    "a5_id", "a5_distance",
    "a6_id", "a6_distance",
    "a7_id", "a7_distance",
    "acc_x",
    "acc_y",
    "acc_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "battery_v",
]


def build_minimal_full_csv_row(data: Dict[str, str], pc_timestamp: float) -> List[Any]:
    row = [
        pc_timestamp,
        data.get("time_ms", ""),
        data.get("packet_type", ""),
        data.get("uwb_new", ""),
        data.get("uwb_valid", ""),
        data.get("uwb_x", ""),
        data.get("uwb_y", ""),
        data.get("uwb_z", ""),
        data.get("uwb_quality", ""),
    ]

    for i in range(MAX_ANCHORS):
        row.append(data.get(f"a{i}_id", ""))
        row.append(data.get(f"a{i}_distance", ""))

    row.extend([
        data.get("acc_x", ""),
        data.get("acc_y", ""),
        data.get("acc_z", ""),
        data.get("gyro_x", ""),
        data.get("gyro_y", ""),
        data.get("gyro_z", ""),
        data.get("battery_v", ""),
    ])

    return row

# =====================================================================================================================
# BASIC HELPERS
# =====================================================================================================================

def ensure_dir(path: str) -> str:
    abs_path = os.path.abspath(path)
    os.makedirs(abs_path, exist_ok=True)
    return abs_path


def safe_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        if value is None:
            return default
        text = str(value).strip()
        if text == "" or text.lower() == "nan":
            return default
        result = float(text)
        if math.isnan(result) or math.isinf(result):
            return default
        return result
    except (TypeError, ValueError):
        return default


def safe_int(value: Any, default: Optional[int] = None) -> Optional[int]:
    try:
        if value is None:
            return default
        text = str(value).strip()
        if text == "":
            return default
        return int(float(text))
    except (TypeError, ValueError):
        return default


def safe_bool_int(value: Any) -> bool:
    parsed = safe_int(value, 0)
    return parsed is not None and parsed != 0


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def normalize_row_length(row: List[str], header: List[str]) -> List[str]:
    if len(row) < len(header):
        return row + [""] * (len(header) - len(row))
    if len(row) > len(header):
        return row[:len(header)]
    return row


def send_hello_to_esp(sock: socket.socket, esp_ip: str, esp_port: int) -> None:
    """
    Send a small packet to the ESP so the ESP can learn the laptop IP and source port.
    The ESP code should handle this with handleUdpIncoming().
    """
    msg = f"HELLO_FROM_PYTHON,{time.time():.6f}".encode("utf-8")
    sock.sendto(msg, (esp_ip, esp_port))

# =====================================================================================================================
# MATLAB / GUI OUTPUT FUNCTIONS
# =====================================================================================================================

def send_uwb_udp(sock: socket.socket, matlab_host: str, matlab_port: int, result: Dict[str, Any]) -> None:
    """
    Sends the final UWB position packet to MATLAB.
    Same packet format as the original serial listener script:
    timestamp,x,y,z,quality,listener_id,network_id,position_type
    """
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


def send_to_master_queue(data_queue: Any, result: Dict[str, Any]) -> None:
    """
    Sends the final UWB position to MasterControlStation for the live plot.
    """
    if data_queue is None:
        return

    x, y, z = result["position"]

    data_queue.put({
        "source": "uwb",
        "source_type": "custom_pcb_udp",
        "data_type": "position",
        "timestamp": result["timestamp"],
        "pc_timestamp": result["timestamp"],
        "position": {
            "x": x,
            "y": y,
            "z": z,
        },
        "quality": {
            "valid": True,
            "accuracy": result.get("quality"),
        },
        "metadata": {
            "listener_id": result.get("listener_id"),
            "network_id": result.get("network_id"),
            "position_type": result.get("position_type"),
            "packet_type": result.get("packet_type"),
            "seq": result.get("seq"),
            "esp_time_ms": result.get("esp_time_ms"),
            "esp_time_us": result.get("esp_time_us"),
            "uwb_update_counter": result.get("uwb_update_counter"),
            "uwb_age_ms": result.get("uwb_age_ms"),
            "battery_v": result.get("battery_v"),
            "battery_status": result.get("battery_status"),
            "bmi_found": result.get("bmi_found"),
            "dwm_status": result.get("dwm_status"),
            "accel": result.get("accel"),
            "gyro": result.get("gyro"),
            "anchor_count": result.get("anchor_count"),
            "anchors": result.get("anchors"),
        }
    })

# =====================================================================================================================
# PACKET PARSING
# =====================================================================================================================

def parse_udp_csv_line(line: str, current_header: Optional[List[str]]) -> Tuple[str, Optional[List[str]], Optional[Dict[str, str]], Optional[str]]:
    """
    Returns:
        kind: "header", "data", "ack", "ignore" or "error"
        new_header: header list if kind == header else current_header
        data: parsed dict if kind == data
        error: error string if kind == error
    """
    clean = line.strip()

    if clean == "":
        return "ignore", current_header, None, None

    try:
        row = next(csv.reader([clean]))
    except Exception as exc:
        return "error", current_header, None, f"CSV parse error: {exc}"

    if not row:
        return "ignore", current_header, None, None

    # The ESP replies to the Python HELLO packet with ESP_ACK.
    # This is useful for debugging but should not be written as a sensor packet.
    if row[0] in ("ESP_ACK", "ACK", "HELLO_ACK"):
        return "ack", current_header, None, None

    # The ESP repeats the header regularly.
    if row[0] == "packet_type":
        return "header", row, None, None

    if row[0] not in ("FAST", "UWB"):
        return "error", current_header, None, f"Unknown packet type: {row[0]}"

    header = current_header or EXPECTED_HEADER
    row = normalize_row_length(row, header)
    data = {key: row[i] for i, key in enumerate(header)}

    return "data", header, data, None


def extract_anchors(data: Dict[str, str], max_anchors: int = MAX_ANCHORS) -> List[Dict[str, Any]]:
    anchors = []

    for i in range(max_anchors):
        valid = safe_bool_int(data.get(f"a{i}_valid"))

        anchor = {
            "index": i,
            "valid": valid,
            "id": data.get(f"a{i}_id", ""),
            "x": safe_float(data.get(f"a{i}_x")),
            "y": safe_float(data.get(f"a{i}_y")),
            "z": safe_float(data.get(f"a{i}_z")),
            "distance": safe_float(data.get(f"a{i}_distance")),
            "distance_quality": safe_int(data.get(f"a{i}_dist_q")),
            "position_quality": safe_int(data.get(f"a{i}_pos_q")),
        }

        if valid:
            anchors.append(anchor)

    return anchors


def extract_position_result(data: Dict[str, str], pc_timestamp: float, source_address: Tuple[str, int]) -> Optional[Dict[str, Any]]:
    uwb_valid = safe_bool_int(data.get("uwb_valid"))
    x = safe_float(data.get("uwb_x"))
    y = safe_float(data.get("uwb_y"))
    z = safe_float(data.get("uwb_z"))

    if not uwb_valid or x is None or y is None or z is None:
        return None

    quality = safe_float(data.get("uwb_quality"))
    anchors = extract_anchors(data, MAX_ANCHORS)

    return {
        "timestamp": pc_timestamp,
        "position": [x, y, z],
        "quality": quality,
        "listener_id": 1,
        "network_id": 1,
        "position_type": "custom_pcb_udp_position",
        "packet_type": data.get("packet_type"),
        "seq": safe_int(data.get("seq")),
        "esp_time_ms": safe_int(data.get("time_ms")),
        "esp_time_us": safe_int(data.get("time_us")),
        "uwb_update_counter": safe_int(data.get("uwb_update_counter")),
        "uwb_age_ms": safe_int(data.get("uwb_age_ms")),
        "battery_v": safe_float(data.get("battery_v")),
        "battery_status": data.get("battery_status"),
        "bmi_found": safe_bool_int(data.get("bmi_found")),
        "dwm_status": data.get("dwm_status"),
        "anchor_count": safe_int(data.get("anchor_count"), 0),
        "anchors": anchors,
        "accel": {
            "x": safe_float(data.get("acc_x")),
            "y": safe_float(data.get("acc_y")),
            "z": safe_float(data.get("acc_z")),
        },
        "gyro": {
            "x": safe_float(data.get("gyro_x")),
            "y": safe_float(data.get("gyro_y")),
            "z": safe_float(data.get("gyro_z")),
        },
        "source_ip": source_address[0],
        "source_port": source_address[1],
    }


def is_new_uwb_packet(data: Dict[str, str], last_output_counter: Optional[int]) -> Tuple[bool, Optional[int]]:
    """
    Checks whether this packet should be treated as a new UWB position measurement.
    The ESP sends FAST at high rate and UWB immediately on new DWM data. To prevent repeated
    UWB positions from being sent to MATLAB / GUI many times per second, this function
    primarily uses uwb_update_counter.
    """
    counter = safe_int(data.get("uwb_update_counter"))
    packet_type = data.get("packet_type", "")
    uwb_new_flag = safe_bool_int(data.get("uwb_new"))

    if counter is not None:
        if last_output_counter is None:
            return True, counter
        return counter != last_output_counter, counter

    # Fallback if counter is missing for some reason.
    return packet_type == "UWB" or uwb_new_flag, counter

# =====================================================================================================================
# CSV WRITING
# =====================================================================================================================

def write_position_row(writer: csv.writer, result: Dict[str, Any]) -> None:
    x, y, z = result["position"]
    writer.writerow([result["timestamp"], x, y, z])


def open_output_files(save_dir: str, session_name: str) -> Tuple[Any, csv.writer, Any, csv.writer, Any, csv.writer, str, str, str]:
    full_path = os.path.join(save_dir, f"[Log]_custom_pcb_udp_full_{session_name}.csv")
    position_path = os.path.join(save_dir, f"[Log]_uwb_listener1_{session_name}.csv")
    error_path = os.path.join(save_dir, f"[Log]_custom_pcb_udp_errors_{session_name}.csv")

    full_file = open(full_path, "w", newline="", buffering=1024 * 1024)
    position_file = open(position_path, "w", newline="", buffering=1024 * 256)
    error_file = open(error_path, "w", newline="", buffering=1024 * 64)

    full_writer = csv.writer(full_file)
    position_writer = csv.writer(position_file)
    error_writer = csv.writer(error_file)

    full_writer.writerow(MINIMAL_FULL_CSV_HEADER)
    position_writer.writerow(["Time", "POSX", "POSY", "POSZ"])
    error_writer.writerow(["pc_time_iso", "pc_timestamp", "source_ip", "source_port", "raw_line", "error"])

    return full_file, full_writer, position_file, position_writer, error_file, error_writer, full_path, position_path, error_path

# =====================================================================================================================
# MAIN UDP DRIVER LOOP
# =====================================================================================================================

def run_custom_pcb_udp_logger(stop_event: threading.Event, config: Dict[str, Any], save_dir: str, data_queue: Any = None) -> None:
    udp_bind_ip = config.get("udp_bind_ip", DEFAULT_UDP_BIND_IP)
    udp_port = int(config.get("udp_port", DEFAULT_UDP_PORT))
    socket_timeout_s = float(config.get("socket_timeout_s", DEFAULT_SOCKET_TIMEOUT_S))

    esp_ip = config.get("esp_ip", DEFAULT_ESP_IP)
    esp_port = int(config.get("esp_port", DEFAULT_ESP_UDP_PORT))
    send_hello = bool(config.get("send_hello_to_esp", DEFAULT_SEND_HELLO_TO_ESP))
    hello_interval_s = float(config.get("hello_interval_s", DEFAULT_HELLO_INTERVAL_S))

    send_matlab = bool(config.get("send_matlab", False))
    matlab_host = config.get("matlab_host", DEFAULT_MATLAB_HOST)
    matlab_port = int(config.get("matlab_uwb_port", DEFAULT_MATLAB_UWB_PORT))

    session_name = config.get("session_name", datetime.now().strftime("Session_%Y%m%d_%H%M%S"))
    abs_save_dir = ensure_dir(save_dir)

    flush_interval_s = float(config.get("flush_interval_s", DEFAULT_FLUSH_INTERVAL_S))
    status_interval_s = float(config.get("status_interval_s", DEFAULT_STATUS_INTERVAL_S))
    only_output_new_uwb = bool(config.get("only_output_new_uwb", DEFAULT_ONLY_OUTPUT_NEW_UWB))

    print("[CUSTOM PCB UDP] Starting UDP reader for mini UWB PCB.")
    print(f"[CUSTOM PCB UDP] Listen: {udp_bind_ip}:{udp_port}")
    print(f"[CUSTOM PCB UDP] Save dir: {abs_save_dir}")
    print(f"[CUSTOM PCB UDP] Session: {session_name}")
    print(f"[CUSTOM PCB UDP] ESP hello: {send_hello} -> {esp_ip}:{esp_port}")
    print(f"[CUSTOM PCB UDP] MATLAB live UDP: {send_matlab} -> {matlab_host}:{matlab_port}")
    print("[CUSTOM PCB UDP] Connect the laptop to Wi-Fi: MiniUWB_ESP32C6 / 12345678")

    matlab_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        # Large receive buffer helps if Python is briefly busy writing files.
        recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    except OSError:
        pass

    recv_sock.bind((udp_bind_ip, udp_port))
    recv_sock.settimeout(socket_timeout_s)

    full_file = position_file = error_file = None
    current_header: Optional[List[str]] = EXPECTED_HEADER
    last_output_counter: Optional[int] = None

    packet_count = 0
    fast_count = 0
    uwb_count = 0
    ack_count = 0
    output_position_count = 0
    error_count = 0

    last_flush_time = time.time()
    last_status_time = time.time()
    last_hello_time = 0.0
    status_packet_count = 0
    status_position_count = 0

    def maybe_send_hello(force: bool = False) -> None:
        nonlocal last_hello_time
        if not send_hello:
            return
        now = time.time()
        if force or (now - last_hello_time >= hello_interval_s):
            try:
                send_hello_to_esp(recv_sock, esp_ip, esp_port)
                last_hello_time = now
            except OSError as exc:
                print(f"[CUSTOM PCB UDP] Could not send HELLO to ESP: {exc}")

    try:
        (
            full_file,
            full_writer,
            position_file,
            position_writer,
            error_file,
            error_writer,
            full_path,
            position_path,
            error_path,
        ) = open_output_files(abs_save_dir, session_name)

        print(f"[CUSTOM PCB UDP] Full CSV:     {full_path}")
        print(f"[CUSTOM PCB UDP] Position CSV: {position_path}")
        print(f"[CUSTOM PCB UDP] Error CSV:    {error_path}")
        print("[CUSTOM PCB UDP] Sending HELLO and listening for UDP packets...")

        # First HELLO immediately. The ESP uses this to learn the laptop source IP and port.
        maybe_send_hello(force=True)

        while not stop_event.is_set():
            maybe_send_hello(force=False)

            try:
                payload, address = recv_sock.recvfrom(8192)
            except socket.timeout:
                now = time.time()
                if now - last_status_time >= status_interval_s:
                    print("[CUSTOM PCB UDP] Waiting for packets... sending HELLO to ESP.")
                    last_status_time = now
                continue
            except OSError as exc:
                if stop_event.is_set():
                    break
                print(f"[CUSTOM PCB UDP] Socket error: {exc}")
                break

            pc_timestamp = time.time()
            pc_time_iso = now_iso()

            text = payload.decode("utf-8", errors="replace")
            lines = text.splitlines()

            for line in lines:
                kind, current_header, data, error = parse_udp_csv_line(line, current_header)

                if kind == "ignore":
                    continue

                if kind == "ack":
                    ack_count += 1
                    continue

                if kind == "header":
                    # Header is repeated by the ESP. Do not log it as data.
                    continue

                if kind == "error" or data is None:
                    error_count += 1
                    error_writer.writerow([pc_time_iso, pc_timestamp, address[0], address[1], line.strip(), error or "unknown parse error"])
                    continue

                packet_count += 1
                status_packet_count += 1

                if data.get("packet_type") == "FAST":
                    fast_count += 1
                elif data.get("packet_type") == "UWB":
                    uwb_count += 1

                # Full high-rate log: every received FAST/UWB packet, reduced to the requested columns only.
                full_writer.writerow(build_minimal_full_csv_row(data, pc_timestamp))

                result = extract_position_result(data, pc_timestamp, address)

                if result is not None:
                    should_output_position = True
                    new_uwb, packet_counter = is_new_uwb_packet(data, last_output_counter)

                    if only_output_new_uwb:
                        should_output_position = new_uwb

                    if should_output_position:
                        if packet_counter is not None:
                            last_output_counter = packet_counter

                        write_position_row(position_writer, result)
                        output_position_count += 1
                        status_position_count += 1

                        if send_matlab:
                            send_uwb_udp(matlab_sock, matlab_host, matlab_port, result)

                        send_to_master_queue(data_queue, result)

                now = time.time()

                if now - last_flush_time >= flush_interval_s:
                    full_file.flush()
                    position_file.flush()
                    error_file.flush()
                    last_flush_time = now

                if now - last_status_time >= status_interval_s:
                    dt = now - last_status_time
                    packet_rate = status_packet_count / dt if dt > 0 else 0.0
                    position_rate = status_position_count / dt if dt > 0 else 0.0

                    print(
                        "[CUSTOM PCB UDP] "
                        f"packets={packet_count} "
                        f"fast={fast_count} "
                        f"uwb={uwb_count} "
                        f"ack={ack_count} "
                        f"positions_out={output_position_count} "
                        f"errors={error_count} "
                        f"rate={packet_rate:.1f} pkt/s "
                        f"pos_rate={position_rate:.1f} pos/s "
                        f"from={address[0]}:{address[1]}"
                    )

                    status_packet_count = 0
                    status_position_count = 0
                    last_status_time = now

    except KeyboardInterrupt:
        print("\n[CUSTOM PCB UDP] Keyboard interrupt. Stopping...")

    finally:
        print("[CUSTOM PCB UDP] Closing files and socket...")

        try:
            recv_sock.close()
        except Exception:
            pass

        try:
            matlab_sock.close()
        except Exception:
            pass

        for file_obj in (full_file, position_file, error_file):
            try:
                if file_obj is not None:
                    file_obj.flush()
                    file_obj.close()
            except Exception:
                pass

        print(
            "[CUSTOM PCB UDP] Finished. "
            f"packets={packet_count}, fast={fast_count}, uwb={uwb_count}, ack={ack_count}, "
            f"positions_out={output_position_count}, errors={error_count}"
        )


# Alias with a short name for future MasterControlStation integration.
def run_custom_pcb_udp(stop_event: threading.Event, config: Dict[str, Any], save_dir: str, data_queue: Any = None) -> None:
    run_custom_pcb_udp_logger(stop_event, config, save_dir, data_queue)


# Optional alias if this file is used as a drop-in UWB reader in the master script.
def run_uwb(stop_event: threading.Event, config: Dict[str, Any], save_dir: str, data_queue: Any = None) -> None:
    run_custom_pcb_udp_logger(stop_event, config, save_dir, data_queue)

# =====================================================================================================================
# STANDALONE TEST MODE
# =====================================================================================================================

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read Mini UWB PCB UDP stream and log CSV files.")

    parser.add_argument("--listen-ip", default=DEFAULT_UDP_BIND_IP, help="IP to bind to. Usually 0.0.0.0.")
    parser.add_argument("--port", type=int, default=DEFAULT_UDP_PORT, help="UDP port to listen on. Default: 4210.")
    parser.add_argument("--save-dir", default="./measurements/custom_pcb_udp_test", help="Folder for CSV logs.")
    parser.add_argument("--session-name", default=None, help="Session name used in CSV filenames.")

    parser.add_argument("--esp-ip", default=DEFAULT_ESP_IP, help="ESP AP IP used for HELLO. Default: 192.168.4.1.")
    parser.add_argument("--esp-port", type=int, default=DEFAULT_ESP_UDP_PORT, help="ESP local UDP port for HELLO. Default: 4211.")
    parser.add_argument("--no-hello", action="store_true", help="Disable HELLO packets to the ESP.")
    parser.add_argument("--hello-interval", type=float, default=DEFAULT_HELLO_INTERVAL_S, help="HELLO interval in seconds. Default: 1.0.")

    parser.add_argument("--send-matlab", action="store_true", help="Forward new UWB positions to MATLAB over UDP.")
    parser.add_argument("--matlab-host", default=DEFAULT_MATLAB_HOST, help="MATLAB UDP host. Default: 127.0.0.1.")
    parser.add_argument("--matlab-port", type=int, default=DEFAULT_MATLAB_UWB_PORT, help="MATLAB UWB UDP port. Default: 5005.")

    parser.add_argument("--output-all-valid-positions", action="store_true", help="Write/send every valid FAST/UWB position, not only new UWB updates.")
    parser.add_argument("--flush-interval", type=float, default=DEFAULT_FLUSH_INTERVAL_S, help="CSV flush interval in seconds.")
    parser.add_argument("--status-interval", type=float, default=DEFAULT_STATUS_INTERVAL_S, help="Terminal status interval in seconds.")

    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    session_name = args.session_name
    if not session_name:
        session_name = datetime.now().strftime("Session_%Y%m%d_%H%M%S")

    config = {
        "udp_bind_ip": args.listen_ip,
        "udp_port": args.port,
        "send_hello_to_esp": not args.no_hello,
        "esp_ip": args.esp_ip,
        "esp_port": args.esp_port,
        "hello_interval_s": args.hello_interval,
        "send_matlab": args.send_matlab,
        "matlab_host": args.matlab_host,
        "matlab_uwb_port": args.matlab_port,
        "session_name": session_name,
        "only_output_new_uwb": not args.output_all_valid_positions,
        "flush_interval_s": args.flush_interval,
        "status_interval_s": args.status_interval,
    }

    stop_event = threading.Event()

    try:
        run_custom_pcb_udp_logger(stop_event, config, args.save_dir)
    except KeyboardInterrupt:
        stop_event.set()


if __name__ == "__main__":
    main()
