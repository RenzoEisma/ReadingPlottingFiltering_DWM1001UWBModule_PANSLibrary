# ===================== PROGRAM_INFO ==================================================================================
"""
Author: Renzo Eisma
Date: 06/2026
Description: Read GPS RTK XYZ position data from a ROS 1 laptop through rosbridge_server. This script runs on the
Windows laptop, subscribes to one ROS topic, writes a CSV file, sends live position packets to MasterControlStation for
plotting, and optionally sends live ground-truth UDP packets to MatlabMasterControl.

Assistance note: ChatGPT Pro 5.5 Thinking Extended was used to clean up variable names, comments, line spacing, and
added in depth logging in the terminal. The rewritten version was checked by a human. The original concept, code logic,
and project structure were created by Renzo Eisma.
"""
# =====================================================================================================================


# Required Python package on Windows:
#   pip install roslibpy
#
# Required on Ubuntu ROS laptop:
#   sudo apt install ros-<distro>-rosbridge-server
#   roslaunch rosbridge_server rosbridge_websocket.launch
#
# Main function for MasterControlStation:
#   run_gps_rtk_logger(stop_event, config, save_dir, data_queue=None)
#
# Standalone test example:
#   python GpsRtkRosReader.py --rosbridge ws://192.168.1.50:9090 --topic /gps_rtk/pose --save-dir ./test


from __future__ import annotations

import argparse
import csv
import os
import socket
import threading
import time
from datetime import datetime
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.parse import urlparse


DEFAULT_ROSBRIDGE_URL = "ws://127.0.0.1:9090"
DEFAULT_MESSAGE_TYPE = "geometry_msgs/PoseStamped"

DEFAULT_UDP_IP = "127.0.0.1"
DEFAULT_UDP_PORT_GROUND_TRUTH = 5006

SOURCE_TYPE = "gps_rtk"
DEFAULT_RIGID_BODY_ID = 0


# =============================================================================
# Small configuration helpers
# =============================================================================

def get_config_value(config: Optional[Dict[str, Any]], keys: Iterable[str], default: Any = None) -> Any:
    """Read a value from a config dictionary using multiple possible key names."""
    if config is None:
        return default

    for key in keys:
        if key in config and config[key] not in (None, ""):
            return config[key]

    return default


def safe_bool(value: Any, default: bool = False) -> bool:
    """Convert common GUI/config values to bool."""
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y", "on", "enabled")
    return bool(value)


def parse_rosbridge_url(url_or_host: str) -> Tuple[str, int, bool]:
    """
    Parse a rosbridge URL.

    Accepted examples:
        ws://192.168.1.50:9090
        192.168.1.50:9090
        192.168.1.50
        wss://robot.local:9090

    Returns:
        host, port, is_secure
    """
    if not url_or_host:
        url_or_host = DEFAULT_ROSBRIDGE_URL

    text = str(url_or_host).strip()

    # roslibpy wants host, port and is_secure, not the full ws:// URL.
    if "://" not in text:
        text = "ws://" + text

    parsed = urlparse(text)
    host = parsed.hostname or "127.0.0.1"
    port = parsed.port or 9090
    is_secure = parsed.scheme.lower() == "wss"

    return host, port, is_secure


def make_session_name(config: Optional[Dict[str, Any]]) -> str:
    """Use the MasterControl session name when available, otherwise create one."""
    session_name = get_config_value(config, ["session_name", "session"], None)
    if session_name:
        return str(session_name)
    return datetime.now().strftime("Session_%Y%m%d_%H%M%S")


def ensure_dir(path: str) -> None:
    """Create a folder if it does not exist yet."""
    if path and not os.path.exists(path):
        os.makedirs(path, exist_ok=True)


# =============================================================================
# CSV and error logging helpers
# =============================================================================

def log_gps_error(error_writer: Optional[csv.writer], error_file: Any, error_type: str, details: str) -> None:
    """Write an error/debug line and print it to the console."""
    timestamp = time.time()

    try:
        if error_writer is not None:
            error_writer.writerow([timestamp, error_type, details])
        if error_file is not None:
            error_file.flush()
    except Exception:
        pass

    print(f"[GPS RTK ERROR] {error_type}: {details}")


# =============================================================================
# ROS message parsing
# =============================================================================

def _to_float(value: Any) -> Optional[float]:
    """Convert a value to float safely."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _point_from_dict(point: Any) -> Optional[Tuple[float, float, float]]:
    """Try to read x/y/z from a dict-like object."""
    if not isinstance(point, dict):
        return None

    if not all(axis in point for axis in ("x", "y", "z")):
        return None

    x = _to_float(point.get("x"))
    y = _to_float(point.get("y"))
    z = _to_float(point.get("z"))

    if x is None or y is None or z is None:
        return None

    return x, y, z


def extract_xyz_from_message(msg: Dict[str, Any]) -> Tuple[float, float, float, str]:
    """
    Extract XYZ from common ROS position message layouts.

    Supported layouts include:
        geometry_msgs/PoseStamped:
            msg["pose"]["position"]["x/y/z"]

        geometry_msgs/Pose:
            msg["position"]["x/y/z"]

        geometry_msgs/PointStamped:
            msg["point"]["x/y/z"]

        geometry_msgs/Point:
            msg["x/y/z"]

        nav_msgs/Odometry:
            msg["pose"]["pose"]["position"]["x/y/z"]

        geometry_msgs/PoseWithCovarianceStamped:
            msg["pose"]["pose"]["position"]["x/y/z"]

        geometry_msgs/TransformStamped:
            msg["transform"]["translation"]["x/y/z"]

        Custom simple messages:
            msg["data"] = [x, y, z]
            msg["position"] = [x, y, z]
            msg["pos"] = [x, y, z]

    Returns:
        x, y, z, extraction_method

    Raises:
        ValueError if no XYZ position can be found.
    """
    if not isinstance(msg, dict):
        raise ValueError(f"ROS message is not a dictionary. Type={type(msg)}")

    candidates: List[Tuple[str, Any]] = [
        ("nav_msgs/Odometry or PoseWithCovarianceStamped: pose.pose.position",
         msg.get("pose", {}).get("pose", {}).get("position", {}) if isinstance(msg.get("pose"), dict) else {}),
        ("geometry_msgs/PoseStamped: pose.position",
         msg.get("pose", {}).get("position", {}) if isinstance(msg.get("pose"), dict) else {}),
        ("geometry_msgs/PointStamped: point",
         msg.get("point", {})),
        ("geometry_msgs/Pose: position",
         msg.get("position", {})),
        ("geometry_msgs/TransformStamped: transform.translation",
         msg.get("transform", {}).get("translation", {}) if isinstance(msg.get("transform"), dict) else {}),
        ("geometry_msgs/Point or custom top-level x/y/z",
         msg),
    ]

    for method, candidate in candidates:
        point = _point_from_dict(candidate)
        if point is not None:
            return point[0], point[1], point[2], method

    # Some custom ROS messages may use arrays instead of named x/y/z fields.
    array_candidates: List[Tuple[str, Any]] = [
        ("custom data array", msg.get("data")),
        ("custom position array", msg.get("position")),
        ("custom pos array", msg.get("pos")),
        ("custom xyz array", msg.get("xyz")),
    ]

    for method, candidate in array_candidates:
        if isinstance(candidate, (list, tuple)) and len(candidate) >= 3:
            x = _to_float(candidate[0])
            y = _to_float(candidate[1])
            z = _to_float(candidate[2])
            if x is not None and y is not None and z is not None:
                return x, y, z, method

    raise ValueError(f"Could not find XYZ position in ROS message. Top-level keys: {list(msg.keys())}")


# =============================================================================
# Packet builders
# =============================================================================

def build_mastercontrol_packet(timestamp: float,
                               pos: Tuple[float, float, float],
                               frame_id: str = "map",
                               message_type: str = "",
                               topic: str = "",
                               extraction_method: str = "") -> Dict[str, Any]:
    """
    Build the dictionary packet sent to MasterControlStation for live plotting.

    This follows the same general structure as the OptiTrack packet:
        source = ground_truth
        source_type = gps_rtk
        data_type = position
        timestamp
        position = {x, y, z}
        orientation
        quality
        metadata
    """
    x, y, z = pos

    return {
        "source": "ground_truth",
        "source_type": SOURCE_TYPE,
        "data_type": "position",
        "timestamp": timestamp,

        # Main nested position format.
        "position": {
            "x": float(x),
            "y": float(y),
            "z": float(z),
        },

        # Extra flat keys for simple plotting code that may expect x/y/z directly.
        "x": float(x),
        "y": float(y),
        "z": float(z),

        # GPS RTK does not necessarily provide orientation.
        # Identity quaternion keeps the structure compatible with OptiTrack-like packets.
        "orientation": {
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        },

        # No filtering here: log everything.
        "quality": {
            "valid": True,
            "source": SOURCE_TYPE,
        },

        "metadata": {
            "frame_id": frame_id,
            "topic": topic,
            "message_type": message_type,
            "extraction_method": extraction_method,
        }
    }


def build_matlab_packet(timestamp: float,
                        pos: Tuple[float, float, float],
                        quality: int = 1,
                        rigid_body_id: int = DEFAULT_RIGID_BODY_ID) -> str:
    """
    Build the live UDP packet for MatlabMasterControl.

    Format:
        timestamp,x,y,z,quality,rigid_body_id,source_type

    This is intentionally the same ground-truth format used by the OptiTrack reader,
    with source_type changed to gps_rtk.
    """
    x, y, z = pos

    return (
        f"{timestamp:.6f},"
        f"{float(x):.6f},"
        f"{float(y):.6f},"
        f"{float(z):.6f},"
        f"{int(quality)},"
        f"{int(rigid_body_id)},"
        f"{SOURCE_TYPE}"
    )


# =============================================================================
# Optional rosapi helpers
# =============================================================================

def _call_ros_service_sync(ros: Any,
                           service_name: str,
                           service_type: str,
                           request_dict: Dict[str, Any],
                           timeout: float = 3.0) -> Optional[Dict[str, Any]]:
    """
    Call a rosbridge/rosapi service and wait for the response.

    This function is intentionally defensive because roslibpy versions can differ
    slightly in how Service.call behaves.
    """
    import roslibpy  # Imported here so the module can still be imported without roslibpy.

    result_box: Dict[str, Any] = {"response": None, "error": None}
    done = threading.Event()

    def callback(response: Dict[str, Any]) -> None:
        result_box["response"] = response
        done.set()

    def errback(error: Any) -> None:
        result_box["error"] = error
        done.set()

    try:
        service = roslibpy.Service(ros, service_name, service_type)
        request = roslibpy.ServiceRequest(request_dict)

        returned = service.call(request, callback=callback, errback=errback)

        # Some roslibpy versions return a response directly if no async handling is used.
        if isinstance(returned, dict):
            return returned

        done.wait(timeout=timeout)

        if result_box["response"] is not None:
            return result_box["response"]

        return None

    except Exception:
        return None


def resolve_topic_type(ros: Any,
                       topic_name: str,
                       configured_type: Optional[str] = None,
                       timeout: float = 3.0) -> str:
    """
    Get the ROS topic type.

    Priority:
        1. Explicit message type from config/CLI.
        2. rosapi /rosapi/topic_type.
        3. DEFAULT_MESSAGE_TYPE.
    """
    if configured_type:
        return configured_type

    response = _call_ros_service_sync(
        ros,
        service_name="/rosapi/topic_type",
        service_type="rosapi/TopicType",
        request_dict={"topic": topic_name},
        timeout=timeout,
    )

    if isinstance(response, dict):
        msg_type = response.get("type")
        if msg_type:
            print(f"[GPS RTK] Topic type detected through rosapi: {msg_type}")
            return str(msg_type)

    print(f"[GPS RTK] Could not auto-detect topic type. Using default: {DEFAULT_MESSAGE_TYPE}")
    return DEFAULT_MESSAGE_TYPE


def list_ros_topics(rosbridge_url: str) -> None:
    """
    Standalone helper to list ROS topics through rosbridge.
    This is included in the same script so no separate topic-probe script is needed.
    """
    import roslibpy

    host, port, is_secure = parse_rosbridge_url(rosbridge_url)
    ros = roslibpy.Ros(host=host, port=port, is_secure=is_secure)

    print(f"[GPS RTK] Connecting to rosbridge at {host}:{port} secure={is_secure}")

    try:
        try:
            ros.run(timeout=10)
        except TypeError:
            ros.run()

        if not is_ros_connected(ros):
            print("[GPS RTK] Not connected to rosbridge.")
            return

        print("[GPS RTK] Connected. Requesting topic list...")

        response = _call_ros_service_sync(
            ros,
            service_name="/rosapi/topics",
            service_type="rosapi/Topics",
            request_dict={},
            timeout=5.0,
        )

        topics = []
        if isinstance(response, dict):
            topics = response.get("topics", [])

        if not topics:
            print("[GPS RTK] No topics returned. Check whether rosapi is running with rosbridge_server.")
            return

        for topic in topics:
            topic_type = resolve_topic_type(ros, topic, configured_type=None, timeout=1.0)
            print(f"{topic:50s} {topic_type}")

    finally:
        try:
            ros.terminate()
        except Exception:
            pass


def is_ros_connected(ros: Any) -> bool:
    """Return ros.is_connected safely for different roslibpy versions."""
    value = getattr(ros, "is_connected", False)
    if callable(value):
        try:
            return bool(value())
        except Exception:
            return False
    return bool(value)


# =============================================================================
# MAIN LOGGER FUNCTION FOR MASTERCONTROLSTATION
# =============================================================================

def run_gps_rtk_logger(stop_event: threading.Event,
                       config: Dict[str, Any],
                       save_dir: str,
                       data_queue: Optional[Any] = None) -> None:
    """
    Main function called by MasterControlStation.

    Args:
        stop_event:
            Threading event used by MasterControlStation to stop the measurement.

        config:
            Configuration dictionary from MasterControlStation.

            Useful keys:
                gps_rtk_topic / gps_topic / topic / ros_topic
                gps_rtk_ros_master / rosbridge_url / rosbridge
                gps_rtk_message_type / message_type / topic_type
                gps_rtk_frame / frame_id
                send_matlab / send_data_to_matlab
                matlab_host
                matlab_gt_port / matlab_port / udp_port
                session_name
                session_dir

        save_dir:
            Measurement session folder.

        data_queue:
            Queue used by MasterControlStation for live plotting.
    """
    try:
        import roslibpy
    except ImportError:
        print("[GPS RTK ERROR] roslibpy is not installed.")
        print("[GPS RTK ERROR] Install it on Windows with: pip install roslibpy")
        return

    # -------------------------------------------------------------------------
    # Read configuration
    # -------------------------------------------------------------------------
    topic_name = str(get_config_value(
        config,
        ["gps_rtk_topic", "gps_topic", "topic", "ros_topic"],
        "",
    )).strip()

    rosbridge_url = str(get_config_value(
        config,
        ["gps_rtk_ros_master", "rosbridge_url", "rosbridge", "gps_ros_master"],
        DEFAULT_ROSBRIDGE_URL,
    )).strip()

    configured_message_type = get_config_value(
        config,
        ["gps_rtk_message_type", "message_type", "topic_type", "ros_message_type"],
        None,
    )

    frame_id = str(get_config_value(
        config,
        ["gps_rtk_frame", "frame_id", "gps_frame"],
        "map",
    ))

    send_matlab = safe_bool(get_config_value(
        config,
        ["send_matlab", "send_data_to_matlab"],
        False,
    ))

    matlab_host = str(get_config_value(
        config,
        ["matlab_host", "udp_ip"],
        DEFAULT_UDP_IP,
    ))

    matlab_port = int(get_config_value(
        config,
        ["matlab_gt_port", "matlab_port", "udp_port"],
        DEFAULT_UDP_PORT_GROUND_TRUTH,
    ))

    print_interval = float(get_config_value(
        config,
        ["print_interval", "gps_rtk_print_interval"],
        0.25,
    ))

    session_name = make_session_name(config)
    ensure_dir(save_dir)

    if not topic_name:
        print("[GPS RTK ERROR] No GPS RTK ROS topic configured.")
        print("[GPS RTK ERROR] Fill in the GPS RTK Topic field in MasterControlStation.")
        print("[GPS RTK ERROR] Example: /gps_rtk/pose or /gps/odom")
        return

    # -------------------------------------------------------------------------
    # File setup
    # -------------------------------------------------------------------------
    filename = os.path.join(save_dir, f"[Log]_GPSRTK_{session_name}.csv")
    error_filename = os.path.join(save_dir, f"[Log]_errors_GPSRTK_{session_name}.csv")

    # -------------------------------------------------------------------------
    # Network setup
    # -------------------------------------------------------------------------
    host, port, is_secure = parse_rosbridge_url(rosbridge_url)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    ros = None
    topic = None
    message_counter = 0
    last_print_time = [0.0]
    first_message_time = [None]

    print("[GPS RTK] Starting GPS RTK ROS bridge logger")
    print(f"[GPS RTK] rosbridge: {host}:{port} secure={is_secure}")
    print(f"[GPS RTK] topic: {topic_name}")
    print(f"[GPS RTK] frame_id: {frame_id}")
    print(f"[GPS RTK] CSV file: {filename}")

    if send_matlab:
        print(f"[GPS RTK UDP] Sending live GPS RTK data to MATLAB at {matlab_host}:{matlab_port}")
    else:
        print("[GPS RTK UDP] MATLAB sending disabled")

    try:
        with open(filename, mode="w", newline="") as file, open(error_filename, mode="w", newline="") as error_file:
            writer = csv.writer(file)
            writer.writerow(["Time", "POSX", "POSY", "POSZ"])

            error_writer = csv.writer(error_file)
            error_writer.writerow(["Time", "ErrorType", "Details"])

            # -----------------------------------------------------------------
            # Connect to rosbridge
            # -----------------------------------------------------------------
            ros = roslibpy.Ros(host=host, port=port, is_secure=is_secure)

            try:
                ros.run(timeout=10)
            except TypeError:
                ros.run()

            # Wait a little for the connection to become active.
            connect_deadline = time.time() + 10.0
            while not is_ros_connected(ros) and time.time() < connect_deadline:
                if stop_event.is_set():
                    return
                time.sleep(0.1)

            if not is_ros_connected(ros):
                log_gps_error(error_writer, error_file, "connection_failed",
                              f"Could not connect to rosbridge at {host}:{port}")
                return

            print("[GPS RTK] Connected to rosbridge.")

            message_type = resolve_topic_type(
                ros,
                topic_name=topic_name,
                configured_type=configured_message_type,
                timeout=3.0,
            )

            print(f"[GPS RTK] Subscribing with message type: {message_type}")

            # -----------------------------------------------------------------
            # Callback for incoming ROS messages
            # -----------------------------------------------------------------
            def callback(message: Dict[str, Any]) -> None:
                nonlocal message_counter

                if stop_event.is_set():
                    return

                try:
                    # User requested Windows receive time.
                    timestamp = time.time()

                    x, y, z, extraction_method = extract_xyz_from_message(message)
                    pos = (x, y, z)

                    writer.writerow([
                        timestamp,
                        round(float(x), 4),
                        round(float(y), 4),
                        round(float(z), 4),
                    ])
                    file.flush()

                    if first_message_time[0] is None:
                        first_message_time[0] = timestamp
                        print(f"[GPS RTK] First GPS RTK message received using: {extraction_method}")

                    # Send data to MasterControlStation for live plotting.
                    if data_queue is not None:
                        packet = build_mastercontrol_packet(
                            timestamp=timestamp,
                            pos=pos,
                            frame_id=frame_id,
                            message_type=message_type,
                            topic=topic_name,
                            extraction_method=extraction_method,
                        )
                        data_queue.put(packet)

                    # Send data to MatlabMasterControl over UDP.
                    if send_matlab:
                        data_msg = build_matlab_packet(timestamp, pos)
                        udp_sock.sendto(data_msg.encode("utf-8"), (matlab_host, matlab_port))

                    message_counter += 1

                    current_time = time.time()
                    if current_time - last_print_time[0] >= print_interval:
                        print(f"[GPS RTK] Read Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
                        last_print_time[0] = current_time

                except Exception as exc:
                    log_gps_error(error_writer, error_file, "message_parse_exception", str(exc))

            topic = roslibpy.Topic(ros, topic_name, message_type)
            topic.subscribe(callback)

            print("[GPS RTK] Subscribed. Waiting for GPS RTK data...")
            print("[GPS RTK] Stop the measurement from MasterControlStation, or press Ctrl+C in standalone mode.")

            while not stop_event.is_set():
                # If rosbridge disconnects during measurement, stop cleanly.
                if not is_ros_connected(ros):
                    log_gps_error(error_writer, error_file, "rosbridge_disconnected",
                                  "rosbridge connection was lost during measurement")
                    break

                time.sleep(0.1)

            print(f"[GPS RTK] Stopping. Total messages logged: {message_counter}")

    except KeyboardInterrupt:
        print("[GPS RTK] KeyboardInterrupt received. Stopping.")

    except Exception as exc:
        print(f"[GPS RTK ERROR] Logger crashed: {exc}")

    finally:
        try:
            if topic is not None:
                topic.unsubscribe()
        except Exception:
            pass

        try:
            if ros is not None:
                ros.terminate()
        except Exception:
            pass

        try:
            udp_sock.close()
        except Exception:
            pass

        print("[GPS RTK] Stopped.")


# =============================================================================
# Standalone command-line testing
# =============================================================================

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read GPS RTK XYZ position from ROS 1 through rosbridge_server."
    )

    parser.add_argument(
        "--rosbridge",
        default=DEFAULT_ROSBRIDGE_URL,
        help=f"rosbridge websocket URL. Default: {DEFAULT_ROSBRIDGE_URL}",
    )

    parser.add_argument(
        "--topic",
        default="",
        help="ROS topic containing XYZ position. Example: /gps_rtk/pose",
    )

    parser.add_argument(
        "--message-type",
        default=None,
        help="Optional ROS message type. If omitted, the script tries rosapi topic_type.",
    )

    parser.add_argument(
        "--frame-id",
        default="map",
        help="Coordinate frame name stored in the live GUI packet metadata.",
    )

    parser.add_argument(
        "--save-dir",
        default=".",
        help="Folder where the GPS RTK CSV file will be written.",
    )

    parser.add_argument(
        "--session-name",
        default=None,
        help="Optional session name. Default: generated Session_YYYYMMDD_HHMMSS.",
    )

    parser.add_argument(
        "--send-matlab",
        action="store_true",
        help="Send live ground-truth UDP packets to MatlabMasterControl.",
    )

    parser.add_argument(
        "--matlab-host",
        default=DEFAULT_UDP_IP,
        help=f"MATLAB UDP host. Default: {DEFAULT_UDP_IP}",
    )

    parser.add_argument(
        "--matlab-port",
        type=int,
        default=DEFAULT_UDP_PORT_GROUND_TRUTH,
        help=f"MATLAB ground-truth UDP port. Default: {DEFAULT_UDP_PORT_GROUND_TRUTH}",
    )

    parser.add_argument(
        "--list-topics",
        action="store_true",
        help="List ROS topics through rosbridge and exit. No separate probe script needed.",
    )

    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.list_topics:
        list_ros_topics(args.rosbridge)
        return

    if not args.topic:
        parser.error("--topic is required unless --list-topics is used.")

    stop_event = threading.Event()

    config = {
        "gps_rtk_topic": args.topic,
        "gps_rtk_ros_master": args.rosbridge,
        "gps_rtk_message_type": args.message_type,
        "gps_rtk_frame": args.frame_id,
        "send_matlab": args.send_matlab,
        "matlab_host": args.matlab_host,
        "matlab_gt_port": args.matlab_port,
    }

    if args.session_name:
        config["session_name"] = args.session_name

    try:
        run_gps_rtk_logger(stop_event, config, args.save_dir, data_queue=None)
    except KeyboardInterrupt:
        stop_event.set()
        print("[GPS RTK] Ctrl+C received.")


if __name__ == "__main__":
    main()
