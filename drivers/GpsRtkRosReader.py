# ===================== PROGRAM_INFO ==================================================================================
"""
Author: Renzo Eisma
Date: 06/2026
Description: Read GPS RTK latitude/longitude/altitude data from a ROS 1 laptop through rosbridge_server. This script
runs on the Windows laptop, subscribes to one ROS topic, converts WGS84 GPS coordinates to a local XYZ/ENU frame using
a manually configured origin, writes CSV files, sends live position packets to MasterControlStation for plotting, and
optionally sends live ground-truth UDP packets to MatlabMasterControl.

Important:
    This version is intentionally for GPS latitude/longitude/altitude only.
    It expects a ROS topic such as sensor_msgs/NavSatFix, or a custom message with latitude/longitude/altitude fields.

Coordinate convention:
    The manually entered origin latitude/longitude/altitude becomes local (0, 0, 0).
    Local XYZ output uses ENU by default:
        X = East  [m]
        Y = North [m]
        Z = Up    [m]

The script uses Windows receive time as the main timestamp. This matches the rest of the measurement framework, but
it includes ROS/rosbridge/network delay. The original ROS header timestamp is stored only as metadata.

Assistance note: ChatGPT Pro 5.5 Thinking Extended was used to make almost the full script. First, a basic structure was
layed out by a human but AI worked it out. It is also untested due to the GPS RTK system at Lab Air not working
properly yet before this project was finished. Therefore, if this code ever gets used, use it as a basis but don't
blindly trust that everything will work.
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
#   python GpsRtkRosReader.py --rosbridge ws://192.168.50.20:9090 --topic /fix --origin-lat -20.2750 --origin-lon -40.3050 --origin-alt 5.0 --save-dir ./test


from __future__ import annotations

import argparse
import csv
import math
import os
import socket
import threading
import time
from datetime import datetime
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.parse import urlparse


DEFAULT_ROSBRIDGE_URL = "ws://127.0.0.1:9090"
DEFAULT_MESSAGE_TYPE = "sensor_msgs/NavSatFix"

DEFAULT_UDP_IP = "127.0.0.1"
DEFAULT_UDP_PORT_GROUND_TRUTH = 5006

SOURCE_TYPE = "gps_rtk"
DEFAULT_RIGID_BODY_ID = 0

# WGS84 ellipsoid constants.
WGS84_A = 6378137.0                         # semi-major axis [m]
WGS84_F = 1.0 / 298.257223563               # flattening [-]
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)        # eccentricity squared [-]


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


def safe_float(value: Any, name: str) -> float:
    """Convert a config/CLI value to float and give a clear error if it fails."""
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be filled in as a number. Current value: {value!r}")


def parse_rosbridge_url(url_or_host: str) -> Tuple[str, int, bool]:
    """
    Parse a rosbridge URL.

    Accepted examples:
        ws://192.168.50.20:9090
        192.168.50.20:9090
        192.168.50.20
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
# ROS GPS message parsing
# =============================================================================

def _to_float(value: Any) -> Optional[float]:
    """Convert a value to float safely."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def extract_lat_lon_alt_from_message(msg: Dict[str, Any]) -> Tuple[float, float, float, str, Dict[str, Any]]:
    """
    Extract latitude, longitude and altitude from a ROS message dictionary.

    Primary supported format:
        sensor_msgs/NavSatFix:
            msg["latitude"]
            msg["longitude"]
            msg["altitude"]
            msg["status"]["status"]
            msg["status"]["service"]
            msg["position_covariance"]

    Also supported for convenience:
        msg["lat"], msg["lon"], msg["alt"]
        msg["latitude"], msg["longitude"], msg["height"]
        msg["data"] = [latitude, longitude, altitude]

    Returns:
        latitude_deg, longitude_deg, altitude_m, extraction_method, metadata
    """
    if not isinstance(msg, dict):
        raise ValueError(f"ROS message is not a dictionary. Type={type(msg)}")

    candidates: List[Tuple[str, Any, Any, Any]] = [
        (
            "sensor_msgs/NavSatFix: latitude/longitude/altitude",
            msg.get("latitude"),
            msg.get("longitude"),
            msg.get("altitude"),
        ),
        (
            "custom: lat/lon/alt",
            msg.get("lat"),
            msg.get("lon"),
            msg.get("alt"),
        ),
        (
            "custom: latitude/longitude/height",
            msg.get("latitude"),
            msg.get("longitude"),
            msg.get("height"),
        ),
    ]

    for method, lat_value, lon_value, alt_value in candidates:
        lat = _to_float(lat_value)
        lon = _to_float(lon_value)
        alt = _to_float(alt_value)

        if lat is not None and lon is not None and alt is not None:
            metadata = extract_gps_metadata(msg)
            return lat, lon, alt, method, metadata

    data = msg.get("data")
    if isinstance(data, (list, tuple)) and len(data) >= 3:
        lat = _to_float(data[0])
        lon = _to_float(data[1])
        alt = _to_float(data[2])
        if lat is not None and lon is not None and alt is not None:
            metadata = extract_gps_metadata(msg)
            return lat, lon, alt, "custom: data array [lat, lon, alt]", metadata

    raise ValueError(f"Could not find latitude/longitude/altitude in ROS message. Top-level keys: {list(msg.keys())}")


def extract_gps_metadata(msg: Dict[str, Any]) -> Dict[str, Any]:
    """Extract optional GPS status/covariance metadata from NavSatFix-like messages."""
    metadata: Dict[str, Any] = {
        "fix_status": None,
        "service": None,
        "position_covariance_type": None,
        "position_covariance": None,
        "header_stamp": None,
        "header_frame_id": None,
    }

    status = msg.get("status")
    if isinstance(status, dict):
        metadata["fix_status"] = status.get("status")
        metadata["service"] = status.get("service")

    if "position_covariance_type" in msg:
        metadata["position_covariance_type"] = msg.get("position_covariance_type")

    if "position_covariance" in msg:
        metadata["position_covariance"] = msg.get("position_covariance")

    header = msg.get("header")
    if isinstance(header, dict):
        metadata["header_frame_id"] = header.get("frame_id")
        stamp = header.get("stamp")
        if isinstance(stamp, dict):
            secs = _to_float(stamp.get("secs"))
            nsecs = _to_float(stamp.get("nsecs"))
            if secs is not None:
                metadata["header_stamp"] = secs + ((nsecs or 0.0) * 1e-9)

    return metadata


def is_invalid_navsatfix(metadata: Dict[str, Any]) -> bool:
    """
    NavSatFix uses status.status = -1 for STATUS_NO_FIX.
    If the field is missing, the script accepts the message.
    """
    status = metadata.get("fix_status")
    if status is None:
        return False

    try:
        return int(status) < 0
    except (TypeError, ValueError):
        return False


# =============================================================================
# WGS84 latitude/longitude/altitude to local ENU/XYZ conversion
# =============================================================================

def geodetic_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> Tuple[float, float, float]:
    """Convert WGS84 geodetic latitude/longitude/altitude to ECEF coordinates."""
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    radius_n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    x = (radius_n + altitude_m) * cos_lat * cos_lon
    y = (radius_n + altitude_m) * cos_lat * sin_lon
    z = (radius_n * (1.0 - WGS84_E2) + altitude_m) * sin_lat

    return x, y, z


def geodetic_to_enu(latitude_deg: float,
                    longitude_deg: float,
                    altitude_m: float,
                    origin_latitude_deg: float,
                    origin_longitude_deg: float,
                    origin_altitude_m: float) -> Tuple[float, float, float]:
    """
    Convert WGS84 latitude/longitude/altitude to local ENU metres relative to the configured origin.

    Output:
        east_m, north_m, up_m
    """
    x, y, z = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)
    x0, y0, z0 = geodetic_to_ecef(origin_latitude_deg, origin_longitude_deg, origin_altitude_m)

    lat0 = math.radians(origin_latitude_deg)
    lon0 = math.radians(origin_longitude_deg)

    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    dx = x - x0
    dy = y - y0
    dz = z - z0

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz

    return east, north, up


# =============================================================================
# Packet builders
# =============================================================================

def build_mastercontrol_packet(timestamp: float,
                               pos: Tuple[float, float, float],
                               raw_gps: Tuple[float, float, float],
                               origin_gps: Tuple[float, float, float],
                               frame_id: str = "map",
                               message_type: str = "",
                               topic: str = "",
                               extraction_method: str = "",
                               gps_metadata: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Build the dictionary packet sent to MasterControlStation for live plotting.

    MasterControlStation receives normal local XYZ ground-truth position values.
    Raw GPS and origin information are stored in metadata for debugging.
    """
    x, y, z = pos
    lat, lon, alt = raw_gps
    origin_lat, origin_lon, origin_alt = origin_gps

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

        "quality": {
            "valid": True,
            "source": SOURCE_TYPE,
            "fix_status": None if gps_metadata is None else gps_metadata.get("fix_status"),
        },

        "metadata": {
            "frame_id": frame_id,
            "topic": topic,
            "message_type": message_type,
            "extraction_method": extraction_method,
            "coordinate_system": "ENU",
            "x_axis": "East",
            "y_axis": "North",
            "z_axis": "Up",
            "latitude": float(lat),
            "longitude": float(lon),
            "altitude": float(alt),
            "origin_latitude": float(origin_lat),
            "origin_longitude": float(origin_lon),
            "origin_altitude": float(origin_alt),
            "gps_metadata": gps_metadata or {},
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
        return str(configured_type)

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

            Required keys for this GPS lat/lon/alt version:
                gps_origin_lat / origin_lat / gps_rtk_origin_lat
                gps_origin_lon / origin_lon / gps_rtk_origin_lon
                gps_origin_alt / origin_alt / gps_rtk_origin_alt

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
                skip_invalid_fix

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

    origin_lat_value = get_config_value(
        config,
        ["gps_origin_lat", "origin_lat", "gps_rtk_origin_lat", "gps_rtk_origin_latitude"],
        None,
    )

    origin_lon_value = get_config_value(
        config,
        ["gps_origin_lon", "origin_lon", "gps_rtk_origin_lon", "gps_rtk_origin_longitude"],
        None,
    )

    origin_alt_value = get_config_value(
        config,
        ["gps_origin_alt", "origin_alt", "gps_rtk_origin_alt", "gps_rtk_origin_altitude"],
        None,
    )

    try:
        origin_lat = safe_float(origin_lat_value, "GPS origin latitude")
        origin_lon = safe_float(origin_lon_value, "GPS origin longitude")
        origin_alt = safe_float(origin_alt_value, "GPS origin altitude")
    except ValueError as exc:
        print(f"[GPS RTK ERROR] {exc}")
        print("[GPS RTK ERROR] Fill in the GPS origin lat/lon/alt fields. This origin becomes local XYZ (0,0,0).")
        return

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

    skip_invalid_fix = safe_bool(get_config_value(
        config,
        ["skip_invalid_fix", "gps_rtk_skip_invalid_fix"],
        True,
    ), default=True)

    session_name = make_session_name(config)
    ensure_dir(save_dir)

    if not topic_name:
        print("[GPS RTK ERROR] No GPS RTK ROS topic configured.")
        print("[GPS RTK ERROR] Fill in the GPS RTK Topic field in MasterControlStation.")
        print("[GPS RTK ERROR] Example: /fix or /gps/fix")
        return

    # -------------------------------------------------------------------------
    # File setup
    # -------------------------------------------------------------------------
    filename = os.path.join(save_dir, f"[Log]_GPSRTK_{session_name}.csv")
    raw_filename = os.path.join(save_dir, f"[Log]_GPSRTK_RAW_{session_name}.csv")
    error_filename = os.path.join(save_dir, f"[Log]_errors_GPSRTK_{session_name}.csv")

    # -------------------------------------------------------------------------
    # Network setup
    # -------------------------------------------------------------------------
    host, port, is_secure = parse_rosbridge_url(rosbridge_url)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    ros = None
    topic = None
    message_counter = 0
    skipped_counter = 0
    last_print_time = [0.0]
    first_message_time = [None]
    origin_gps = (origin_lat, origin_lon, origin_alt)

    print("[GPS RTK] Starting GPS RTK lat/lon/alt ROS bridge logger")
    print(f"[GPS RTK] rosbridge: {host}:{port} secure={is_secure}")
    print(f"[GPS RTK] topic: {topic_name}")
    print(f"[GPS RTK] frame_id: {frame_id}")
    print(f"[GPS RTK] origin lat/lon/alt: {origin_lat:.10f}, {origin_lon:.10f}, {origin_alt:.3f}")
    print("[GPS RTK] output axes: X=East, Y=North, Z=Up")
    print(f"[GPS RTK] CSV file: {filename}")
    print(f"[GPS RTK] RAW CSV file: {raw_filename}")

    if send_matlab:
        print(f"[GPS RTK UDP] Sending live GPS RTK XYZ data to MATLAB at {matlab_host}:{matlab_port}")
    else:
        print("[GPS RTK UDP] MATLAB sending disabled")

    try:
        with open(filename, mode="w", newline="") as file, \
             open(raw_filename, mode="w", newline="") as raw_file, \
             open(error_filename, mode="w", newline="") as error_file:

            writer = csv.writer(file)
            writer.writerow(["Time", "POSX", "POSY", "POSZ"])

            raw_writer = csv.writer(raw_file)
            raw_writer.writerow([
                "Time",
                "Latitude",
                "Longitude",
                "Altitude",
                "OriginLatitude",
                "OriginLongitude",
                "OriginAltitude",
                "East_X",
                "North_Y",
                "Up_Z",
                "FixStatus",
                "Service",
                "CovarianceType",
                "ExtractionMethod",
            ])

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
            if message_type != "sensor_msgs/NavSatFix":
                print("[GPS RTK WARNING] This version expects latitude/longitude/altitude data.")
                print("[GPS RTK WARNING] It will still try to parse fields named latitude/longitude/altitude or lat/lon/alt.")

            # -----------------------------------------------------------------
            # Callback for incoming ROS messages
            # -----------------------------------------------------------------
            def callback(message: Dict[str, Any]) -> None:
                nonlocal message_counter, skipped_counter

                if stop_event.is_set():
                    return

                try:
                    # User requested Windows receive time.
                    timestamp = time.time()

                    lat, lon, alt, extraction_method, gps_metadata = extract_lat_lon_alt_from_message(message)

                    if skip_invalid_fix and is_invalid_navsatfix(gps_metadata):
                        skipped_counter += 1
                        log_gps_error(
                            error_writer,
                            error_file,
                            "invalid_navsatfix_skipped",
                            f"NavSatFix status.status={gps_metadata.get('fix_status')} lat={lat} lon={lon} alt={alt}",
                        )
                        return

                    east, north, up = geodetic_to_enu(
                        latitude_deg=lat,
                        longitude_deg=lon,
                        altitude_m=alt,
                        origin_latitude_deg=origin_lat,
                        origin_longitude_deg=origin_lon,
                        origin_altitude_m=origin_alt,
                    )

                    pos = (east, north, up)
                    raw_gps = (lat, lon, alt)

                    # Main CSV stays compatible with OptiTrack-like ground truth processing.
                    writer.writerow([
                        timestamp,
                        round(float(east), 4),
                        round(float(north), 4),
                        round(float(up), 4),
                    ])
                    file.flush()

                    # Raw debug CSV keeps GPS values for checking/conversion debugging.
                    raw_writer.writerow([
                        timestamp,
                        f"{lat:.10f}",
                        f"{lon:.10f}",
                        f"{alt:.4f}",
                        f"{origin_lat:.10f}",
                        f"{origin_lon:.10f}",
                        f"{origin_alt:.4f}",
                        f"{east:.4f}",
                        f"{north:.4f}",
                        f"{up:.4f}",
                        gps_metadata.get("fix_status"),
                        gps_metadata.get("service"),
                        gps_metadata.get("position_covariance_type"),
                        extraction_method,
                    ])
                    raw_file.flush()

                    if first_message_time[0] is None:
                        first_message_time[0] = timestamp
                        print(f"[GPS RTK] First GPS RTK message received using: {extraction_method}")

                    # Send data to MasterControlStation for live plotting.
                    if data_queue is not None:
                        packet = build_mastercontrol_packet(
                            timestamp=timestamp,
                            pos=pos,
                            raw_gps=raw_gps,
                            origin_gps=origin_gps,
                            frame_id=frame_id,
                            message_type=message_type,
                            topic=topic_name,
                            extraction_method=extraction_method,
                            gps_metadata=gps_metadata,
                        )
                        data_queue.put(packet)

                    # Send data to MatlabMasterControl over UDP.
                    if send_matlab:
                        data_msg = build_matlab_packet(timestamp, pos)
                        udp_sock.sendto(data_msg.encode("utf-8"), (matlab_host, matlab_port))

                    message_counter += 1

                    current_time = time.time()
                    if current_time - last_print_time[0] >= print_interval:
                        print(
                            "[GPS RTK] "
                            f"Lat={lat:.8f}, Lon={lon:.8f}, Alt={alt:.3f} -> "
                            f"X/E={east:.3f}, Y/N={north:.3f}, Z/U={up:.3f}"
                        )
                        last_print_time[0] = current_time

                except Exception as exc:
                    log_gps_error(error_writer, error_file, "message_parse_exception", str(exc))

            topic = roslibpy.Topic(ros, topic_name, message_type)
            topic.subscribe(callback)

            print("[GPS RTK] Subscribed. Waiting for GPS RTK lat/lon/alt data...")
            print("[GPS RTK] Stop the measurement from MasterControlStation, or press Ctrl+C in standalone mode.")

            while not stop_event.is_set():
                # If rosbridge disconnects during measurement, stop cleanly.
                if not is_ros_connected(ros):
                    log_gps_error(error_writer, error_file, "rosbridge_disconnected",
                                  "rosbridge connection was lost during measurement")
                    break

                time.sleep(0.1)

            print(f"[GPS RTK] Stopping. Total messages logged: {message_counter}. Skipped invalid fixes: {skipped_counter}")

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
        description="Read GPS RTK latitude/longitude/altitude from ROS 1 through rosbridge_server and convert to local XYZ."
    )

    parser.add_argument(
        "--rosbridge",
        default=DEFAULT_ROSBRIDGE_URL,
        help=f"rosbridge websocket URL. Default: {DEFAULT_ROSBRIDGE_URL}",
    )

    parser.add_argument(
        "--topic",
        default="",
        help="ROS topic containing GPS lat/lon/alt. Example: /fix or /gps/fix",
    )

    parser.add_argument(
        "--message-type",
        default=None,
        help="Optional ROS message type. For normal GPS use: sensor_msgs/NavSatFix. If omitted, the script tries rosapi topic_type.",
    )

    parser.add_argument(
        "--frame-id",
        default="map",
        help="Coordinate frame name stored in the live GUI packet metadata.",
    )

    parser.add_argument(
        "--origin-lat",
        type=float,
        default=None,
        required=False,
        help="Manual WGS84 origin latitude in degrees. This becomes local X/Y/Z = 0/0/0.",
    )

    parser.add_argument(
        "--origin-lon",
        type=float,
        default=None,
        required=False,
        help="Manual WGS84 origin longitude in degrees. This becomes local X/Y/Z = 0/0/0.",
    )

    parser.add_argument(
        "--origin-alt",
        type=float,
        default=None,
        required=False,
        help="Manual WGS84 origin altitude in metres. This becomes local X/Y/Z = 0/0/0.",
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
        "--log-invalid-fixes",
        action="store_true",
        help="Do not skip NavSatFix messages with status.status < 0. Default is to skip invalid fixes.",
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

    missing_origin_parts = []
    if args.origin_lat is None:
        missing_origin_parts.append("--origin-lat")
    if args.origin_lon is None:
        missing_origin_parts.append("--origin-lon")
    if args.origin_alt is None:
        missing_origin_parts.append("--origin-alt")

    if missing_origin_parts:
        parser.error("Manual GPS origin is required: " + ", ".join(missing_origin_parts))

    stop_event = threading.Event()

    config = {
        "gps_rtk_topic": args.topic,
        "gps_rtk_ros_master": args.rosbridge,
        "gps_rtk_message_type": args.message_type,
        "gps_rtk_frame": args.frame_id,
        "gps_origin_lat": args.origin_lat,
        "gps_origin_lon": args.origin_lon,
        "gps_origin_alt": args.origin_alt,
        "send_matlab": args.send_matlab,
        "matlab_host": args.matlab_host,
        "matlab_gt_port": args.matlab_port,
        "skip_invalid_fix": not args.log_invalid_fixes,
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