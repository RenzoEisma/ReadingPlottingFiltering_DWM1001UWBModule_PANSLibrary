import threading
import time
import os
from datetime import datetime

from drivers.DataDescriptions import DataDescriptions
from drivers.NatNetClient import run_simple_logger
from drivers import uwb_sensor
from drivers import ComparisonReportMaker
# import gps_sensor  <-- uncomment this later

# ===================== PROGRAM_INFO =====================
"""
    Author: Renzo Eisma
    Version: 2.0
    Date: 02/20/2026
    Description: Group of scripts for measuring difference
    between UWB sensor and ground truth sensor
"""


# ========================================================
# ================= MASTER CONFIGURATION =================
# ========================================================
ENABLE_UWB = True
ENABLE_OPTI = True
ENABLE_GPS = False  # Set to True when you add the GPS module

UWB_CONFIG = {
    'port1': 'COM3',
    'port2': 'COM5',
    'baud': 115200,
    'latency': 0
}
OPTI_CONFIG = {
    'server_ip': "192.168.1.188",
    'client_ip': "192.168.1.15",
    'multicast': False,
    'latency': 0
}
# GPS_CONFIG = {
#     'GPS_PORT' = 'COM4',
#     'latency': 0.015
# }

PLOT_RESULTS = True
# Plotter_CONFIG = {
#     'Title' = 'Insert_Title_Here',
#     'number': 542
# }

# ========================================================
# ========================================================
# ========================================================

def main():
    print("=== MASTER LOGGER STARTING ===")

    # 1.1. Create Folder Structure
    base_dir = "measurements"
    session_name = datetime.now().strftime("Session_%Y%m%d_%H%M%S")
    session_dir = os.path.join(base_dir, session_name)

    if not os.path.exists(session_dir):
        os.makedirs(session_dir)

    stop_event = threading.Event()
    threads = []

    # 1.2. Create Configuration file
    configuration = os.path.join(session_dir, f"configuration_{session_name}.txt")
    with open(configuration, 'w') as file:
        file.write(f"UWB_CONFIG: {UWB_CONFIG}, OPTI_CONFIG: {OPTI_CONFIG}")

    # 2. Launch OptiTrack (Directly from NatNetClient)
    if ENABLE_OPTI:
        t_opti = threading.Thread(
            target=run_simple_logger,
            args=(stop_event, OPTI_CONFIG, session_dir)
        )
        t_opti.start()
        threads.append(t_opti)

    # 3. Launch UWB
    if ENABLE_UWB:
        t_uwb = threading.Thread(
            target=uwb_sensor.run_uwb,
            args=(stop_event, UWB_CONFIG, session_dir)
        )
        t_uwb.start()
        threads.append(t_uwb)

    # # 4. Launch GPS
    # if ENABLE_GPS:
    #     t_gps = threading.Thread(
    #         target=uwb_sensor.run_uwb,
    #         args=(stop_event, UWB_CONFIG, session_dir)
    #     )
    #     t_gps.start()
    #     threads.append(t_gps)

    print(f"\n[MASTER] Saving data to: {session_dir}")
    print("[MASTER] Running. Press Ctrl+C to stop.\n")

    try:
        while not stop_event.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[MASTER] Shutdown initiated...")
        stop_event.set()
        for t in threads:
            t.join()
        print("[MASTER] All sensors stopped.")

        if PLOT_RESULTS:
            print("\n[MASTER] Generating report...")
            ComparisonReportMaker.run_dashboard(session_dir)
        print("\n[MASTER] Process Complete. Goodbye.")

if __name__ == "__main__":
    main()