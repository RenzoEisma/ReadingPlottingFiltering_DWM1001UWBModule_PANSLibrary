import threading
import time
import os
import sys
import json
import tkinter as tk
from tkinter import ttk, scrolledtext
from datetime import datetime

from drivers.DataDescriptions import DataDescriptions
from drivers.NatNetClient import run_simple_logger
from drivers import uwb_sensor
from drivers import ComparisonReportMaker

stop_event = threading.Event()


class ConsoleRedirector:
    def __init__(self, text_widget):
        self.text_widget = text_widget

    def write(self, string):
        self.text_widget.insert(tk.END, string)
        self.text_widget.see(tk.END)

    def flush(self):
        pass


class MasterLoggerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Master Logger Control Panel")
        self.settings_file = "../logger_settings.json"

        self.enable_uwb = tk.BooleanVar(value=True)
        self.enable_gt = tk.BooleanVar(value=False)
        self.gt_type = tk.StringVar(value="OptiTrack")
        self.plot_results = tk.BooleanVar(value=True)

        self.uwb_port1 = tk.StringVar(value='COM3')
        self.enable_uwb_port2 = tk.BooleanVar(value=True)
        self.uwb_port2 = tk.StringVar(value='COM5')
        self.opti_server = tk.StringVar(value='192.168.1.188')

        self.load_settings()
        self.setup_ui()

        sys.stdout = ConsoleRedirector(self.console_output)

    def load_settings(self):
        if os.path.exists(self.settings_file):
            try:
                with open(self.settings_file, 'r') as f:
                    data = json.load(f)
                    self.enable_uwb.set(data.get("enable_uwb", True))
                    self.enable_gt.set(data.get("enable_gt", False))
                    self.gt_type.set(data.get("gt_type", "OptiTrack"))
                    self.plot_results.set(data.get("plot_results", True))
                    self.uwb_port1.set(data.get("uwb_port1", "COM3"))
                    self.enable_uwb_port2.set(data.get("enable_uwb_port2", True))
                    self.uwb_port2.set(data.get("uwb_port2", "COM5"))
                    self.opti_server.set(data.get("opti_server", "192.168.1.188"))
            except Exception as e:
                print(f"Could not load settings: {e}")

    def save_settings(self):
        data = {
            "enable_uwb": self.enable_uwb.get(),
            "enable_gt": self.enable_gt.get(),
            "gt_type": self.gt_type.get(),
            "plot_results": self.plot_results.get(),
            "uwb_port1": self.uwb_port1.get(),
            "enable_uwb_port2": self.enable_uwb_port2.get(),
            "uwb_port2": self.uwb_port2.get(),
            "opti_server": self.opti_server.get()
        }
        try:
            with open(self.settings_file, 'w') as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            print(f"Could not save settings: {e}")

    def setup_ui(self):
        settings_frame = tk.Frame(self.root, padx=10, pady=10)
        settings_frame.pack(fill=tk.X)

        tk.Checkbutton(settings_frame, text="Enable UWB", variable=self.enable_uwb).grid(row=0, column=0, sticky="w")
        tk.Label(settings_frame, text="UWB Port 1:").grid(row=0, column=1, padx=5, sticky="e")
        tk.Entry(settings_frame, textvariable=self.uwb_port1, width=15).grid(row=0, column=2)

        tk.Checkbutton(settings_frame, text="Enable UWB Port 2", variable=self.enable_uwb_port2).grid(row=1, column=1,
                                                                                                      sticky="w")
        tk.Entry(settings_frame, textvariable=self.uwb_port2, width=15).grid(row=1, column=2)

        tk.Frame(settings_frame, height=10).grid(row=2, column=0, columnspan=3)

        tk.Checkbutton(settings_frame, text="Enable Ground Truth", variable=self.enable_gt).grid(row=3, column=0,
                                                                                                 sticky="w")
        gt_dropdown = ttk.Combobox(settings_frame, textvariable=self.gt_type, values=["OptiTrack", "GPS"],
                                   state="readonly", width=12)
        gt_dropdown.grid(row=3, column=1, sticky="w", padx=5)

        tk.Label(settings_frame, text="Opti Server IP:").grid(row=4, column=1, padx=5, sticky="e")
        tk.Entry(settings_frame, textvariable=self.opti_server, width=15).grid(row=4, column=2)

        tk.Frame(settings_frame, height=10).grid(row=5, column=0, columnspan=3)
        tk.Checkbutton(settings_frame, text="Plot Results on Stop", variable=self.plot_results).grid(row=6, column=0,
                                                                                                     sticky="w")

        control_frame = tk.Frame(self.root, pady=10)
        control_frame.pack()

        self.start_btn = tk.Button(control_frame, text="Start Logging", command=self.start_logging, bg="green",
                                   fg="white", width=15)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = tk.Button(control_frame, text="Stop Logging", command=self.stop_logging, bg="red", fg="white",
                                  state=tk.DISABLED, width=15)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        self.console_output = scrolledtext.ScrolledText(self.root, height=15, width=70)
        self.console_output.pack(padx=10, pady=10)

    def start_logging(self):
        self.save_settings()
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.console_output.delete(1.0, tk.END)
        stop_event.clear()

        self.log_thread = threading.Thread(target=self.run_master_process)
        self.log_thread.start()

    def stop_logging(self):
        print("\n[MASTER] Shutdown initiated...")
        stop_event.set()
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

    def run_master_process(self):
        print("=== MASTER LOGGER STARTING ===")

        base_dir = "../measurements"
        session_name = datetime.now().strftime("Session_%Y%m%d_%H%M%S")
        session_dir = os.path.join(base_dir, session_name)

        if not os.path.exists(session_dir):
            os.makedirs(session_dir)

        threads = []

        uwb_config = {
            'port1': self.uwb_port1.get(),
            'port2': self.uwb_port2.get() if self.enable_uwb_port2.get() else None,
            'baud': 115200,
            'latency': 0
        }

        if self.enable_gt.get():
            if self.gt_type.get() == "OptiTrack":
                opti_config = {
                    'server_ip': self.opti_server.get(),
                    'client_ip': "192.168.1.15",
                    'multicast': False,
                    'latency': 0
                }
                t_opti = threading.Thread(target=run_simple_logger, args=(stop_event, opti_config, session_dir))
                t_opti.start()
                threads.append(t_opti)
            elif self.gt_type.get() == "GPS":
                print("[MASTER] GPS ground truth selected (Placeholder for GPS launch logic)")
                # gps_config = {'GPS_PORT': 'COM4', 'latency': 0.015}
                # t_gps = threading.Thread(target=gps_sensor.run_gps, args=(stop_event, gps_config, session_dir))
                # t_gps.start()
                # threads.append(t_gps)

        if self.enable_uwb.get():
            t_uwb = threading.Thread(target=uwb_sensor.run_uwb, args=(stop_event, uwb_config, session_dir))
            t_uwb.start()
            threads.append(t_uwb)

        print(f"\n[MASTER] Saving data to: {session_dir}")
        print("[MASTER] Running. Press 'Stop Logging' to halt.\n")

        while not stop_event.is_set():
            time.sleep(1)

        for t in threads:
            t.join()

        print("[MASTER] All sensors stopped.")

        if self.plot_results.get():
            print("\n[MASTER] Generating report...")
            ComparisonReportMaker.run_dashboard(session_dir)
        print("\n[MASTER] Process Complete.")


if __name__ == "__main__":
    root = tk.Tk()
    app = MasterLoggerGUI(root)


    def on_closing():
        app.save_settings()
        app.stop_logging()
        root.destroy()


    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
























# import threading
# import time
# import os
# from datetime import datetime
#
# from drivers.DataDescriptions import DataDescriptions
# from drivers.NatNetClient import run_simple_logger
# from drivers import uwb_sensor
# from drivers import ComparisonReportMaker
# # import gps_sensor  <-- uncomment this later
#
# # ===================== PROGRAM_INFO =====================
# """
#     Author: Renzo Eisma
#     Version: 2.0
#     Date: 02/20/2026
#     Description: Group of scripts for measuring difference
#     between UWB sensor and ground truth sensor
# """
#
#
# # ========================================================
# # ================= MASTER CONFIGURATION =================
# # ========================================================
# ENABLE_UWB = True
# ENABLE_OPTI = False
# ENABLE_GPS = False  # Set to True when you add the GPS module
#
# UWB_CONFIG = {
#     'port1': 'COM3',
#     'port2': 'COM5',
#     'baud': 115200,
#     'latency': 0
# }
# OPTI_CONFIG = {
#     'server_ip': "192.168.1.188",
#     'client_ip': "192.168.1.15",
#     'multicast': False,
#     'latency': 0
# }
# # GPS_CONFIG = {
# #     'GPS_PORT' = 'COM4',
# #     'latency': 0.015
# # }
#
# PLOT_RESULTS = True
# # Plotter_CONFIG = {
# #     'Title' = 'Insert_Title_Here',
# #     'number': 542
# # }
#
# # ========================================================
# # ========================================================
# # ========================================================
#
# def main():
#     print("=== MASTER LOGGER STARTING ===")
#
#     # 1.1. Create Folder Structure
#     base_dir = "measurements"
#     session_name = datetime.now().strftime("Session_%Y%m%d_%H%M%S")
#     session_dir = os.path.join(base_dir, session_name)
#
#     if not os.path.exists(session_dir):
#         os.makedirs(session_dir)
#
#     stop_event = threading.Event()
#     threads = []
#
#     # 1.2. Create Configuration file
#     configuration = os.path.join(session_dir, f"[Log]_configuration_{session_name}.txt")
#     with open(configuration, 'w') as file:
#         file.write(f"UWB_CONFIG: {UWB_CONFIG}, OPTI_CONFIG: {OPTI_CONFIG}")
#
#     # 2. Launch OptiTrack (Directly from NatNetClient)
#     if ENABLE_OPTI:
#         t_opti = threading.Thread(
#             target=run_simple_logger,
#             args=(stop_event, OPTI_CONFIG, session_dir)
#         )
#         t_opti.start()
#         threads.append(t_opti)
#
#     # 3. Launch UWB
#     if ENABLE_UWB:
#         t_uwb = threading.Thread(
#             target=uwb_sensor.run_uwb,
#             args=(stop_event, UWB_CONFIG, session_dir)
#         )
#         t_uwb.start()
#         threads.append(t_uwb)
#
#     # # 4. Launch GPS
#     # if ENABLE_GPS:
#     #     t_gps = threading.Thread(
#     #         target=uwb_sensor.run_uwb,
#     #         args=(stop_event, UWB_CONFIG, session_dir)
#     #     )
#     #     t_gps.start()
#     #     threads.append(t_gps)
#
#     print(f"\n[MASTER] Saving data to: {session_dir}")
#     print("[MASTER] Running. Press Ctrl+C to stop.\n")
#
#     try:
#         while not stop_event.is_set():
#             time.sleep(1)
#     except KeyboardInterrupt:
#         print("\n[MASTER] Shutdown initiated...")
#         stop_event.set()
#         for t in threads:
#             t.join()
#         print("[MASTER] All sensors stopped.")
#
#         if PLOT_RESULTS:
#             print("\n[MASTER] Generating report...")
#             ComparisonReportMaker.run_dashboard(session_dir)
#         print("\n[MASTER] Process Complete. Goodbye.")
#
# if __name__ == "__main__":
#     main()