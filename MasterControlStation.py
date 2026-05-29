# ===================== PROGRAM_INFO ==================================================================================
"""
Author: Renzo Eisma
Date: 04/2026
Description: Master Script for everything to do with measuring, plotting and configuring Qorvo UWB modules
"""
# =====================================================================================================================


# =====================================================================================================================
# IMPORTS
# =====================================================================================================================
import threading
import time
import os
import sys
import json
import socket
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
from datetime import datetime
import queue
import csv

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from drivers.NatNetClient import run_simple_logger
from drivers import uwb_sensor
from drivers import ComparisonReportMaker
from drivers import ReadUWBBluetooth


stop_event = threading.Event()


# Class for intercepting print statements and displaying them inside the Tkinter text widget
# ---------------------------------------------------------------------------------------------------------------------
class ConsoleRedirector:
    def __init__(self, text_widget):
        self.text_widget = text_widget

    def write(self, string):
        self.text_widget.insert(tk.END, string)
        self.text_widget.see(tk.END)

    # Required method when overriding standard output in Python
    def flush(self):
        pass


# Main application class that builds and manages the graphical user interface
# ---------------------------------------------------------------------------------------------------------------------
class MasterControlApp:

    # Setup for the main application window
    # -----------------------------------------------------------------------------------------------------------------
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Localization Control Station")  # GUI title
        #self.root.geometry("900x750")  # Define size of GUI
        self.root.state('zoomed')

        # Define settings files
        self.settings_file = "logger_settings.json"  # File for storing saved settings
        self.uwb_config_file = "uwb_network_config.json"

        # UDP settings for communication with the MATLAB master script.
        # MATLAB is NOT started from Python anymore. MatlabMasterControl must be started manually.
        # MasterControlStation only sends the session/settings packet.
        # The sensor scripts send their own live measurement packets to MATLAB.
        self.matlab_host = "127.0.0.1"
        self.matlab_settings_port = 5004
        self.matlab_uwb_port = 5005
        self.matlab_gt_port = 5006
        self.matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Store the active session name and folder for configuration and sensor script setup.
        self.current_session_name = None
        self.current_session_dir = None
        self.measurement_running = False

        # Measurement window timing.
        self.logging_start_time = None
        self.measurement_start_time = None
        self.measurement_stop_time = None
        self.measurement_active = False
        self.measurement_window_path = None

        # Store the latest values that came from the sensor scripts.
        self.system_state = {
            "latest_uwb": None,
            "latest_ground_truth": None,
            "latest_gps_rtk": None,
            "latest_packet_time": None
        }

        # Define Tkinter notebook, this is used for making tabs in the GUI
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Make three tabs
        self.tab_logging = ttk.Frame(self.notebook)
        self.tab_report = ttk.Frame(self.notebook)
        self.tab_anchors = ttk.Frame(self.notebook)

        # Add the tabs to the notebook
        self.notebook.add(self.tab_logging, text="Live Logging / Robot Control")
        self.notebook.add(self.tab_report, text="Report Maker")
        self.notebook.add(self.tab_anchors, text="UWB Anchors")

        # Call all the startup functions
        self.init_logging_vars()
        self.init_report_vars()
        self.init_anchor_vars()
        self.load_settings()
        self.setup_logging_tab()
        self.setup_report_tab()
        self.setup_anchor_tab()

        sys.stdout = ConsoleRedirector(self.console_output)

    # Creates and stores the Tkinter variables related to Hardware Setup and Data Routing
    # -----------------------------------------------------------------------------------------------------------------
    def init_logging_vars(self):
        # Hardware Config
        self.enable_uwb = tk.BooleanVar(value=True)
        self.uwb_source = tk.StringVar(value="Listener")
        self.uwb_port1 = tk.StringVar(value='COM3')
        self.enable_uwb_port2 = tk.BooleanVar(value=True)
        self.uwb_port2 = tk.StringVar(value='COM5')

        self.enable_gt = tk.BooleanVar(value=False)
        self.gt_type = tk.StringVar(value="OptiTrack")
        self.opti_server = tk.StringVar(value='192.168.1.188')
        self.opti_client = tk.StringVar(value='192.168.1.15')

        # GPS RTK placeholder settings. GPS RTK will be read in MATLAB/ROS later, not live-plotted in Python for now.
        self.gps_rtk_topic = tk.StringVar(value='/gps/rtk')
        self.gps_rtk_ros_master = tk.StringVar(value='http://localhost:11311')
        self.gps_rtk_frame = tk.StringVar(value='map')

        # Data Routing Config
        self.send_matlab = tk.BooleanVar(value=False)
        self.send_ros = tk.BooleanVar(value=False)
        self.control_robots = tk.BooleanVar(value=False)
        self.read_type = tk.StringVar(value="Tag Position")
        self.network_scale = tk.StringVar(value="1 Network / 1 Listener")

        # Create arrays for the 3D plot
        self.data_queue = queue.Queue()
        self.plot_x_uwb, self.plot_y_uwb, self.plot_z_uwb = [], [], []
        self.plot_x_gt, self.plot_y_gt, self.plot_z_gt = [], [], []

    # Creates and stores the Tkinter variables used in the Report Maker Tab
    # -----------------------------------------------------------------------------------------------------------------
    def init_report_vars(self):
        self.rep_name = tk.StringVar(value="UWB_Measurement_Default")
        self.rep_notes = tk.StringVar(value="None")
        self.rep_show_anchors = tk.BooleanVar(value=True)
        self.rep_use_measurement_window = tk.BooleanVar(value=True)

    # Creates and stores the Tkinter variables used in the UWB Configuration Tab
    # Initializes an empty list to hold module data and creates Tkinter variables corresponding to the properties
    # of a single UWB module
    # -----------------------------------------------------------------------------------------------------------------
    def init_anchor_vars(self):
        # We will keep an empty list, and load a default if nothing exists
        self.uwb_modules = []
        self.config_dir = "configurations"
        if not os.path.exists(self.config_dir):
            os.makedirs(self.config_dir)

        self.current_module_index = 0
        self.mod_name = tk.StringVar()
        self.mod_address = tk.StringVar()
        self.mod_type = tk.StringVar()
        self.mod_location = tk.StringVar()
        self.mod_network_id = tk.StringVar()
        self.mod_turned_on = tk.BooleanVar()
        self.mod_led_enabled = tk.BooleanVar()

    # =================================================================================================================
    # Data Logger Tab
    # =================================================================================================================

    # Attempts to open and read local JSON files containing previous configurations.
    # -----------------------------------------------------------------------------------------------------------------
    def load_settings(self):
        if os.path.exists(self.settings_file):
            try:
                with open(self.settings_file, 'r') as f:
                    data = json.load(f)

                # Logging settings
                self.enable_uwb.set(data.get("enable_uwb", True))
                self.uwb_source.set(data.get("uwb_source", "Listener"))
                self.uwb_port1.set(data.get("uwb_port1", "COM3"))
                self.enable_uwb_port2.set(data.get("enable_uwb_port2", True))
                self.uwb_port2.set(data.get("uwb_port2", "COM5"))

                self.enable_gt.set(data.get("enable_gt", False))
                self.gt_type.set(data.get("gt_type", "OptiTrack"))
                if self.gt_type.get() == "GPS":
                    self.gt_type.set("GPS RTK")
                self.opti_server.set(data.get("opti_server", "192.168.1.188"))
                self.opti_client.set(data.get("opti_client", "192.168.1.15"))

                self.gps_rtk_topic.set(data.get("gps_rtk_topic", "/gps/rtk"))
                self.gps_rtk_ros_master.set(data.get("gps_rtk_ros_master", "http://localhost:11311"))
                self.gps_rtk_frame.set(data.get("gps_rtk_frame", "map"))

                # Routing settings
                self.send_matlab.set(data.get("send_matlab", False))
                self.send_ros.set(data.get("send_ros", False))
                self.control_robots.set(data.get("control_robots", False))
                self.read_type.set(data.get("read_type", "Tag Position"))

                # Older settings files used the name anchor_count. Convert that value if it is present.
                network_value = data.get("network_scale", data.get("anchor_count", "1 Network / 1 Listener"))
                self.network_scale.set(self.convert_network_scale_to_new_name(network_value))

                # Report settings
                self.rep_name.set(data.get("rep_name", "UWB_Measurement_Default"))
                self.rep_notes.set(data.get("rep_notes", "None"))
                self.rep_show_anchors.set(data.get("rep_show_anchors", True))
                self.rep_use_measurement_window.set(data.get("rep_use_measurement_window", True))

            except Exception as e:
                print(f"Could not load logger settings: {e}")

        if os.path.exists(self.uwb_config_file):
            try:
                with open(self.uwb_config_file, 'r') as f:
                    loaded_modules = json.load(f)
                if isinstance(loaded_modules, list):
                    self.uwb_modules = loaded_modules
            except Exception as e:
                print(f"Could not load UWB config: {e}")

    # Gathers all current inputs from the GUI variables and writes them back into JSON files for persistence.
    # -----------------------------------------------------------------------------------------------------------------
    def save_settings(self):
        self.save_current_module_edits()

        log_data = {
            "enable_uwb": self.enable_uwb.get(),
            "uwb_source": self.uwb_source.get(),
            "uwb_port1": self.uwb_port1.get(),
            "enable_uwb_port2": self.enable_uwb_port2.get(),
            "uwb_port2": self.uwb_port2.get(),

            "enable_gt": self.enable_gt.get(),
            "gt_type": self.gt_type.get(),
            "opti_server": self.opti_server.get(),
            "opti_client": self.opti_client.get(),

            "gps_rtk_topic": self.gps_rtk_topic.get(),
            "gps_rtk_ros_master": self.gps_rtk_ros_master.get(),
            "gps_rtk_frame": self.gps_rtk_frame.get(),

            "send_matlab": self.send_matlab.get(),
            "send_ros": self.send_ros.get(),
            "control_robots": self.control_robots.get(),
            "read_type": self.read_type.get(),
            "network_scale": self.network_scale.get(),

            "rep_name": self.rep_name.get(),
            "rep_notes": self.rep_notes.get(),
            "rep_show_anchors": self.rep_show_anchors.get(),
            "rep_use_measurement_window": self.rep_use_measurement_window.get()
        }

        try:
            with open(self.settings_file, 'w') as f:
                json.dump(log_data, f, indent=4)
        except Exception as e:
            print(f"Could not save logger settings: {e}")

        try:
            with open(self.uwb_config_file, 'w') as f:
                json.dump(self.uwb_modules, f, indent=4)
            print("Settings saved.")
        except Exception as e:
            print(f"Could not save UWB config: {e}")

    # Converts older network scale names from the previous GUI to the new names.
    # -----------------------------------------------------------------------------------------------------------------
    def convert_network_scale_to_new_name(self, value):
        if "8" in str(value) or "2 Network" in str(value) or "2 Listener" in str(value):
            return "2 Networks / 2 Listeners"
        return "1 Network / 1 Listener"

    # Converts the new network scale names back to the old names for temporary compatibility with uwb_sensor.py.
    # -----------------------------------------------------------------------------------------------------------------
    def convert_network_scale_to_legacy_name(self):
        if self.network_scale.get() == "2 Networks / 2 Listeners":
            return "8 Anchors (2 Listeners)"
        return "4 Anchors (1 Listener)"

    # Builds the visual layout for the Live Logging tab
    # -----------------------------------------------------------------------------------------------------------------
    def setup_logging_tab(self):
        # --- Create Main Layout Panes ---
        left_pane = tk.Frame(self.tab_logging)
        left_pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 5), pady=10)

        right_pane = tk.Frame(self.tab_logging)
        right_pane.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 10), pady=10)

        # ==========================================
        # LEFT PANE: Controls and Console
        # ==========================================

        # 1. Hardware Config
        hardware_frame = tk.LabelFrame(left_pane, text="Hardware Setup", padx=10, pady=10)
        hardware_frame.pack(fill=tk.X, pady=5)

        tk.Checkbutton(hardware_frame, text="Enable UWB", variable=self.enable_uwb).grid(row=0, column=0, sticky="w")

        tk.Label(hardware_frame, text="UWB Source:").grid(row=0, column=1, padx=5, sticky="e")
        ttk.Combobox(hardware_frame, textvariable=self.uwb_source, values=["Listener", "ROS"],
                     state="readonly", width=12).grid(row=0, column=2, sticky="w")

        tk.Label(hardware_frame, text="UWB Port 1:").grid(row=1, column=1, padx=5, sticky="e")
        tk.Entry(hardware_frame, textvariable=self.uwb_port1, width=15).grid(row=1, column=2, sticky="w")

        tk.Checkbutton(hardware_frame, text="Enable UWB Port 2", variable=self.enable_uwb_port2).grid(row=2, column=1, sticky="w")
        tk.Entry(hardware_frame, textvariable=self.uwb_port2, width=15).grid(row=2, column=2, sticky="w")

        tk.Frame(hardware_frame, height=10).grid(row=3, column=0, columnspan=3)

        tk.Checkbutton(hardware_frame, text="Enable Ground Truth", variable=self.enable_gt).grid(row=4, column=0, sticky="w")
        gt_dropdown = ttk.Combobox(hardware_frame, textvariable=self.gt_type, values=["OptiTrack", "GPS RTK"],
                                   state="readonly", width=12)
        gt_dropdown.grid(row=4, column=1, sticky="w", padx=5)

        tk.Label(hardware_frame, text="Opti Server IP:").grid(row=5, column=1, padx=5, sticky="e")
        tk.Entry(hardware_frame, textvariable=self.opti_server, width=15).grid(row=5, column=2, sticky="w")

        tk.Label(hardware_frame, text="Opti Local Client IP:").grid(row=6, column=1, padx=5, sticky="e")
        tk.Entry(hardware_frame, textvariable=self.opti_client, width=15).grid(row=6, column=2, sticky="w")

        tk.Label(hardware_frame, text="GPS RTK Topic:").grid(row=7, column=1, padx=5, sticky="e")
        tk.Entry(hardware_frame, textvariable=self.gps_rtk_topic, width=15).grid(row=7, column=2, sticky="w")

        tk.Label(hardware_frame, text="GPS ROS Master:").grid(row=8, column=1, padx=5, sticky="e")
        tk.Entry(hardware_frame, textvariable=self.gps_rtk_ros_master, width=15).grid(row=8, column=2, sticky="w")

        tk.Label(hardware_frame, text="GPS Frame ID:").grid(row=9, column=1, padx=5, sticky="e")
        tk.Entry(hardware_frame, textvariable=self.gps_rtk_frame, width=15).grid(row=9, column=2, sticky="w")

        # 2. Data Routing
        routing_frame = tk.LabelFrame(left_pane, text="Data Routing & Processing", padx=10, pady=10)
        routing_frame.pack(fill=tk.X, pady=5)

        tk.Checkbutton(routing_frame, text="Send Data to MATLAB", variable=self.send_matlab).grid(row=0, column=0, sticky="w", padx=5, pady=2)
        tk.Checkbutton(routing_frame, text="Send Data to ROS", variable=self.send_ros).grid(row=0, column=1, sticky="w", padx=5, pady=2)
        tk.Checkbutton(routing_frame, text="Control Robots", variable=self.control_robots).grid(row=1, column=0, sticky="w", padx=5, pady=2)

        tk.Label(routing_frame, text="Read Mode:").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        ttk.Combobox(routing_frame, textvariable=self.read_type, values=["Tag Position", "Tag Distances"],
                     state="readonly", width=18).grid(row=2, column=1, sticky="w", pady=2)

        tk.Label(routing_frame, text="Network Scale:").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        ttk.Combobox(routing_frame, textvariable=self.network_scale,
                     values=["1 Network / 1 Listener", "2 Networks / 2 Listeners"],
                     state="readonly", width=24).grid(row=3, column=1, sticky="w", pady=2)

        # 3. Controls
        control_frame = tk.LabelFrame(left_pane, text="Measurement Controls", padx=10, pady=10)
        control_frame.pack(fill=tk.X, pady=5)

        self.start_btn = tk.Button(control_frame,text="Start Logging",command=self.start_logging,bg="green",
            fg="white",width=15)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.measure_start_btn = tk.Button(control_frame,text="Start Measuring",command=self.mark_measurement_start,
            bg="#0052cc",fg="white",state=tk.DISABLED,width=15)
        self.measure_start_btn.pack(side=tk.LEFT, padx=5)

        self.measure_stop_btn = tk.Button(control_frame,text="Stop Measuring",command=self.mark_measurement_stop,
            bg="#ff9900",fg="black",state=tk.DISABLED,width=15)
        self.measure_stop_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = tk.Button(
            control_frame,text="Stop Logging",command=self.stop_logging,bg="red",fg="white",state=tk.DISABLED,width=15)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        # 4. Console
        self.console_output = scrolledtext.ScrolledText(left_pane, height=10, width=50)
        self.console_output.pack(fill=tk.BOTH, expand=True, pady=5)

        # ==========================================
        # RIGHT PANE: 3D Live Plot
        # ==========================================
        plot_frame = tk.Frame(right_pane)
        plot_frame.pack(fill=tk.BOTH, expand=True)

        self.fig = plt.figure(figsize=(6, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("Live 3D Trajectory")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")

        self.line_uwb, = self.ax.plot([], [], [], 'b-', label='UWB')
        self.line_gt, = self.ax.plot([], [], [], 'r--', label='GT')
        self.ax.legend()

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # Triggered when the user clicks "Start Logging"
    # Saves current settings, disables the start button, clears any old plotting data, resets the stopping event, and
    # launches the main logging sequence in a separate background thread so the GUI remains responsive. It also starts
    # the recursive GUI plot-updating loop.
    # -----------------------------------------------------------------------------------------------------------------
    def start_logging(self):
        self.save_settings()
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.console_output.delete(1.0, tk.END)

        # Clear previous plot data
        self.plot_x_uwb.clear()
        self.plot_y_uwb.clear()
        self.plot_z_uwb.clear()
        self.plot_x_gt.clear()
        self.plot_y_gt.clear()
        self.plot_z_gt.clear()
        self.line_uwb.set_data_3d([], [], [])
        self.line_gt.set_data_3d([], [], [])
        self.canvas.draw()

        # Clear any old packets that might still be in the queue
        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                break

        self.system_state = {
            "latest_uwb": None,
            "latest_ground_truth": None,
            "latest_gps_rtk": None,
            "latest_packet_time": None
        }

        stop_event.clear()

        self.logging_start_time = time.time()
        self.measurement_start_time = None
        self.measurement_stop_time = None
        self.measurement_active = False
        self.measurement_window_path = None

        self.measurement_running = True

        self.measure_start_btn.config(state=tk.NORMAL)
        self.measure_stop_btn.config(state=tk.DISABLED)

        self.log_thread = threading.Thread(target=self.run_master_process, daemon=True)
        self.log_thread.start()

        # Start the GUI update loop
        self.root.after(100, self.update_live_plot)

    # Triggered when the user clicks "Stop Logging"
    # Sets a threading event flag that tells all background measurement threads to safely terminate
    # -----------------------------------------------------------------------------------------------------------------
    def stop_logging(self):
        print("\n[MASTER] Shutdown initiated...")

        # If the measurement was started but not stopped, stop it automatically.
        if self.measurement_active:
            self.mark_measurement_stop()

        stop_event.set()
        self.measurement_running = False

        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.measure_start_btn.config(state=tk.DISABLED)
        self.measure_stop_btn.config(state=tk.DISABLED)

    # Triggered when the user clicks "Start Measuring"
    # This stores the absolute PC timestamp at which the useful measurement starts.
    # -----------------------------------------------------------------------------------------------------------------
    def mark_measurement_start(self):
        if not self.measurement_running:
            print("[MEASUREMENT] Cannot start measuring. Logging is not running.")
            return

        if self.current_session_dir is None or self.current_session_name is None:
            print("[MEASUREMENT] Cannot start measuring yet. Session folder is not ready.")
            return

        self.measurement_start_time = time.time()
        self.measurement_stop_time = None
        self.measurement_active = True

        self.write_measurement_window_file()

        self.measure_start_btn.config(state=tk.DISABLED)
        self.measure_stop_btn.config(state=tk.NORMAL)

        print(f"[MEASUREMENT] Start marked at PC time: {self.measurement_start_time:.5f}")

    # Triggered when the user clicks "Stop Measuring"
    # This stores the absolute PC timestamp at which the useful measurement stops.
    # -----------------------------------------------------------------------------------------------------------------
    def mark_measurement_stop(self):
        if not self.measurement_running:
            print("[MEASUREMENT] Cannot stop measuring. Logging is not running.")
            return

        if self.measurement_start_time is None:
            print("[MEASUREMENT] Cannot stop measuring. No start time was marked.")
            return

        self.measurement_stop_time = time.time()
        self.measurement_active = False

        self.write_measurement_window_file()

        self.measure_start_btn.config(state=tk.NORMAL)
        self.measure_stop_btn.config(state=tk.DISABLED)

        print(f"[MEASUREMENT] Stop marked at PC time: {self.measurement_stop_time:.5f}")

    # Writes the measurement window CSV into the current measurement folder.
    # The report maker uses this file to crop the plotted/analyzed data.
    # -----------------------------------------------------------------------------------------------------------------
    def write_measurement_window_file(self):
        if self.current_session_dir is None or self.current_session_name is None:
            return

        self.measurement_window_path = os.path.join(
            self.current_session_dir,
            f"[Log]_measurement_window_{self.current_session_name}.csv"
        )

        try:
            with open(self.measurement_window_path, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Event", "PC_Timestamp", "Description", "DateTime"])

                if self.measurement_start_time is not None:
                    writer.writerow([
                        "start",
                        f"{self.measurement_start_time:.5f}",
                        "Measurement started",
                        datetime.fromtimestamp(self.measurement_start_time).strftime("%Y-%m-%d %H:%M:%S.%f")
                    ])

                if self.measurement_stop_time is not None:
                    writer.writerow([
                        "stop",
                        f"{self.measurement_stop_time:.5f}",
                        "Measurement stopped",
                        datetime.fromtimestamp(self.measurement_stop_time).strftime("%Y-%m-%d %H:%M:%S.%f")
                    ])

            print(f"[MEASUREMENT] Window file written: {self.measurement_window_path}")

        except Exception as e:
            print(f"[MEASUREMENT] Failed to write measurement window file: {e}")

    # The core background process
    # Generates new timestamped folder for each session, writes a text file documenting chosen configuration, and
    # packages dictionaries of settings. It then starts individual threads for recording OptiTrack ground truth data and
    # UWB sensor data. It loops until the stop flag is triggered, at which point it joins all threads before closing.
    # -----------------------------------------------------------------------------------------------------------------
    def run_master_process(self):
        print("=== MASTER LOGGER STARTING ===")

        base_dir = os.path.abspath("measurements")
        self.current_session_name = datetime.now().strftime("Session_%Y%m%d_%H%M%S")
        self.current_session_dir = os.path.join(base_dir, self.current_session_name)

        self.measurement_window_path = os.path.join(
            self.current_session_dir,
            f"[Log]_measurement_window_{self.current_session_name}.csv"
        )

        if not os.path.exists(self.current_session_dir):
            os.makedirs(self.current_session_dir)

        self.write_session_configuration_file()

        threads = []

        # ==========================================
        # Ground Truth Setup
        # ==========================================
        if self.enable_gt.get():
            if self.gt_type.get() == "OptiTrack":
                opti_config = {
                    'server_ip': self.opti_server.get(),
                    'client_ip': self.opti_client.get(),
                    'multicast': False,
                    'latency': 0,

                    # OptiTrack-specific script routing. NatNetClient keeps writing its own CSV and sending live
                    # ground-truth packets to MATLAB. MasterControlStation only sends the session/settings packet.
                    'send_matlab': self.send_matlab.get(),
                    'matlab_host': self.matlab_host,
                    'matlab_gt_port': self.matlab_gt_port,
                    'session_name': self.current_session_name,
                    'session_dir': self.current_session_dir
                }

                t_opti = threading.Thread(
                    target=run_simple_logger,
                    args=(stop_event, opti_config, self.current_session_dir, self.data_queue),
                    daemon=True
                )
                t_opti.start()
                threads.append(t_opti)
                print("[MASTER] OptiTrack thread started.")

            elif self.gt_type.get() == "GPS RTK":
                self.start_gps_rtk_reader()

        # ==========================================
        # UWB Setup
        # ==========================================
        if self.enable_uwb.get():
            if self.uwb_source.get() == "Listener":
                uwb_config = {
                    'source': self.uwb_source.get(),
                    'port1': self.uwb_port1.get(),
                    'port2': self.uwb_port2.get() if self.enable_uwb_port2.get() else None,
                    'baud': 115200,
                    'latency': 0,

                    # UWB-specific script routing. The UWB script keeps writing its own CSV and sending live UWB
                    # measurement packets to MATLAB. MasterControlStation only sends the session/settings packet.
                    'send_matlab': self.send_matlab.get(),
                    'send_ros': self.send_ros.get(),
                    'control_robots': self.control_robots.get(),
                    'matlab_host': self.matlab_host,
                    'matlab_uwb_port': self.matlab_uwb_port,

                    # MATLAB is now the only filter target. This value is kept temporarily for uwb_sensor.py compatibility.
                    'filter_type': "MATLAB Filter",

                    # Session and measurement options
                    'session_name': self.current_session_name,
                    'session_dir': self.current_session_dir,
                    'read_type': self.read_type.get(),
                    'network_scale': self.network_scale.get(),
                    'anchor_count': self.convert_network_scale_to_legacy_name()
                }

                t_uwb = threading.Thread(
                    target=uwb_sensor.run_uwb,
                    args=(stop_event, uwb_config, self.current_session_dir, self.data_queue),
                    daemon=True
                )
                t_uwb.start()
                threads.append(t_uwb)
                print("[MASTER] UWB listener thread started.")

            elif self.uwb_source.get() == "ROS":
                self.start_uwb_ros_reader()

        # ==========================================
        # MATLAB Settings Routing
        # ==========================================
        if self.send_matlab.get():
            print("[MASTER] MATLAB UDP communication is enabled.")
            print("[MASTER] MatlabMasterControl must be started manually. Python will not launch MATLAB.")
            print(f"[MASTER] Settings UDP target: {self.matlab_host}:{self.matlab_settings_port}")
            print(f"[MASTER] UWB live data should be sent by uwb_sensor.py to: {self.matlab_host}:{self.matlab_uwb_port}")
            print(f"[MASTER] GT  live data should be sent by NatNetClient.py to: {self.matlab_host}:{self.matlab_gt_port}")
            self.send_settings_to_matlab()

        print(f"\n[MASTER] Saving data to: {self.current_session_dir}")
        print("[MASTER] Running. Press 'Stop Logging' to halt.\n")

        while not stop_event.is_set():
            time.sleep(1)

        for t in threads:
            t.join(timeout=2)

        self.measurement_running = False
        print("[MASTER] All sensors stopped.")

    # Writes a human readable text file with all relevant session configuration settings.
    # -----------------------------------------------------------------------------------------------------------------
    def write_session_configuration_file(self):
        config_file = os.path.join(self.current_session_dir, f"[Log]_configuration_{self.current_session_name}.txt")

        with open(config_file, 'w') as f:
            f.write("MASTER_CONTROL_CONFIGURATION\n")
            f.write("============================================================\n")
            f.write(f"SESSION_NAME: {self.current_session_name}\n")
            f.write(f"SESSION_FOLDER: {self.current_session_dir}\n")
            f.write(f"DATE_TIME: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

            f.write("UWB_CONFIG\n")
            f.write("------------------------------------------------------------\n")
            f.write(f"enable_uwb={self.enable_uwb.get()}\n")
            f.write(f"uwb_source={self.uwb_source.get()}\n")
            f.write(f"uwb_port1={self.uwb_port1.get()}\n")
            f.write(f"enable_uwb_port2={self.enable_uwb_port2.get()}\n")
            f.write(f"uwb_port2={self.uwb_port2.get() if self.enable_uwb_port2.get() else 'None'}\n")
            f.write(f"read_type={self.read_type.get()}\n")
            f.write(f"network_scale={self.network_scale.get()}\n\n")

            f.write("GROUND_TRUTH_CONFIG\n")
            f.write("------------------------------------------------------------\n")
            f.write(f"enable_ground_truth={self.enable_gt.get()}\n")
            f.write(f"ground_truth_type={self.gt_type.get()}\n")
            f.write(f"opti_server_ip={self.opti_server.get()}\n")
            f.write(f"opti_local_client_ip={self.opti_client.get()}\n")
            f.write(f"gps_rtk_topic={self.gps_rtk_topic.get()}\n")
            f.write(f"gps_rtk_ros_master={self.gps_rtk_ros_master.get()}\n")
            f.write(f"gps_rtk_frame={self.gps_rtk_frame.get()}\n\n")

            f.write("ROUTING_CONFIG\n")
            f.write("------------------------------------------------------------\n")
            f.write(f"send_data_to_matlab={self.send_matlab.get()}\n")
            f.write(f"send_data_to_ros={self.send_ros.get()}\n")
            f.write(f"control_robots={self.control_robots.get()}\n")
            f.write("filter_target=MATLAB Filter\n")
            f.write(f"matlab_host={self.matlab_host}\n")
            f.write(f"matlab_settings_port={self.matlab_settings_port}\n")
            f.write(f"matlab_uwb_port={self.matlab_uwb_port}\n")
            f.write(f"matlab_gt_port={self.matlab_gt_port}\n")

    # Placeholder for GPS RTK reading. For now GPS RTK will be handled in MATLAB/ROS and not live-plotted in Python.
    # -----------------------------------------------------------------------------------------------------------------
    def start_gps_rtk_reader(self):
        print("[MASTER] GPS RTK selected as ground truth.")
        print("[MASTER] GPS RTK Python live logging is not implemented yet.")
        print("[MASTER] For now, GPS RTK should be handled by MATLAB/ROS separately.")
        print(f"[MASTER] GPS RTK placeholder topic: {self.gps_rtk_topic.get()}")

    # Placeholder for reading UWB data through ROS. Listener mode is currently the implemented Python route.
    # -----------------------------------------------------------------------------------------------------------------
    def start_uwb_ros_reader(self):
        print("[MASTER] UWB ROS source selected.")
        print("[MASTER] UWB ROS Python live logging is not implemented yet.")
        print("[MASTER] Use Listener source for current testing.")

    # =================================================================================================================
    # MATLAB Settings UDP Routing
    # =================================================================================================================

    # Creates the settings packet that is sent from MasterControlStation to MatlabMasterControl.
    # Live UWB and OptiTrack measurements are not routed here. They are sent from the sensor scripts themselves.
    # -----------------------------------------------------------------------------------------------------------------
    def build_matlab_settings_packet(self):
        return {
            "packet_type": "settings",
            "timestamp": time.time(),
            "session_name": self.current_session_name,
            "session_dir": self.current_session_dir,

            "uwb": {
                "enabled": self.enable_uwb.get(),
                "source": self.uwb_source.get(),
                "port1": self.uwb_port1.get(),
                "port2_enabled": self.enable_uwb_port2.get(),
                "port2": self.uwb_port2.get() if self.enable_uwb_port2.get() else None,
                "read_type": self.read_type.get(),
                "network_scale": self.network_scale.get(),
                "legacy_anchor_count": self.convert_network_scale_to_legacy_name()
            },

            "ground_truth": {
                "enabled": self.enable_gt.get(),
                "type": self.gt_type.get(),
                "opti_server_ip": self.opti_server.get(),
                "opti_local_client_ip": self.opti_client.get(),
                "gps_rtk_topic": self.gps_rtk_topic.get(),
                "gps_rtk_ros_master": self.gps_rtk_ros_master.get(),
                "gps_rtk_frame": self.gps_rtk_frame.get()
            },

            "routing": {
                "send_data_to_matlab": self.send_matlab.get(),
                "send_data_to_ros": self.send_ros.get(),
                "control_robots": self.control_robots.get(),
                "filter_target": "MATLAB Filter",
                "matlab_host": self.matlab_host,
                "matlab_settings_port": self.matlab_settings_port,
                "matlab_uwb_port": self.matlab_uwb_port,
                "matlab_gt_port": self.matlab_gt_port
            }
        }

    # Sends the settings packet to MatlabMasterControl. This is sent once at measurement start.
    # -----------------------------------------------------------------------------------------------------------------
    def send_settings_to_matlab(self):
        if not self.send_matlab.get():
            return

        try:
            packet = self.build_matlab_settings_packet()
            msg = json.dumps(packet)
            self.matlab_socket.sendto(msg.encode('utf-8'), (self.matlab_host, self.matlab_settings_port))
            print("[MASTER UDP] Settings packet sent to MATLAB.")

        except Exception as e:
            print(f"[MASTER UDP] Failed to send settings packet to MATLAB: {e}")

    # =================================================================================================================
    # Report Maker Tab
    # =================================================================================================================

    # Builds the visual layout for the Report Maker tab
    # -----------------------------------------------------------------------------------------------------------------
    def setup_report_tab(self):
        report_frame = tk.Frame(self.tab_report, padx=20, pady=20)
        report_frame.pack(fill=tk.BOTH, expand=True)

        tk.Label(report_frame, text="Generate Standalone Report", font=("Arial", 14)).grid(row=0, column=0, columnspan=3, pady=(0, 20), sticky="w")

        tk.Label(report_frame, text="Measurement Name:").grid(row=1, column=0, sticky="w", pady=5)
        tk.Entry(report_frame, textvariable=self.rep_name, width=40).grid(row=1, column=1, columnspan=2, pady=5, sticky="w")

        tk.Label(report_frame, text="Notes:").grid(row=2, column=0, sticky="w", pady=5)
        tk.Entry(report_frame, textvariable=self.rep_notes, width=40).grid(row=2, column=1, columnspan=2, pady=5, sticky="w")

        tk.Checkbutton(report_frame,text="Show Anchors",variable=self.rep_show_anchors).grid(row=3, column=0, sticky="w", pady=5)

        tk.Checkbutton(report_frame,text="Use Measurement Window CSV",variable=self.rep_use_measurement_window).grid(row=4, column=0, sticky="w", pady=5)

        btn_frame = tk.Frame(report_frame, pady=20)
        btn_frame.grid(row=5, column=0, columnspan=3, sticky="w")

        tk.Button(btn_frame, text="Save Report Settings", command=self.save_settings, width=20).pack(side=tk.LEFT, padx=(0, 10))
        tk.Button(btn_frame, text="Select Folder & Generate", command=self.manual_report_trigger,
                  bg="#0052cc", fg="white", width=25).pack(side=tk.LEFT)

    # Prompts the user with a file dialogue to select a specific session folder for making a measurement report
    # -----------------------------------------------------------------------------------------------------------------
    def manual_report_trigger(self):
        self.save_settings()
        folder = filedialog.askdirectory(title="Select Measurement Session Folder")

        if folder:
            report_config = {
                'name': self.rep_name.get(),
                'notes': self.rep_notes.get(),
                'show_anchors': self.rep_show_anchors.get(),
                'use_measurement_window': self.rep_use_measurement_window.get(),
                'save_pdf': False,
                'save_pngs': False
            }

            config_path = os.path.join(folder, "report_settings.json")
            with open(config_path, 'w') as f:
                json.dump(report_config, f, indent=4)

            print(f"\n[REPORT] Generating report for {folder}...")
            try:
                ComparisonReportMaker.run_dashboard(folder, skip_popup=True)
            except Exception as e:
                print(f"Error generating report: {e}")

    # =================================================================================================================
    # Bluetooth Configuration Tab
    # =================================================================================================================

    # Builds the visual layout for the UWB Anchors tab
    # -----------------------------------------------------------------------------------------------------------------
    def setup_anchor_tab(self):
        container = tk.Frame(self.tab_anchors, padx=10, pady=10)
        container.pack(fill=tk.BOTH, expand=True)

        # === LEFT PANEL: List and File Operations ===
        left_panel = tk.Frame(container, width=200)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 15))

        tk.Label(left_panel, text="Network Configurations", font=("Arial", 12, "bold")).pack(anchor="w", pady=(0, 5))

        # File Control Buttons
        file_btn_frame = tk.Frame(left_panel)
        file_btn_frame.pack(fill=tk.X, pady=(0, 10))
        tk.Button(file_btn_frame, text="Load Config", command=self.load_uwb_config).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        tk.Button(file_btn_frame, text="Save As...", command=self.save_uwb_config_as).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))

        # Listbox
        self.module_listbox = tk.Listbox(left_panel, height=15, exportselection=False)
        self.module_listbox.pack(fill=tk.BOTH, expand=True)
        self.module_listbox.bind("<<ListboxSelect>>", self.on_module_select)

        # List Control Buttons
        list_btn_frame = tk.Frame(left_panel)
        list_btn_frame.pack(fill=tk.X, pady=(5, 0))
        tk.Button(list_btn_frame, text="+ Add Module", command=self.add_module).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        tk.Button(list_btn_frame, text="- Remove", command=self.remove_module).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))

        # === RIGHT PANEL: Module Details ===
        detail_frame = tk.Frame(container)
        detail_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        tk.Label(detail_frame, text="Module Settings", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 15))

        tk.Label(detail_frame, text="Module Name:").grid(row=1, column=0, sticky="e", padx=5, pady=5)
        tk.Entry(detail_frame, textvariable=self.mod_name, width=25).grid(row=1, column=1, sticky="w")

        tk.Label(detail_frame, text="MAC Address:").grid(row=2, column=0, sticky="e", padx=5, pady=5)
        tk.Entry(detail_frame, textvariable=self.mod_address, width=25).grid(row=2, column=1, sticky="w")

        tk.Label(detail_frame, text="Module Type:").grid(row=3, column=0, sticky="e", padx=5, pady=5)
        type_dropdown = ttk.Combobox(detail_frame, textvariable=self.mod_type, values=["Anchor", "Tag", "Listener"], state="readonly", width=22)
        type_dropdown.grid(row=3, column=1, sticky="w")

        tk.Label(detail_frame, text="Location (x, y, z):").grid(row=4, column=0, sticky="e", padx=5, pady=5)
        tk.Entry(detail_frame, textvariable=self.mod_location, width=25).grid(row=4, column=1, sticky="w")

        tk.Label(detail_frame, text="Network ID:").grid(row=5, column=0, sticky="e", padx=5, pady=5)
        tk.Entry(detail_frame, textvariable=self.mod_network_id, width=25).grid(row=5, column=1, sticky="w")

        tk.Label(detail_frame, text="Toggles:").grid(row=6, column=0, sticky="e", padx=5, pady=5)
        toggles_frame = tk.Frame(detail_frame)
        toggles_frame.grid(row=6, column=1, sticky="w")
        tk.Checkbutton(toggles_frame, text="Radio Turned On", variable=self.mod_turned_on).pack(side=tk.LEFT)
        tk.Checkbutton(toggles_frame, text="LEDs Enabled", variable=self.mod_led_enabled).pack(side=tk.LEFT, padx=(10, 0))

        # Action Buttons
        action_frame = tk.Frame(detail_frame, pady=30)
        action_frame.grid(row=7, column=0, columnspan=2, sticky="w")
        tk.Button(action_frame, text="Apply Edit to List", command=self.apply_module_edit, width=15).pack(side=tk.LEFT, padx=(0, 10))
        tk.Button(action_frame, text="Push Configuration via BLE", command=self.push_ble_config,
                  bg="purple", fg="white", width=25).pack(side=tk.LEFT)

        # Load UI
        self.refresh_listbox()

    # Opens a file dialogue allowing the user to select and load an entire network setup from a previously saved JSON
    # file into the listbox
    # -----------------------------------------------------------------------------------------------------------------
    def load_uwb_config(self):
        filepath = filedialog.askopenfilename(initialdir=self.config_dir, title="Select Configuration",
                                              filetypes=[("JSON Files", "*.json")])
        if filepath:
            try:
                with open(filepath, 'r') as f:
                    self.uwb_modules = json.load(f)
                self.refresh_listbox()
                print(f"[GUI] Loaded configuration: {os.path.basename(filepath)}")
            except Exception as e:
                print(f"[GUI] Failed to load configuration: {e}")

    # Opens a file dialogue so the user can export the current list of UWB modules and their configurations to a new
    # JSON file
    # -----------------------------------------------------------------------------------------------------------------
    def save_uwb_config_as(self):
        filepath = filedialog.asksaveasfilename(initialdir=self.config_dir, defaultextension=".json",
                                                filetypes=[("JSON Files", "*.json")])
        if filepath:
            self.save_current_module_edits()
            try:
                with open(filepath, 'w') as f:
                    json.dump(self.uwb_modules, f, indent=4)
                print(f"[GUI] Saved configuration: {os.path.basename(filepath)}")
            except Exception as e:
                print(f"[GUI] Failed to save configuration: {e}")

    # Adds a module to the list
    # -----------------------------------------------------------------------------------------------------------------
    def add_module(self):
        self.save_current_module_edits()
        new_module = {
            "name": f"Module {len(self.uwb_modules) + 1}",
            "address": "00:00:00:00:00:00",
            "type": "Tag",
            "location": "0.0, 0.0, 0.0",
            "network_id": "0x1D32",
            "turned_on": True,
            "led_enabled": True
        }
        self.uwb_modules.append(new_module)
        self.refresh_listbox()
        self.module_listbox.selection_set(tk.END)
        self.load_module_details(len(self.uwb_modules) - 1)

    # Removes a module from the list
    # -----------------------------------------------------------------------------------------------------------------
    def remove_module(self):
        if not self.uwb_modules:
            return

        selections = self.module_listbox.curselection()
        if selections:
            index = selections[0]
            del self.uwb_modules[index]
            self.refresh_listbox()

            if self.uwb_modules:
                new_idx = min(index, len(self.uwb_modules) - 1)
                self.module_listbox.selection_set(new_idx)
                self.load_module_details(new_idx)
            else:
                self.current_module_index = -1

    # Clears the visual list of modules and repopulates it by looping through the internal data list, formatting the
    # strings to show both the module name and its role type
    # -----------------------------------------------------------------------------------------------------------------
    def refresh_listbox(self):
        self.module_listbox.delete(0, tk.END)

        for mod in self.uwb_modules:
            # We put the type in brackets just so it's easier to read the list
            self.module_listbox.insert(tk.END, f"{mod.get('name', 'Unknown')} [{mod.get('type', 'Tag')}]")

        if self.uwb_modules:
            self.module_listbox.selection_set(0)
            self.load_module_details(0)

    # A callback function triggered whenever the user clicks a different module in the listbox
    # -----------------------------------------------------------------------------------------------------------------
    def on_module_select(self, event):
        self.save_current_module_edits()
        selections = self.module_listbox.curselection()
        if selections:
            self.load_module_details(selections[0])

    # Fetches the data dictionary of a specific module based on its index and updates all the Tkinter variables in the
    # right-hand panel to match the stored data.
    # -----------------------------------------------------------------------------------------------------------------
    def load_module_details(self, index):
        if index < 0 or index >= len(self.uwb_modules):
            return

        self.current_module_index = index
        mod = self.uwb_modules[index]
        self.mod_name.set(mod.get("name", f"Module {index + 1}"))
        self.mod_address.set(mod.get("address", ""))
        self.mod_type.set(mod.get("type", "Tag"))
        self.mod_location.set(mod.get("location", "0.0, 0.0, 0.0"))
        self.mod_network_id.set(mod.get("network_id", "0x1D32"))

        # Convert strings to booleans safely just in case
        turned_on = mod.get("turned_on", True)
        self.mod_turned_on.set(str(turned_on).lower() != "false")

        led_en = mod.get("led_enabled", True)
        self.mod_led_enabled.set(str(led_en).lower() != "false")

    # Save
    # -----------------------------------------------------------------------------------------------------------------
    def save_current_module_edits(self):
        if not self.uwb_modules or self.current_module_index < 0:
            return

        mod = self.uwb_modules[self.current_module_index]
        mod["name"] = self.mod_name.get()
        mod["address"] = self.mod_address.get()
        mod["type"] = self.mod_type.get()
        mod["location"] = self.mod_location.get()
        mod["network_id"] = self.mod_network_id.get()
        mod["turned_on"] = self.mod_turned_on.get()
        mod["led_enabled"] = self.mod_led_enabled.get()

    # Apply
    # -----------------------------------------------------------------------------------------------------------------
    def apply_module_edit(self):
        """Saves current edits and visually updates the listbox text"""
        self.save_current_module_edits()
        selections = self.module_listbox.curselection()
        if selections:
            index = selections[0]
            self.module_listbox.delete(index)
            mod = self.uwb_modules[index]
            self.module_listbox.insert(index, f"{mod.get('name')} [{mod.get('type')}]")
            self.module_listbox.selection_set(index)

    # Spawns a background thread that takes the current internal list of UWB modules and passes them to the
    # ReadUWBBluetooth script so that modules can be updated.
    # -----------------------------------------------------------------------------------------------------------------
    def push_ble_config(self):
        self.save_current_module_edits()
        print("\n[GUI] Spawning Bluetooth Configuration Thread...")

        # Run the BLE configuration in a background thread so the GUI doesn't freeze
        ble_thread = threading.Thread(
            target=ReadUWBBluetooth.run_bluetooth_configuration,
            args=(self.uwb_modules,),
            daemon=True
        )
        ble_thread.start()

    # =================================================================================================================
    # Live Data Queue Handling
    # =================================================================================================================

    # Normalizes both the old tuple format and the new dictionary format into one internal packet format.
    # -----------------------------------------------------------------------------------------------------------------
    def normalize_data_packet(self, packet):
        now = time.time()

        # New preferred format
        if isinstance(packet, dict):
            raw_source = str(packet.get("source", "unknown"))
            source = self.normalize_source_name(raw_source)

            position = self.extract_position_from_packet(packet)
            if position is None:
                return None

            timestamp = packet.get("timestamp", packet.get("pc_timestamp", now))

            return {
                "source": source,
                "raw_source": raw_source,
                "data_type": packet.get("data_type", "position"),
                "timestamp": timestamp,
                "position": position,
                "quality": packet.get("quality", {}),
                "metadata": packet.get("metadata", {})
            }

        # Old temporary format: ('UWB', x, y, z) or ('GT', x, y, z)
        if isinstance(packet, (tuple, list)) and len(packet) >= 4:
            try:
                raw_source = str(packet[0])
                source = self.normalize_source_name(raw_source)
                x = float(packet[1])
                y = float(packet[2])
                z = float(packet[3])

                return {
                    "source": source,
                    "raw_source": raw_source,
                    "data_type": "position",
                    "timestamp": now,
                    "position": [x, y, z],
                    "quality": {},
                    "metadata": {"legacy_packet": True}
                }

            except Exception as e:
                print(f"[MASTER] Could not normalize old queue packet: {e}")
                return None

        print(f"[MASTER] Unknown queue packet format: {packet}")
        return None

    # Converts different possible source labels into a smaller group of internal source names.
    # -----------------------------------------------------------------------------------------------------------------
    def normalize_source_name(self, source):
        src = str(source).strip().lower()

        if src in ["uwb", "uwb_raw", "uwb_filtered", "uwb_position", "listener", "listener_serial"]:
            return "uwb"

        if src in ["gt", "ground_truth", "groundtruth", "optitrack", "opti", "natnet"]:
            return "ground_truth"

        if src in ["gps", "gps_rtk", "rtk"]:
            return "gps_rtk"

        return src

    # Extracts [x, y, z] from a queue packet, whether the position is stored as a list or dictionary.
    # -----------------------------------------------------------------------------------------------------------------
    def extract_position_from_packet(self, packet):
        try:
            if "position" in packet:
                pos = packet["position"]

                if isinstance(pos, dict):
                    return [float(pos["x"]), float(pos["y"]), float(pos["z"])]

                if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    return [float(pos[0]), float(pos[1]), float(pos[2])]

            # Also allow flat dictionaries with x/y/z keys
            if all(k in packet for k in ["x", "y", "z"]):
                return [float(packet["x"]), float(packet["y"]), float(packet["z"])]

        except Exception as e:
            print(f"[MASTER] Could not extract position from packet: {e}")

        return None

    # Stores the latest packet in system_state. This can later be used by GUI, UDP routing, and future robot control.
    # -----------------------------------------------------------------------------------------------------------------
    def update_system_state(self, packet):
        self.system_state["latest_packet_time"] = time.time()

        if packet["source"] == "uwb":
            self.system_state["latest_uwb"] = packet
        elif packet["source"] == "ground_truth":
            self.system_state["latest_ground_truth"] = packet
        elif packet["source"] == "gps_rtk":
            self.system_state["latest_gps_rtk"] = packet

    # Sends the normalized packet to the correct live plot buffers. Live measurement UDP is handled by the sensor scripts.
    # -----------------------------------------------------------------------------------------------------------------
    def handle_data_packet(self, packet):
        if packet is None:
            return False

        self.update_system_state(packet)

        x, y, z = packet["position"]
        updated = False

        if packet["source"] == "uwb":
            self.plot_x_uwb.append(x)
            self.plot_y_uwb.append(y)
            self.plot_z_uwb.append(z)
            updated = True

        elif packet["source"] == "ground_truth":
            self.plot_x_gt.append(x)
            self.plot_y_gt.append(y)
            self.plot_z_gt.append(z)
            updated = True

        elif packet["source"] == "gps_rtk":
            # GPS RTK is not live-plotted in Python for now. It is expected to be handled in MATLAB/ROS.
            pass

        return updated

    # Loop that runs every 0.1 seconds. It checks the data queue for incoming X,Y and Z coordinates. It appends
    # the coordinates to the respective plotline object, recalculates the bounds of the 3D axis and redraws the canvas.
    # -----------------------------------------------------------------------------------------------------------------
    def update_live_plot(self):
        if stop_event.is_set():
            return

        updated = False

        try:
            while True:
                raw_packet = self.data_queue.get_nowait()
                packet = self.normalize_data_packet(raw_packet)
                if self.handle_data_packet(packet):
                    updated = True

        except queue.Empty:
            pass

        if updated:
            self.line_uwb.set_data_3d(self.plot_x_uwb, self.plot_y_uwb, self.plot_z_uwb)
            self.line_gt.set_data_3d(self.plot_x_gt, self.plot_y_gt, self.plot_z_gt)

            # Manually calculate 3D boundaries to keep the plot perfectly framed
            all_x = self.plot_x_uwb + self.plot_x_gt
            all_y = self.plot_y_uwb + self.plot_y_gt
            all_z = self.plot_z_uwb + self.plot_z_gt

            if all_x:
                padding = 0.5
                self.ax.set_xlim(min(all_x) - padding, max(all_x) + padding)
                self.ax.set_ylim(min(all_y) - padding, max(all_y) + padding)
                self.ax.set_zlim(min(all_z) - padding, max(all_z) + padding)

            self.canvas.draw()

        self.root.after(100, self.update_live_plot)


# =====================================================================================================================
# End Program
# =====================================================================================================================
if __name__ == "__main__":
    root = tk.Tk()
    app = MasterControlApp(root)

    def on_closing():
        app.save_settings()
        app.stop_logging()
        try:
            app.matlab_socket.close()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
