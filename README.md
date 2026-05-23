=======================================================================================================================
  _____   _____   ____  _   _ ______    _____ ______ _   _  _____  ____  _____
 |  __ \ |  __ \ / __ \| \ | |  ____|  / ____|  ____| \ | |/ ____|/ __ \|  __ \
 | |  | || |__) | |  | |  \| | |__    | (___ | |__  |  \| | (___ | |  | | |__) |
 | |  | ||  _  /| |  | | . ` |  __|    \___ \|  __| | . ` |\___ \| |  | |  _  /
 | |__| || | \ \| |__| | |\  | |____   ____) | |____| |\  |____) | |__| | | \ \
 |_____/ |_|  \_\\____/|_| \_|______| |_____/|______|_| \_|_____/ \____/|_|  \_\

                ______ _    _  _____ _____ ____  _   _
               |  ____| |  | |/ ____|_   _/ __ \| \ | |
               | |__  | |  | | (___   | || |  | |  \| |
               |  __| | |  | |\___ \  | || |  | | . ` |
               | |    | |__| |____) |_| || |__| | |\  |
               |_|     \____/|_____/|_____\____/|_| \_|
=======================================================================================================================

- Author:               Renzo Eisma
- Date last Rev.:       14/04/2026
- Version:              5
- Project:              UWB vs Ground Truth Comparison
- Lab:                  LabAir

unified software framework

-----------------------------------------------------------------------------------------------------------------------
I. PROJECT OVERVIEW
-----------------------------------------------------------------------------------------------------------------------

This project is a comprehensive logging and analysis suite for drone positioning
systems[cite: 1]. It simultaneously records data from multiple sensors like
UWB, OptiTrack ground truth, and eventually GPS[cite: 1]. It provides tools to
filter, compare, and visualize the accuracy of the positioning hardware
[cite: 1]. The system is designed around a central logger that orchestrates
individual sensor drivers[cite: 2, 3].

A MATLAB script is also planned for the drivers folder to handle data filtering
and routing to ROS.

-----------------------------------------------------------------------------------------------------------------------
II. TABLE OF CONTENTS
-----------------------------------------------------------------------------------------------------------------------

1. SYSTEM ARCHITECTURE ........................................ [SECTION 1]
2. READING SENSORS SETUP GUIDE ................................ [SECTION 2]
    a. Client Computer Setup (Python)
    b. OptiTrack Setup
    c. UWB Setup
    d. GPS Setup
3. USER GUIDES ................................................ [SECTION 3]
    a. Using the Master Logger
    b. Using the Plotting Script (Report Maker)
    c. Using the Bluetooth Assigner (DWM1001C)
4. DATA OUTPUT SCHEMA ......................................... [SECTION 4]
5. TROUBLESHOOTING / COMMON ERRORS ............................ [SECTION 5]
6. TO-DO LISTS BY MODULE ...................................... [SECTION 6]
    a. MasterLogger
    b. ComparisonReportMaker
    c. ReadUWBBluetooth
    d. UWB_Sensor Driver
7. VERSION HISTORY ............................................ [SECTION 7]


-----------------------------------------------------------------------------------------------------------------------
1. SYSTEM ARCHITECTURE
-----------------------------------------------------------------------------------------------------------------------

[1.1] Broad system overview
The system uses a multi-threaded approach where the MasterLogger script acts as
the primary orchestrator[cite: 2, 3]. It initializes separate threads for each
enabled sensor (UWB, OptiTrack, GPS) to ensure high-frequency data collection
without blocking the main process[cite: 3].

[1.2] Code Blockdiagram
MasterLogger -> Thread 1: OptiTrack Driver (UDP)
             -> Thread 2: UWB Driver (UART/Serial)
             -> Thread 3: GPS Driver (TBD) [cite: 3]
             -> Post-Process: ComparisonReportMaker (Data Fusion)


-----------------------------------------------------------------------------------------------------------------------
2. READING SENSORS SETUP GUIDE
-----------------------------------------------------------------------------------------------------------------------

[2.1] CLIENT COMPUTER SETUP (PYTHON)
1. Required Programs:
    - Matlab 2025b
        - Can be acquired for free with a 30 day free trial Matlab license
          License can be extended for free every month
        - Needs the ROS and UDP toolboxes
    - Python installation between 3.x and 3.12 (3.13 and above won't work with matlabengine)
    - Python environment (Pycharm or Visual Studio Code are ideal)
2. Install all necessary python libraries
    - Can be found in the requirements.txt file
3. When launching MasterControlStation.py configure all settings according to your computer
    - IP address
    - COM Ports
4. Connect UWB listener to a USB port
    - Open Windows Device Manager to find COM port
5. Connect Controller to a USB port (optional)
    - For backup control of robots

[2.2] Linux PC & Network SETUP
1. Required Programs
    - ROS 1
2. Connect Linux PC to Client PC with ethernet
3. Network...

[2.3] GROUND TRUTH SETUP
[2.3.1] Optitrack
1. Open Motive on the server PC and ensure your drone's Rigid Body is
   tracking
2. Go to View -> Data Streaming Pane
3. Check the box for "Broadcast Frame Data"
4. Set "Local Interface" to the Motive PC's specific IP address. Do not use
   Loopback
5. Set "Transmission Type" to Unicast
6. Check the box for "Rigid Bodies"
* Note: Ensure Windows Firewall on the Motive PC allows UDP traffic on
  ports 1510 and 1511

[2.3.2] GPS SETUP
[Placeholder: Instructions for RTK GPS base/rover configuration, NMEA
formatting, and COM port setup will go here once integrated.]

[2.4] Robot Setup
[2.4.1] Limu Setup
1. ...
    - ...

[2.4.2] Bebop2 Setup
1. ...
    - ...

-----------------------------------------------------------------------------------------------------------------------
3. USER GUIDES
-----------------------------------------------------------------------------------------------------------------------

[3.1] USING THE MASTER LOGGER
The central orchestrator for data collection[cite: 2].

1. Ensure Python 3.x is installed along with: pyserial, pandas, numpy,
   matplotlib, plotly, scipy, and bleak[cite: 5, 21].
2. Open MasterLogger.py and update configuration for COM ports and IP
   addresses.
3. For OptiTrack setup, open Motive -> Data Streaming -> Broadcast Frame Data ->
   Local IP -> Unicast -> Rigid Bodies[cite: 9, 10, 11].
4. Turn on all sensors and run: python MasterLogger.py[cite: 16].
5. Fly the drone or move the sensors as needed[cite: 16].
6. Press Ctrl+C to stop logging, close ports, and trigger report
   generation[cite: 17].

[3.2] USING THE PLOTTING SCRIPT
The report maker can be run independently to analyze previous sessions.

1. Run: python drivers/ComparisonReportMaker.py.
2. A configuration window appears for name, notes, and preferences.
3. Click "Start Analysis" and select the session folder in /measurements.
4. The script calculates offsets, generates a PDF, and an interactive HTML
   dashboard.

[3.3] USING THE BLUETOOTH ASSIGNER
Direct configuration of DWM1001C modules via BLE[cite: 18, 19].

1. Update dictionaries in ReadUWBBluetooth.py with MAC addresses[cite: 23].
2. Run: python drivers/ReadUWBBluetooth.py.
3. The script assigns roles (Tag/Anchor/Listener), pushes positions, and
   optionally streams location data[cite: 25, 26, 28].

[3.4] USING ROS
In ROS heb je configurations voor robots. De bebop heeft zijn configuration op de linux pc, de limu heeft het intern.
Die configuration is bebop_lau_1 bijvoorbeeld. Dit heeft allemaal dingen geconfigureerd. Als je dingen wilt aanpassen
kan je nog de configurations pakken zoals ip en het veranderen.
- List topics for seeing all available topics
		○ Belangrijk: land, takeoff, cmd_vel

Limu Instructions:
	- First do 'roscore' in terminal (initiaization)

	- Turn on Limu
	- Open roscore terminal
	- Open another roscore terminal
	- sudo ssh agilex@192.168.0.103 (username always the same but ip can change, ip is on the frame of the limu. (103))
	- Password for pc: admin
	- Password Limu: agx
	- Password Linux PC: admin
	- Launch ros node: roslaunch limo_base limo_base.launch namespace:=L1
		○ Press tab twice to see options
		○ Namespace:=L1 represents name of robot in matlab code
	- Red lights on limu = error mode, fix by pressing power button once
Create rigidbody:
	- When creating rigid body, have it facing in x axis
	- Have balls center be robot center
In ros:
	- Rostopic list
	- Rostopic pub /L1/cmd_vel geometry_msgs -> then press tab tab
    - Je kan ook rostopic + tab tab doen om alle opties te zien


Bebop Instructions:
	- Turn on
		○ Press once, wait
		○ Press three times
	- Ping ip: 192.168.0.21x
	- Roslaunch bebop_driver bebop_lai_1.launch ip:=… namespace:=…
	- Driver for bebop is inside computer and limu is inside limo
	- Rostopic pub /B7/land std_msgs/Empty "{}"
    - Rostopic pub /B7/takeoff std_msgs/Empty "{}"


-----------------------------------------------------------------------------------------------------------------------
4. DATA OUTPUT SCHEMA
-----------------------------------------------------------------------------------------------------------------------

All logs follow a standardized 4-column CSV format for internal compatibility
between drivers and the report generator:

Column 1: Time          (Unix PC Timestamp in seconds)
Column 2: POSX          (X coordinate in meters)
Column 3: POSY          (Y coordinate in meters)
Column 4: POSZ          (Z coordinate in meters)

Example Log Header:
Time, POSX, POSY, POSZ
1740673321.451, 1.2345, -0.5678, 2.1012


-----------------------------------------------------------------------------------------------------------------------
5. TROUBLESHOOTING / COMMON ERRORS
-----------------------------------------------------------------------------------------------------------------------

- ERROR: "Permission Denied" during folder rename
  Cause: A CSV log from the session is open in Excel or another program.
  Fix: Close all CSV files before ending the MasterLogger script.

- ERROR: OptiTrack data not appearing in logs
  Cause: Windows Firewall blocking UDP ports 1510/1511 or Motive set to Loopback.
  Fix: Disable firewall temporarily or add an exception; set Motive to a specific
  Local IP[cite: 10, 12].

- ERROR: UWB Jumps/Outliers in Raw Data
  Cause: Non-Line-of-Sight (NLOS) interference or radio collisions.
  Fix: Check "uwbFiltered_log" which uses the UWBSmoother Kalman Filter for
  outlier rejection.

- ERROR: Bluetooth device connection fails
  Cause: Device already connected to another host or MAC address mismatch.
  Fix: Reset DWM1001C power and verify the MAC address in the script[cite: 23, 35].



-------------------------------------------------------------------------------
6. Explanations
-------------------------------------------------------------------------------

6.1. MasterControlStation Explanation
-------------------------------------------------------------------------------
This script serves as the central command and control station for a drone localization system. It provides a graphical
interface to manage high-precision Ultra-Wideband (UWB) sensors alongside ground truth systems like OptiTrack. Its
primary purpose is to orchestrate data collection, visualize drone movement in real time, and configure the hardware
network without requiring manual command-line interaction.


KEY FEATURES
---
1. Live Logging and Visualization
This section handles the simultaneous recording of data from UWB sensors and ground truth systems. It features a
real-time 3D plot that compares the estimated position from the UWB modules against the actual position provided by the
ground truth system.
2. Network Configuration
Users can manage a list of UWB modules including anchors, tags, and listeners. The script allows for editing MAC
addresses, network IDs, and physical coordinates. These configurations can be saved to or loaded from JSON files and
pushed to the physical hardware via Bluetooth Low Energy (BLE).
3. Automated Reporting
The script includes a utility to generate comprehensive measurement reports. It can process a recorded session to
create PDF dashboards and individual plots that analyze the accuracy and performance of the localization setup.
4. Data Routing
Beyond local logging, the script can stream data to external environments like MATLAB or ROS (Robot Operating System)
for further filtering or advanced control algorithms.


HOW IT WORKS
---
The application architecture is built on a multithreaded model to ensure the user interface remains responsive while
handling high-frequency data streams.

When the logging process starts, the script spawns dedicated background threads for each hardware component. For
example, it starts one thread for the UWB serial connection and another for the OptiTrack network client. As these
threads receive new coordinate data, they place it into a thread-safe queue.

The main GUI thread runs a continuous loop every 100 milliseconds to check this queue. If new data is present, the
script updates the internal coordinate arrays and refreshes the Matplotlib 3D canvas. This separation of concerns
prevents the heavy computational load of 3D plotting and data processing from freezing the buttons and menus. Settings
and hardware configurations are persisted using JSON files, allowing the system to restore its previous state upon
restart.

LIBRARIES AND DEPENDENCIES
---
- Tkinter and Ttk
Used to build the entire graphical user interface, including the window, tabs, buttons, and text consoles.
- Matplotlib
Specifically using the toolkit for 3D projections, this library handles the live trajectory plotting and the generation
of visual charts for reports.
- Threading and Queue
These manage the concurrent execution of sensor drivers and the safe transfer of data between the background workers
and the GUI.
- JSON
Used for parsing and writing configuration files for the hardware network and application settings.
- Matlab Engine
An optional library that allows the script to start a MATLAB session and call custom scripts directly from Python.
- Custom Drivers
The script integrates several project-specific modules including NatNetClient for ground truth data, uwb_sensor for
serial communication, ReadUWBBluetooth for wireless configuration, and ComparisonReportMaker for post-processing.



6.2. ComparisonReportMaker Explanation
-----------------------------------------------------------------------------------------------------------------------
...


6.3. ReadUWBBluetooth Explanation
-----------------------------------------------------------------------------------------------------------------------
...



6.4. NatNetClient Explanation
-----------------------------------------------------------------------------------------------------------------------
...



6.5. uwb_sensor Explanation
-----------------------------------------------------------------------------------------------------------------------
...


6.6. matlab Scripts Explanations
-----------------------------------------------------------------------------------------------------------------------

- Master script
- Filter script
- Bebop control
- Crazyflie control
- Limo control

- How to add a new robot










-----------------------------------------------------------------------------------------------------------------------
7. TO-DO LISTS
-----------------------------------------------------------------------------------------------------------------------

[6.1] MASTERLOGGER
- Maybe use CPP for certain scripts that need to be faster

[6.2] COMPARISONREPORTMAKER
- Think of what is important to measure and implement it
- Offsets in general
- Settings in masterlogger are not connected to reportmaker
automatische file selectie methode er uit halen (wordt vgm niet eens gebruikt)
- 95% error
- median error ipv mean
- Begin en einde van meting af trimmen
- Automatically calculate Angle offset

[6.3] READUWBBLUETOOTH
- Snappen hoe die networks werken en modules kunnen wisselen van netwerk
- networks van modules kunnen aanpassen in dit programma


Instructies toevoegen van hoe je uwb_sensor kan vervangen met iets anders


Offset is now calculated in the following way
- Python: offset of uwb antenna from opti center
- Matlab: offset of robot center from opti center
    - opti center in matlab is same as uwb center as it is already converted


-----------------------------------------------------------------------------------------------------------------------
8. VERSION CHANGES
-----------------------------------------------------------------------------------------------------------------------

- VERSION 1.0: Initial release; sensor reading only.
- VERSION 2.0: Integrated ComparisonReportMaker functionality.
- VERSION 3.0: Added Matlab integration and direct Bluetooth reading.
- Version 3.1:
    - Make a front end for the code
    - Add a live plotter
    - Eventually make the code open source (Add a GitHub for it)
    - Have the name that you input be saved and used again (plotter script)

=======================================================================================================================
END OF DOCUMENT
=======================================================================================================================
