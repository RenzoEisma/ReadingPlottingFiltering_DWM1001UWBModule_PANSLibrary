=======================================================================================================================
UNIFIED SOFTWARE FRAMEWORK
=======================================================================================================================

- Author:               Renzo Eisma
- Date last Rev.:       26/05/2026
- Lab:                  Air Lab - UFES - Espirito Santo

-----------------------------------------------------------------------------------------------------------------------
I. PROJECT OVERVIEW
-----------------------------------------------------------------------------------------------------------------------

NOTE: THE README IS NOT DONE YET, FAR FROM IT. THIS will be completed after finishing all the tests.

short description

mention the graduation report document that i have. Could provide more information

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
the primary orchestrator. It initializes separate threads for each
enabled sensor (UWB, OptiTrack, GPS) to ensure high-frequency data collection
without blocking the main process.

[1.2] Code Blockdiagram
MasterLogger -> Thread 1: OptiTrack Driver (UDP)
             -> Thread 2: UWB Driver (UART/Serial)
             -> Thread 3: GPS Driver (TBD)
             -> Post-Process: ComparisonReportMaker (Data Fusion)

[1.3] .
System overview. Mention block diagram, i have it in the folder


-----------------------------------------------------------------------------------------------------------------------
2. SETUP GUIDE
-----------------------------------------------------------------------------------------------------------------------

[2.1] WINDOWS COMPUTER SETUP
1. Required Programs:
    - Matlab 2025b
        - Can be acquired for free with a 30 day free trial Matlab license
          License can be extended for free every month
        - Needs the ROS and UDP toolboxes
    - Python installation between 3.x and 3.12 (3.13 and above won't work with matlabengine) [maybe leave out last part about matlabengine]
    - Python environment (Pycharm or Visual Studio Code are ideal)
2. Have this Unified Software Framework installed on PC
2. Install all necessary python libraries
    - Can be found in the requirements.txt file
3. When launching MasterControlStation.py configure all settings according to your computer
    - IP address
    - COM Ports
    - ...
5. Connect Controller to a USB port (optional)
    - For backup control of robots

[2.2] Linux PC setup
1. Required Programs
    - ROS 1 [insert ROS version here]
    - ubuntu pc
2. Connect Linux PC to Client PC with ethernet

[2.3] UWB sensors setup
1. have an anchor system ready according to ideal anchor geometry
2. have a tag
3.1 If connecting tag via normal way (listeners)
- Put one listener per network into a USB port on the windows computer
3.2 If connecting tag via Wi-Fi
- ...

[2.4] GROUND TRUTH SETUP
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

[2.3.2] GPS RTK SETUP
[Placeholder: Instructions for RTK GPS base/rover configuration, NMEA
formatting, and COM port setup will go here once integrated.]

[2.4] Robot Setup
[2.4.1] Limu Setup
1. ...
    - ...

[2.4.2] Bebop2 Setup
1. ...
    - ...

[2.4.3] Crazyflie Setup
1. ...
    - ...


[2.5] Connecting all components together
1. Get the Wifi Router



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

- more ...



-------------------------------------------------------------------------------
6. Explanations Python scripts
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







-------------------------------------------------------------------------------
7. matlab Scripts Explanations
-------------------------------------------------------------------------------

- Master script
- Filter script
- Bebop control
- Crazyflie control
- Limo control

- How to add a new robot very short guide



-----------------------------------------------------------------------------------------------------------------------
8. TO-DO LISTS
-----------------------------------------------------------------------------------------------------------------------

General code cleanup and bugs
- Fix the issue where UWB reading consistently goes wrong every other time.
- Add support for two listeners again, since this is currently no longer in the code.
- Make GUI updating easier if possible.
- Add a way to start the program through MasterControl.
  - Make Matlab launch only when UWB is enabled, not when only OptiTrack is used.
- Add a way to run the Python program only for logging.
- Add a way to choose which robot is being used in Python.
- Instead of sending all settings in every UDP packet, send the settings once at the beginning.
- Add compensation for data delay.

GUI improvements
- Make the GUI full screen.
- Add more color to the GUI, More blue and white instead of gray
- Show more live data in the GUI on the right side bar
- Add GUI options for:
  - using UWB or OptiTrack as position for a moving robot;
  - enabling ROS;
  - enabling robot movement.
- Note: enabling ROS and enabling robot movement are currently partly combined.
- Add an explanation in readme for why matplotlib

UWB data reading
- Try to read individual distances from the anchors. Use the dist command. Sending the command dist returns a list of the anchors the tag is currently ranging with, along with the distance to each one in millimeters.
- Read the accuracy percentage via UART.

Data structure and communication
- There is currently a problem where all data is stored in uwb_sensor, so when UWB is turned off, ROS does not receive information about whether or not to control the robot.
- A UDP port should be made in the MasterControl script where this data can be sent.
- Add an explanation in readme for the use of threads with live visualization.

Filtering and sensor fusion
- Finish code for fusing two tags.
- Prepare code for accelerometer fusion. Use accelerometer data from robots via ROS.
- Add a filter for UWB with:
  - outlier rejection;
  - Kalman filter;
- UWB does not provide angle, but angle is needed.
  - The angle can be taken from the robot topic.
  - Or the angle can be calculated from the known position and the change in position over time.

Robot control
- Prepare code for Bebop Matlab control.
  - Get accelerometer data.
  - Get angle data.
  - General code.
- Prepare code for LIMO Matlab control.
  - Get accelerometer data.
  - Get angle data.
  - General code.
- Program Figure 8 movement for the drone.
- Get the drone control code from Miguel.
- Add a trespass area for the drone.
- Define what should happen if the connection with the drone is lost.

GPS RTK
- Add space/support for GPS RTK reading from ROS with Enzo

ROS integration
- Send filtered UWB data to ROS.
- Make sure ROS still receives control-related information even when UWB is turned off.



-------------------------------------

[6.1] MASTERLOGGER
- Maybe use CPP for certain scripts that need to be faster

[6.2] COMPARISONREPORTMAKER
- Settings in masterlogger are not connected to reportmaker
automatische file selectie methode er uit halen (wordt vgm niet eens gebruikt)
- 95% error
- median error ipv mean
- Begin en einde van meting af trimmen
- Automatically calculate Angle offset

[6.3] READUWBBLUETOOTH
- Snappen hoe die networks werken en modules kunnen wisselen van netwerk

Note: Offset is now calculated in the following way
- Python: offset of uwb antenna from opti center
- Matlab: offset of robot center from opti center
    - opti center in matlab is same as uwb center as it is already converted




-----------------------------------------------------------------------------------------------------------------------
9. VERSION CHANGES
-----------------------------------------------------------------------------------------------------------------------

- Look to the github for all different versions




=======================================================================================================================
END OF DOCUMENT
=======================================================================================================================