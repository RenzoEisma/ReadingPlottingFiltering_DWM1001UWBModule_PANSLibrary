DWM1001C Direct BLE Reader README
This project allows you to read UWB position data directly from a Qorvo DWM1001C tag using Python. By using Bluetooth Low Energy, this method bypasses the need for a dedicated listener node and the DRTLS application.

Prerequisites
Python 3.8 or newer installed on your system.

The Bleak library for Bluetooth communication. Install it by running pip install bleak in your terminal.

A DWM1001C module powered on and already configured as an active tag.

The MAC address of your specific tag updated in the DEVICE_ADDRESS variable within the script.

How the Program Works
The script operates by acting as a Bluetooth Central device that connects to the DWM1001C, which acts as the Peripheral. It functions through four primary steps.

Device Connection
The script uses the BleakClient from the Bleak library to target the specific MAC address of your tag. It establishes a direct wireless connection to the module.

Subscribing to Notifications
Instead of constantly polling the tag for its location, the script subscribes to a specific data channel known as a GATT characteristic. The UUID 003bbdf2-c634-4b3d-ab56-7ec889b89a37 is the dedicated channel for location data on the DWM1001C. By using the start_notify function, we instruct the tag to automatically push new data to our Python script the moment a new position is calculated.

Unpacking the Data
When the tag pushes a notification, it triggers the location_notification_handler function. The module does not send readable text. It sends a raw package consisting of exactly 13 bytes.

Bytes 0 to 3 contain the X coordinate.

Bytes 4 to 7 contain the Y coordinate.

Bytes 8 to 11 contain the Z coordinate.

Byte 12 contains the location quality factor.
The script uses the built-in Python struct library to decode these raw bytes into standard numbers. It uses the format string "<iiiB" to specify that the incoming data is Little Endian format, extracting three 32-bit signed integers for the coordinates in millimeters, followed by one 8-bit unsigned integer for the quality percentage. The script then divides the millimeters by 1000 to display the final output in meters.

The Async Loop
Because Bluetooth network operations take unpredictable amounts of time, the script relies on Python's asyncio library. The while True loop inside the stream_tag_coordinates function keeps the program awake and listening for incoming Bluetooth notifications indefinitely until you manually terminate it by pressing Ctrl+C.

Future Expansions
The code includes clearly marked, empty comment blocks designed for future features. These are placeholders where we can later add functionality to store configurations, write commands to the Node Config characteristic to swap modules between Tag and Anchor modes, and send custom system reset commands.