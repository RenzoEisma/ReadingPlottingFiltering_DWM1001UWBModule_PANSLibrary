import serial
import time

port = 'COM3'  # <-- VERIFY THIS IS CORRECT
baud = 115200

print(f"--- UWB RAW HARDWARE TEST ---")
print(f"Connecting to {port}...")

try:
    # Open port with hardware flow control disabled
    ser = serial.Serial(port, baud, timeout=1, dsrdtr=False, rtscts=False)
    ser.dtr = False
    ser.rts = False

    print("Port opened successfully! Sending \\r\\r to wake up...")
    ser.write(b'\r\r')
    time.sleep(1)

    # Read everything sitting in the buffer
    response = ser.read_all().decode('utf-8', errors='ignore').strip()
    print(f"Wake-up Response: '{response}'")

    if not response:
        print("\nCRITICAL FAILURE: The module did not respond at all.")
        print("Causes: Wrong COM port, charge-only USB cable, or module needs a hard reset.")
    else:
        print("\nModule responded! Sending 'lec'...")
        ser.write(b'lec\r')
        time.sleep(0.5)

        print("Listening for 3 seconds...")
        for _ in range(30):
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    print(f"DATA: {data}")
            time.sleep(0.1)

    ser.close()
    print("\nTest complete.")

except Exception as e:
    print(f"Python crashed trying to open the port: {e}")