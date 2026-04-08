# ===================== PROGRAM_INFO ==================================================================================
""" Author: Renzo Eisma
    Date: 04/2026
    Description: UWB DWM1001 Bluetooth worker script. Takes a configuration list from the GUI
    and pushes settings to all devices concurrently."""

# =====================================================================================================================

import asyncio
import struct
from bleak import BleakClient

# =====================================================================================================================
# BLUETOOTH UUIDS
# =====================================================================================================================
NETWORK_ID_UUID = "80f9d8bc-3bff-45bb-a181-2d6a37991208"
ANCHOR_POS_UUID = "f0f26c9b-2c8c-49ac-ab60-fe03def1b40c"
OPERATION_MODE_UUID = "3f0afd88-7770-46b0-b5e7-9fc099598964"


# =====================================================================================================================
# HELPER FUNCTIONS
# =====================================================================================================================
def parse_location(loc_string):

    """Converts a string like '2.0, 4.5, 6.7' (meters) into 13 packed bytes.
    This is required by the DWM1001 hardware. ..."""
    coords = [float(c.strip()) for c in loc_string.split(',')]
    # Pad to 3 coordinates if the user messed up the string
    while len(coords) < 3:
        coords.append(0.0)

    x_mm, y_mm, z_mm = int(coords[0] * 1000), int(coords[1] * 1000), int(coords[2] * 1000)
    # '<iiiB' packs three 32-bit integers and one 8-bit unsigned integer (100 = Quality Factor)
    return struct.pack('<iiiB', x_mm, y_mm, z_mm, 100)


# =====================================================================================================================
# ASYNC BLUETOOTH ROUTINES
# =====================================================================================================================
async def configure_device(client, device):
    dev_name = device.get('name', 'Unknown')
    dev_type = device.get('type', 'Tag')
    print(f"  -> Configuring {dev_name} as {dev_type}...")

    # 1. Set Anchor Position
    if dev_type == "Anchor":
        loc_bytes = parse_location(device.get('location', "0.0, 0.0, 0.0"))
        await client.write_gatt_char(ANCHOR_POS_UUID, loc_bytes)

    # 2. Safely update Operation Mode (READ -> MODIFY -> WRITE)
    current_mode_bytes = await client.read_gatt_char(OPERATION_MODE_UUID)
    current_mode_int = struct.unpack('<H', current_mode_bytes)[0]
    new_mode_int = current_mode_int

    # --- BIT CLEARING (Resetting target bits to 0 before we configure them) ---
    new_mode_int &= ~0x0080  # Clear Role (Bit 7 of Byte 0)
    new_mode_int &= ~0x0060  # Clear UWB Mode (Bits 6 & 5 of Byte 0)
    new_mode_int &= ~0x0004  # Clear LED (Bit 2 of Byte 0)
    new_mode_int &= ~0x2000  # Clear Location Engine (Bit 5 of Byte 1)
    new_mode_int &= ~0x4000  # Clear Low Power Mode (Bit 6 of Byte 1)

    # --- ROLE & LOCATION ENGINE SETUP ---
    if dev_type == "Anchor":
        new_mode_int |= 0x0080  # Set Anchor bit ON
    elif dev_type == "Listener":
        new_mode_int |= 0x0080  # Listeners are Anchors sniffing the network
    else:
        # Tag
        new_mode_int |= 0x2000  # Turn Location Engine ON so the Tag calculates position

    # --- UWB RADIO SETUP ---
    is_on = device.get('turned_on', True)
    if str(is_on).lower() == "false":
        print(f"  -> {dev_name}: UWB Radio commanded to OFF.")
        # We leave the UWB bits at 00 (Off)
    else:
        if dev_type == "Listener":
            new_mode_int |= 0x0020  # Set UWB to Passive (Binary 01)
        else:
            new_mode_int |= 0x0040  # Set UWB to Active (Binary 10)

    # --- LED SETUP ---
    led_enable = device.get('led_enabled', True)
    if str(led_enable).lower() != "false":
        new_mode_int |= 0x0004  # Turn LED ON

    # 3. Write the safely modified bytes back to the module
    await client.write_gatt_char(OPERATION_MODE_UUID, struct.pack('<H', new_mode_int))
    print(f"  -> {dev_name} configuration applied successfully.")


async def connect_and_manage(device):
    """Handles connection and configuration for a single device."""
    address = device.get('address')
    if not address or address == "00:00:00:00:00:00":
        return  # Skip placeholder modules

    print(f"\nAttempting connection to {address} ({device.get('name')})...")

    try:
        async with BleakClient(address, timeout=10.0) as client:
            if not client.is_connected:
                print(f"❌ Failed to connect to {address}")
                return

            print(f"✅ Connected to {address}")
            await configure_device(client, device)

    except Exception as e:
        print(f"⚠️ Error with device {address}: {e}")


async def main_async(device_list):
    """Creates concurrent tasks for all devices in the list."""
    print(f"\n=== PUSHING BLUETOOTH CONFIGURATION TO {len(device_list)} DEVICES ===")
    tasks = [connect_and_manage(device) for device in device_list]
    await asyncio.gather(*tasks)
    print("\n=== BLUETOOTH CONFIGURATION COMPLETE ===")


def run_bluetooth_configuration(device_list):
    """
    Entry point for the Master Control Station.
    Creates a new event loop so it can run safely in a background thread.
    """
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(main_async(device_list))
    finally:
        loop.close()


if __name__ == "__main__":
    print("This script is now a worker module. Please run it via MasterControlStation.py")