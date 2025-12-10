import pyrealsense2 as rs

try:
    # Create a context object. This object owns the handles to all connected realsense devices
    ctx = rs.context()
    devices = ctx.query_devices()

    if len(devices) == 0:
        print("No RealSense devices found!")
    else:
        print(f"Found {len(devices)} RealSense device(s):")
        print("-" * 30)

        for dev in devices:
            # Get the name of the device (e.g. Intel RealSense D435i)
            name = dev.get_info(rs.camera_info.name)
            # Get the serial number
            serial = dev.get_info(rs.camera_info.serial_number)
            # Get firmware version
            fw = dev.get_info(rs.camera_info.firmware_version)
            
            print(f"Model:    {name}")
            print(f"Serial:   {serial}")
            print(f"Firmware: {fw}")
            print("-" * 30)

except Exception as e:
    print(f"Error: {e}")
    print("\nTroubleshooting Tip: If you see 'AttributeError', check your installation path again.")