import serial
import time

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'  # Check if this matches your Arduino!
BAUD_RATE = 115200

# Motor Settings
STEPS_PER_REV = 1600
TARGET_REVOLUTIONS = 2

# WHICH MOTOR TO TEST? (1 to 5)
TEST_JOINT_ID = 1

def main():
    try:
        # 1. Open Serial Port
        print(f"Connecting to {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        
        print("Waiting 3 seconds for Arduino to reset...")
        time.sleep(3)
        
        # 2. Calculate Steps
        target_steps = STEPS_PER_REV * TARGET_REVOLUTIONS
        print(f"Target: {TARGET_REVOLUTIONS} Revolutions = {target_steps} steps")

        # 3. Format the Command (The Decision Logic)
        # We create a list of 5 zeros: [0, 0, 0, 0, 0]
        targets = [0, 0, 0, 0, 0]
        
        # We set the specific index for the motor we want to move
        # Python lists are 0-indexed, so Motor 1 is index 0
        motor_index = TEST_JOINT_ID - 1
        if 0 <= motor_index < 5:
            targets[motor_index] = target_steps
        else:
            print("Error: Invalid Joint ID")
            return

        # Create the string: "val1,val2,val3,val4,val5\n"
        command = f"{targets[0]},{targets[1]},{targets[2]},{targets[3]},{targets[4]}\n"
        
        # 4. Send Command
        print(f"Testing Motor {TEST_JOINT_ID}...")
        print(f"Sending command: {command.strip()}")
        ser.write(command.encode('utf-8'))
        
        print("Done! The Arduino should be moving the motor now.")
        print("(Note: The Arduino code blocks until movement is finished)")

        ser.close()
        
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == '__main__':
    main()