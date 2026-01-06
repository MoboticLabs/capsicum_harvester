#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import time
import math

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # Serial Configuration
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('steps_per_rev', 400)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.steps_per_rev = self.get_parameter('steps_per_rev').value
        self.is_ready = False  # Flag to check if Arduino is ready
        
        # Open Serial Connection
        try:
            self.get_logger().info(f'Connecting to {serial_port} at {baud_rate} baud...')
            self.ser = serial.Serial()
            self.ser.port = serial_port
            self.ser.baudrate = baud_rate
            self.ser.timeout = 1
            self.ser.dtr = False  # Prevent reset on connection
            self.ser.rts = False
            self.ser.open()
            
            # Wait for Arduino Reset & Homing
            self.get_logger().info('Waiting for Arduino Homing to complete...')
            self.wait_for_homing()
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None
        
        # Subscribe to joint commands from IK solver
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/arm_controller/commands',
            self.joint_command_callback,
            10
        )
        
        self.get_logger().info('Serial Bridge Node Started - Ready to send commands to Arduino')

    def wait_for_homing(self):
        """Blocks until 'Homing Complete' is received from Arduino"""
        if not self.ser: return
        
        start_time = time.time()
        while (time.time() - start_time) < 60: # 60 second timeout
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f'Arduino: {line}')
                if "Homing Complete" in line:
                    self.get_logger().info("✅ Arduino is Ready!")
                    self.is_ready = True
                    return
            time.sleep(0.1)
        self.get_logger().warn("⚠️ Timed out waiting for homing. Proceeding anyway...")
        self.is_ready = True

    def joint_command_callback(self, msg):
        """
        Receives joint angles in radians from IK solver,
        converts to steps, and sends to Arduino
        """
        if not self.is_ready:
            self.get_logger().warn('Ignoring command: Arduino is still homing...')
            return

        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not available. Skipping command.')
            return
        
        if len(msg.data) < 5:
            self.get_logger().warn(f'Expected 5 joint values, got {len(msg.data)}')
            return
        
        # Convert radians to steps
        # steps = (angle_radians / (2*pi)) * steps_per_rev
        steps = []
        for i, angle_rad in enumerate(msg.data[:5]):
            revolutions = angle_rad / (2 * math.pi)
            step_count = int(revolutions * self.steps_per_rev)
            steps.append(step_count)
        
        # Format command: "step1,step2,step3,step4,step5\n"
        command = f"{steps[0]},{-5.7*steps[1]},{15*steps[2]},{steps[3]},{3*steps[4]}\n"
        
        try:
            self.get_logger().info(f'Sending to Arduino: {command.strip()}')
            self.ser.write(command.encode('utf-8'))
            
            # Optional: Read response from Arduino
            time.sleep(0.1)
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f'Arduino response: {response}')
                
        except Exception as e:
            self.get_logger().error(f'Error sending command: {e}')

    def __del__(self):
        """Clean up serial connection"""
        if hasattr(self, 'ser') and self.ser is not None and self.ser.is_open:
            self.get_logger().info('Closing serial connection...')
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
