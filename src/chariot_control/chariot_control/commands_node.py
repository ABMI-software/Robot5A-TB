import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import os
import threading

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # Serial port configuration
        self.PORT = "/dev/ttyACM0"  # Port used by the Nucleo
        self.BAUDRATE = 9600
        
        # Check if the port exists
        if not os.path.exists(self.PORT):
            self.get_logger().error(f"The port {self.PORT} does not exist.")
            exit(1)

        # Initialize serial communication
        try:
            self.ser = serial.Serial(self.PORT, self.BAUDRATE, timeout=1)
            self.get_logger().info(f"Serial connection established with {self.PORT} at {self.BAUDRATE} bps.")
            time.sleep(2)  # Time to ensure the connection is ready
        except serial.SerialException as e:
            self.get_logger().error(f"Error during serial connection: {e}")
            exit(1)

        # Create a subscriber to listen for commands
        self.command_subscriber = self.create_subscription(
            String,
            'command_topic',  # Change this to your desired topic name
            self.command_callback,
            10
        )

        # Start a thread to continuously read from the serial port
        self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        self.read_thread.start()

    def send_command(self, command):
        """Send a command to the Nucleo."""
        try:
            self.ser.write((command + "\n").encode())
            self.get_logger().info(f"Sent command: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def command_callback(self, msg):
        """Callback function to handle incoming commands."""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        self.send_command(command)

    def read_from_serial(self):
        """Continuously read responses from the Nucleo."""
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    response = self.ser.readline().decode().strip()
                    self.get_logger().info(f"Nucleo response: {response}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error: {e}")
            time.sleep(0.1)  # Small delay to prevent busy waiting

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    
    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        pass
    finally:
        if serial_node.ser.is_open:
            serial_node.ser.close()
            serial_node.get_logger().info("Serial connection closed.")
        serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()