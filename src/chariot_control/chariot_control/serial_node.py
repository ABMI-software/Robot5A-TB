import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32  # For publishing position as a float
import serial
import time
import os
import threading

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')  # Initialize the node with name 'serial_node'
        
        # Serial port configuration
        self.PORT = "/dev/ttyACM0"  # Port used by the Nucleo/Arduino
        self.BAUDRATE = 9600        # Baud rate for serial communication
        
        # Check if the port exists
        if not os.path.exists(self.PORT):
            self.get_logger().error(f"The port {self.PORT} does not exist.")
            exit(1)

        # Initialize serial communication
        try:
            self.ser = serial.Serial(self.PORT, self.BAUDRATE, timeout=1)  # Open serial port with 1s timeout
            self.get_logger().info(f"Serial connection established with {self.PORT} at {self.BAUDRATE} bps.")
            time.sleep(2)  # Wait 2 seconds to ensure the connection stabilizes
            self.ser.reset_input_buffer()  # Clear any residual data in the buffer
        except serial.SerialException as e:
            self.get_logger().error(f"Error during serial connection: {e}")
            exit(1)

        # Create a subscriber to listen for commands
        self.command_subscriber = self.create_subscription(
            String,
            'command_topic',  # Topic to receive commands from
            self.command_callback,
            10  # QoS profile depth
        )

        # Create a publisher for serial responses (excluding position)
        self.response_publisher = self.create_publisher(String, '/serial_response_topic', 10)

        # Create a new publisher for current position
        self.position_publisher = self.create_publisher(Float32, '/current_position', 10)

        # Flag to track if node is shutting down
        self.is_shutting_down = False

        # Start a thread to continuously read from the serial port
        self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)  # Daemon thread exits with program
        self.read_thread.start()

    def send_command(self, command):
        """Send a command to the Nucleo/Arduino."""
        try:
            self.ser.write((command + "\n").encode())  # Send command with newline, encoded to bytes
            # self.get_logger().info(f"Sent command: {command}")  # Uncomment to log sent commands
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def command_callback(self, msg):
        """Callback function to handle incoming commands."""
        command = msg.data  # Extract command from message
        self.get_logger().info(f"Received command: {command}")
        self.send_command(command)  # Send to Arduino

    def read_from_serial(self):
        """Continuously read responses from the Nucleo/Arduino."""
        while rclpy.ok() and not self.is_shutting_down:  # Run while ROS is active and not shutting down
            try:
                if self.ser.in_waiting > 0:  # Check if there’s data in the serial buffer
                    response = self.ser.readline().decode('utf-8', errors='ignore').strip()  # Read one line, handle decode errors
                    if response:  # Only process non-empty responses
                        self.get_logger().info(f"Nucleo response: {response}")

                        # Check if the response is a position update
                        if response.startswith("Current Position:"):
                            try:
                                position_str = response.split(":")[1].strip()  # Extract number after "Current Position:"
                                position = float(position_str)  # Convert to float
                                position_msg = Float32(data=position)  # Create Float32 message
                                self.position_publisher.publish(position_msg)  # Publish to /current_position
                            except (IndexError, ValueError) as e:
                                self.get_logger().error(f"Failed to parse position: {response}, error: {e}")
                        else:
                            # Publish all other responses to /serial_response_topic
                            response_msg = String(data=response)
                            self.response_publisher.publish(response_msg)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error: {e}")
            time.sleep(0.05)  # 50 ms delay to keep up with 10 FPS (100 ms) updates

    def on_shutdown(self):
        self.is_shutting_down = True
        time.sleep(0.1)  # Brief delay to let read thread stop
        if self.ser.is_open:
            self.get_logger().info("Serial connection closing...")  # Log before closing
            self.ser.close()
            # Log moved before close to ensure publisher is still valid

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    serial_node = SerialNode()  # Create node instance
    
    try:
        rclpy.spin(serial_node)  # Keep node running
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        serial_node.on_shutdown()  # Custom shutdown handling
        serial_node.destroy_node()  # Clean up node
        if rclpy.ok():  # Only call shutdown if ROS is still initialized
            rclpy.shutdown()  # Shutdown ROS 2

if __name__ == '__main__':
    main()