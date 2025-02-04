import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import os

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

        # Global variables for tracking
        self.positions_mode1 = []
        self.positions_mode2 = []
        self.aller_retours_mode3 = 0
        self.current_speed = 0  # Stores the current speed
        self.start_time = time.time()  # Start time for feedback

        # Create a timer to periodically check for commands
        self.timer = self.create_timer(1.0, self.check_for_commands)

        # Create a subscriber to listen for commands
        self.command_subscriber = self.create_subscription(
            String,
            'command_topic',  # Change this to your desired topic name
            self.command_callback,
            10
        )

    def send_command(self, command):
        """Send a command to the Nucleo and read the response."""
        try:
            self.ser.write((command + "\n").encode())
            time.sleep(0.1)
            while self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                self.get_logger().info(response)
                return response
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def command_callback(self, msg):
        """Callback function to handle incoming commands."""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        if command == '1':
            self.send_command('1')  # Activate manual mode
            self.get_logger().info("Manual mode activated. Enter 'P <position_mm>' to set a position or 'S' to stop.")
        elif command.startswith('P '):
            position = command[2:]  # Extract the position value
            self.send_command(f'P {position}')  # Send position command to the Nucleo
            self.get_logger().info(f"Setting position to {position} mm.")
        elif command == 'S':
            self.send_command('S')  # Send stop command to the Nucleo
            self.get_logger().info("Stopping the movement.")
        elif command == 'M':
            self.get_logger().info("Displaying available modes...")
            self.send_command('M')  # Send command to display modes
        else:
            self.get_logger().warning(f"Unknown command: {command}")

    def check_for_commands(self):
        # This method can be used for additional command checks if needed
        pass

    def manual_mode(self):
        """Manual mode to set specific positions."""
        self.get_logger().info("Manual mode activated. Enter a position in mm or 'S' to stop.")
        self.send_command('1')  # Activate manual mode
        # Implement a way to receive positions via a ROS2 topic or service

    def random_mode(self):
        """Random mode for random movements."""
        self.send_command('2')  # Activate random mode
        # Implement a way to receive the number of random movements via a ROS2 topic or service

    def return_mode(self):
        """Return mode between two limit positions."""
        self.send_command('3')  # Activate return mode
        # Implement a way to stop the mode via a ROS2 topic or service

    def adjust_speed(self):
        """Adjust the movement speed."""
        # Implement a way to receive speed adjustments via a ROS2 topic or service

    def return_to_initial_state(self):
        """Return the cart to the initial position."""
        self.send_command('R')

    def display_feedback(self):
        """Display a summary of actions performed."""
        self.get_logger().info(f"Positions reached in manual mode: {self.positions_mode1}")
        self.get_logger().info(f"Positions reached in random mode: {self.positions_mode2}")
        self.get_logger().info(f"Number of returns performed in mode 3: {self.aller_retours_mode3}")
        self.get_logger().info(f"Current speed: {self.current_speed} mm/s")
        self.get_logger().info(f"Total test duration: {round(time.time() - self.start_time, 2)} seconds")

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