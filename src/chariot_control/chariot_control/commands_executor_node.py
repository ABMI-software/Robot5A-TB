import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage  # Import the TFMessage type
from ament_index_python.packages import get_package_share_directory
import csv
import os

class CommandExecutorNode(Node):
    def __init__(self):
        super().__init__('commands_executor_node')

        # Initialize variables
        self.commands = []
        self.current_command_index = 0
        self.is_executing = False
        self.csv_file_path = '/home/chipmunk-151/Robot5A-TB/src/chariot_control/logs/aruco_log.csv'
        self.target_reached = False  # Flag to check if target is reached

        # Create a publisher for command topic
        self.command_publisher = self.create_publisher(String, '/command_topic', 10)

        # Create a subscriber to listen for ArUco marker transforms
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            50
        )

        # Create a subscriber to listen for serial responses
        self.serial_response_subscriber = self.create_subscription(
            String,  
            '/serial_response_topic',  # Topic for serial responses
            self.serial_response_callback,
            10
        )

        # Load commands from a file
        self.load_commands('commands/commands.txt')

        # Initialize CSV file
        self.initialize_csv()

        # Start a timer to execute commands after initialization
        self.create_timer(1.0, self.start_command_execution)  # Start after 1 second

    def load_commands(self, filename):
        """Load commands from a text file."""
        package_share_directory = get_package_share_directory('chariot_control')
        full_path = os.path.join(package_share_directory, filename)

        self.get_logger().info(f"Attempting to load commands from: {full_path}")

        if os.path.isfile(full_path):
            with open(full_path, 'r') as file:
                self.commands = [line.strip() for line in file.readlines()]
            self.get_logger().info(f"Loaded {len(self.commands)} commands from {full_path}.")
        else:
            self.get_logger().error(f"Command file {full_path} not found or is not a file.")

    def initialize_csv(self):
        """Initialize the CSV file for logging."""
        with open(self.csv_file_path, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Command', 'Initial Position', 'End Position', 'ArUco Info'])  # Header

    def tf_callback(self, msg):
        """Callback function to handle incoming TF messages."""
        if self.is_executing:
            for transform in msg.transforms:
                aruco_info = self.extract_aruco_info(transform)
                self.log_to_csv(self.commands[self.current_command_index], aruco_info)

    def extract_aruco_info(self, transform):
        """Extract relevant ArUco information from the transform message."""
        return f"Position: ({transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z})"

    def serial_response_callback(self, msg):
        """Callback function to handle responses from the serial node."""
        self.get_logger().info(f"Received serial response: {msg.data}")
        if "Reached target position" in msg.data:
            self.target_reached = True  # Set the flag when the target is reached

    def start_command_execution(self):
        """Start executing commands after initialization."""
        self.execute_command()  # Start executing commands

    def execute_command(self):
        """Execute the current command."""
        if self.current_command_index < len(self.commands):
            if not self.is_executing:
                command = self.commands[self.current_command_index]
                self.get_logger().info(f"Executing command: {command}")
                self.is_executing = True
                self.target_reached = False  # Reset the flag

                # Publish the command to the /command_topic
                command_msg = String(data=command)
                self.command_publisher.publish(command_msg)
                self.get_logger().info(f"Published command: {command}")

                # Wait for the target position to be reached
                while not self.target_reached:
                    rclpy.spin_once(self)  # Process callbacks

                # After execution, update the command index
                self.is_executing = False
                self.current_command_index += 1
                self.get_logger().info(f"Finished executing command: {command}")

                # Call execute_command again to process the next command
                self.execute_command()

    def log_to_csv(self, command, aruco_info):
        """Log command and ArUco information to the CSV file."""
        initial_position = "0.0"  # Replace with actual initial position
        end_position = "100.0"  # Replace with actual end position

        with open(self.csv_file_path, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([command, initial_position, end_position, aruco_info])

def main(args=None):
    rclpy.init(args=args)
    commands_executor_node = CommandExecutorNode()

    try:
        rclpy.spin(commands_executor_node)
    except KeyboardInterrupt:
        pass
    finally:
        commands_executor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()