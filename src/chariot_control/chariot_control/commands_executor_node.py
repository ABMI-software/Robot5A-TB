import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import csv
import os
import time

class CommandExecutorNode(Node):
    def __init__(self):
        super().__init__('commands_executor_node')

        # Initialize variables
        self.commands = []
        self.current_command_index = 0
        self.is_executing = False
        self.csv_file_path = 'aruco_log.csv'

        # Create a subscriber to listen for ArUco marker transforms
        self.tf_subscriber = self.create_subscription(
            TransformStamped,
            '/tf',  # Topic for ArUco marker transforms
            self.tf_callback,
            10
        )

        # Load commands from a file
        self.load_commands('commands.txt')

        # Create a timer to check for command execution
        self.timer = self.create_timer(1.0, self.execute_command)

        # Initialize CSV file
        self.initialize_csv()

    def load_commands(self, filename):
        """Load commands from a text file."""
        if os.path.exists(filename):
            with open(filename, 'r') as file:
                self.commands = [line.strip() for line in file.readlines()]
            self.get_logger().info(f"Loaded {len(self.commands)} commands from {filename}.")
        else:
            self.get_logger().error(f"Command file {filename} not found.")

    def initialize_csv(self):
        """Initialize the CSV file for logging."""
        with open(self.csv_file_path, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Command', 'Initial Position', 'End Position', 'ArUco Info'])  # Header

    def tf_callback(self, msg):
        """Callback function to handle incoming ArUco marker transforms."""
        if self.is_executing:
            # Log the ArUco information while executing a command
            aruco_info = self.extract_aruco_info(msg)
            self.log_to_csv(self.commands[self.current_command_index], aruco_info)

    def extract_aruco_info(self, msg):
        """Extract relevant ArUco information from the transform message."""
        # Here you can extract the necessary information from the msg
        # For demonstration, we will return a dummy string
        return f"Position: ({msg.transform.translation.x}, {msg.transform.translation.y}, {msg.transform.translation.z})"

    def execute_command(self):
        """Execute the current command."""
        if self.current_command_index < len(self.commands):
            if not self.is_executing:
                command = self.commands[self.current_command_index]
                self.get_logger().info(f"Executing command: {command}")
                self.is_executing = True

                # Simulate command execution (replace with actual command sending logic)
                time.sleep(2)  # Simulate time taken to execute the command

                # After execution, update the command index
                self.is_executing = False
                self.current_command_index += 1
                self.get_logger().info(f"Finished executing command: {command}")

    def log_to_csv(self, command, aruco_info):
        """Log command and ArUco information to the CSV file."""
        # Here you would extract initial and end positions from the command
        # For demonstration, we will use dummy values
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