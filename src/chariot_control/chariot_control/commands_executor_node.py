import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage  # Import the TFMessage type
from ament_index_python.packages import get_package_share_directory
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
        self.csv_file_path = '/home/chipmunk-151/Robot5A-TB/src/chariot_control/logs/aruco_log.csv'
        self.target_reached = False  # Flag to check if target is reached
        self.initial_position = 0.0  # Starting position
        self.end_position = 0.0  # End position based on commands

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
        self.create_timer(4.0, self.start_command_execution)  # Start after 4 seconds

        # Create a timer to check for command completion
        self.create_timer(0.1, self.check_command_completion)  # Check every 100 ms

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
            writer.writerow(['Command', 'Initial Position', 'End Position', 'Time', 'ArUco ID', 'Translation X', 'Translation Y', 'Translation Z', 'Rotation X', 'Rotation Y', 'Rotation Z', 'Rotation W'])  # Header

    def tf_callback(self, msg):
        """Callback function to handle incoming TF messages."""
        if self.is_executing:
            for transform in msg.transforms:
                aruco_id, translation, rotation = self.extract_info(transform)
                self.log_to_csv(self.commands[self.current_command_index], aruco_id, translation, rotation)

    def extract_info(self, transform):
        """Extract relevant ArUco information and full transform information."""
        aruco_id = transform.child_frame_id  # Assuming child_frame_id is the ArUco marker ID
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        
        return aruco_id, translation, rotation

    def serial_response_callback(self, msg):
        """Callback function to handle responses from the serial node."""
        self.get_logger().info(f"Received serial response: {msg.data}")
        if "Reached target position" in msg.data or "Speed set to" in msg.data:
            self.target_reached = True  # Set the flag when the target is reached or speed is set

    def start_command_execution(self):
        """Start executing commands after initialization."""
        if self.commands:
            self.execute_command()  # Start executing commands immediately
        else:
            self.get_logger().error("No commands loaded. Command execution will not start.")

    def execute_command(self):
        """Execute the current command."""
        if self.current_command_index < len(self.commands):
            if not self.is_executing:
                command = self.commands[self.current_command_index]
                self.get_logger().info(f"Executing command: {command}")
                self.is_executing = True
                self.target_reached = False  # Reset the flag

                # Determine initial and end positions based on command
                if command.startswith('P'):
                    self.end_position = float(command[1:])  # Extract the number after 'P'
                    self.get_logger().info(f"Setting end position to: {self.end_position}")
                elif command.startswith('V'):
                    self.get_logger().info(f"Speed command received: {command}")

                # Publish the command to the /command_topic
                command_msg = String(data=command)
                self.command_publisher.publish(command_msg)
                self.get_logger().info(f"Published command: {command}")

    def check_command_completion(self):
        """Check if the current command has been completed."""
        if self.is_executing and self.target_reached:
            self.is_executing = False
            self.get_logger().info(f"Finished executing command: {self.commands[self.current_command_index]}")
            self.initial_position = self.end_position  # Update initial position to the last end position
            self.current_command_index += 1
            
            # Check if there are more commands to execute
            if self.current_command_index < len(self.commands):
                self.execute_command()  # Process the next command
            else:
                self.get_logger().info("All commands executed. Shutting down.")
                rclpy.shutdown()  # Shutdown the node when all commands are done

    def log_to_csv(self, command, aruco_id, translation, rotation):
        """Log command, ArUco information, and transform information to the CSV file."""
        current_time = time.time()  # Get the current time

        # Extract translation and rotation components
        translation_x = translation.x
        translation_y = translation.y
        translation_z = translation.z
        rotation_x = rotation.x
        rotation_y = rotation.y
        rotation_z = rotation.z
        rotation_w = rotation.w

        with open(self.csv_file_path, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([command, self.initial_position, self.end_position, current_time, aruco_id, translation_x, translation_y, translation_z, rotation_x, rotation_y, rotation_z, rotation_w])

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