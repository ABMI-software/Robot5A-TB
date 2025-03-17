import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32  # For subscribing to current position
from tf2_msgs.msg import TFMessage  # Import the TFMessage type
from ament_index_python.packages import get_package_share_directory
import csv
import os
import time

class CommandExecutorNode(Node):
    def __init__(self):
        super().__init__('commands_executor_node')  # Initialize node with name 'commands_executor_node'

        # Initialize variables
        self.commands = []  # List to store commands from file
        self.current_command_index = 0  # Index of current command
        self.is_executing = False  # Flag to track if a command is being executed
        self.csv_file_path = '/home/chipmunk-151/Robot5A-TB/src/chariot_control/logs/aruco_log.csv'  # Path to CSV log file
        self.target_reached = False  # Flag to check if target is reached
        self.initial_position = 0.0  # Starting position in mm
        self.end_position = 0.0  # End position based on commands in mm
        self.speed = 200.0  # Default speed in mm/s
        self.current_position = 0.0  # Latest position from /current_position in mm

        # Create a publisher for command topic
        self.command_publisher = self.create_publisher(String, '/command_topic', 10)

        # Create a subscriber to listen for ArUco marker transforms
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            50  # Higher QoS for frequent TF messages
        )

        # Create a subscriber to listen for serial responses
        self.serial_response_subscriber = self.create_subscription(
            String,  
            '/serial_response_topic',
            self.serial_response_callback,
            10
        )

        # Create a subscriber to listen for current position
        self.position_subscriber = self.create_subscription(
            Float32,
            '/current_position',
            self.position_callback,
            10
        )

        # Load commands from a file
        self.load_commands('commands/commands.txt')

        # Initialize CSV file
        self.initialize_csv()

        # Start a timer to execute commands after initialization
        self.create_timer(4.0, self.start_command_execution)  # Delay start by 4 seconds

        # Create a timer to check for command completion
        self.create_timer(0.1, self.check_command_completion)  # Check every 100 ms

    def load_commands(self, filename):
        """Load commands from a text file."""
        package_share_directory = get_package_share_directory('chariot_control')  # Get package directory
        full_path = os.path.join(package_share_directory, filename)  # Construct full file path

        self.get_logger().info(f"Attempting to load commands from: {full_path}")
        if os.path.isfile(full_path):  # Check if file exists
            with open(full_path, 'r') as file:
                self.commands = [line.strip() for line in file.readlines()]  # Read and strip each line
            self.get_logger().info(f"Loaded {len(self.commands)} commands from {full_path}.")
        else:
            self.get_logger().error(f"Command file {full_path} not found or is not a file.")

    def initialize_csv(self):
        """Initialize the CSV file for logging."""
        with open(self.csv_file_path, mode='w', newline='') as csvfile:  # Open in write mode
            writer = csv.writer(csvfile)
            writer.writerow(['Command', 'Speed (mm/s)', 'Initial Position (mm)', 'End Position (mm)', 'Current Position (mm)', 'Time (s)', 'ArUco ID', 'Translation X (mm)', 'Translation Y (mm)', 'Translation Z (mm)', 'Rotation X', 'Rotation Y', 'Rotation Z', 'Rotation W'])  # Write header

    def position_callback(self, msg):
        """Callback function to handle incoming current position messages."""
        self.current_position = msg.data  # Update current position from /current_position topic
        # self.get_logger().info(f"Received current position: {self.current_position} mm")  # Uncomment for debugging

    def tf_callback(self, msg):
        """Callback function to handle incoming TF messages."""
        if self.is_executing:  # Only log if a command is active
            for transform in msg.transforms:  # Process each transform in the message
                aruco_id, translation, rotation = self.extract_info(transform)
                self.log_to_csv(self.commands[self.current_command_index], aruco_id, translation, rotation)

    def extract_info(self, transform):
        """Extract relevant ArUco information and full transform information."""
        aruco_id = transform.child_frame_id  # Child_frame_id is the ArUco marker ID
        translation = transform.transform.translation  # Translation vector (meters)
        rotation = transform.transform.rotation  # Quaternion rotation
        
        return aruco_id, translation, rotation

    def serial_response_callback(self, msg):
        """Callback function to handle responses from the serial node."""
        self.get_logger().info(f"Received serial response: {msg.data}")
        if "Reached target position" in msg.data or "Speed set to" in msg.data:  # Check for completion messages
            self.target_reached = True  # Set flag when target is reached or speed is set
            if "Speed set to" in msg.data:
                speed_value = float(msg.data.split(":")[-1].strip())  # Extract speed value
                self.speed = speed_value  # Update speed variable
                self.get_logger().info(f"Speed updated to: {self.speed}")

    def start_command_execution(self):
        """Start executing commands after initialization."""
        if self.commands:  # Check if there are commands to execute
            self.execute_command()  # Start execution
        else:
            self.get_logger().error("No commands loaded. Command execution will not start.")

    def execute_command(self):
        """Execute the current command."""
        if self.current_command_index < len(self.commands):  # Check if there are more commands
            if not self.is_executing:  # Only execute if not already running
                command = self.commands[self.current_command_index]
                self.get_logger().info(f"Executing command: {command}")
                self.is_executing = True
                self.target_reached = False  # Reset completion flag

                # Determine initial and end positions based on command
                if command.startswith('P'):
                    self.end_position = float(command[1:])  # Extract position value
                    self.get_logger().info(f"Setting end position to: {self.end_position}")
                elif command.startswith('V'):
                    self.get_logger().info(f"Speed command received: {command}")

                # Publish the command to the /command_topic
                command_msg = String(data=command)
                self.command_publisher.publish(command_msg)
                self.get_logger().info(f"Published command: {command}")

    def check_command_completion(self):
        """Check if the current command has been completed."""
        if self.is_executing and self.target_reached:  # Check if command is done
            self.is_executing = False
            self.get_logger().info(f"Finished executing command: {self.commands[self.current_command_index]}")
            self.initial_position = self.end_position  # Update initial position
            self.current_command_index += 1  # Move to next command

            time.sleep(0.5)  # 0.5-second delay between commands
            
            if self.current_command_index < len(self.commands):  # More commands to execute?
                self.execute_command()  # Process next command
            else:
                self.get_logger().info("All commands executed.")  # Done with all commands

    def log_to_csv(self, command, aruco_id, translation, rotation):
        """Log command, ArUco information, and transform information to the CSV file."""
        current_time = time.time()  # Get current time in seconds

        # Extract translation and rotation components, convert translations to mm
        translation_x = translation.x * 1000  # Convert meters to mm
        translation_y = translation.y * 1000
        translation_z = translation.z * 1000
        rotation_x = rotation.x  # Quaternion components (unitless)
        rotation_y = rotation.y
        rotation_z = rotation.z
        rotation_w = rotation.w

        with open(self.csv_file_path, mode='a', newline='') as csvfile:  # Append to CSV
            writer = csv.writer(csvfile)
            writer.writerow([command, self.speed, self.initial_position, self.end_position, self.current_position, current_time, aruco_id, translation_x, translation_y, translation_z, rotation_x, rotation_y, rotation_z, rotation_w])

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    commands_executor_node = CommandExecutorNode()  # Create node instance

    try:
        rclpy.spin(commands_executor_node)  # Keep node running
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        commands_executor_node.destroy_node()  # Clean up node
        if rclpy.ok():  # Only call shutdown if ROS is still initialized
            rclpy.shutdown()  # Shutdown ROS 2

if __name__ == '__main__':
    main()