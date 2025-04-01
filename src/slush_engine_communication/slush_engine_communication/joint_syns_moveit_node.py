import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveGroupFeedback, MoveGroupResult
from message_filters import ApproximateTimeSynchronizer, Subscriber
import csv
import os
import time
from ament_index_python.packages import get_package_share_directory

class JointSyncMoveItNode(Node):
    def __init__(self):
        super().__init__('joint_sync_moveit_node')

        # Define joint names
        self.joint_names = ["R0_Yaw", "R1_Pitch", "R2_Pitch", "R3_Yaw", "R4_Pitch", "ServoGear"]
        self.commands = []
        self.current_command_index = 0
        self.is_executing = False
        self.csv_file_path = os.path.join(
            get_package_share_directory('slush_engine_communication'), 'logs', 'joint_sync_log.csv'
        )

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # Synchronized subscribers
        self.combined_sub = Subscriber(self, JointState, '/combined_joint_states', qos_profile=qos)
        self.true_sub = Subscriber(self, JointState, '/true_joint_states', qos_profile=qos)
        self.ts = ApproximateTimeSynchronizer([self.combined_sub, self.true_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.sync_callback)

        # MoveIt 2 Action Client
        self.move_group_client = ActionClient(self, MoveGroupAction, '/move_group')
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveGroup action server not available. Ensure move_group node is running.")

        # Initialize CSV and load commands
        self.initialize_csv()
        self.load_commands('config/joint_commands.txt')

        # Timers
        self.create_timer(4.0, self.start_command_execution)
        self.create_timer(0.1, self.check_command_completion)

    def initialize_csv(self):
        with open(self.csv_file_path, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Command', 'Time (s)', 'True Joints', 'True Positions', 'True Velocities',
                'Combined Joints', 'Combined Positions', 'Combined Velocities', 'MoveIt Success'
            ])

    def load_commands(self, filename):
        package_share_directory = get_package_share_directory('slush_engine_communication')
        full_path = os.path.join(package_share_directory, filename)
        self.get_logger().info(f"Attempting to load commands from: {full_path}")
        if os.path.isfile(full_path):
            with open(full_path, 'r') as file:
                self.commands = [list(map(float, line.strip().split())) for line in file.readlines()]
            self.get_logger().info(f"Loaded {len(self.commands)} commands from {full_path}")
        else:
            self.get_logger().error(f"Command file {full_path} not found")

    def sync_callback(self, combined_msg, true_msg):
        self.combined_joint_states = combined_msg
        self.true_joint_states = true_msg
        self.log_to_csv()

    def start_command_execution(self):
        if self.commands:
            self.execute_command()
        else:
            self.get_logger().error("No commands loaded. Execution will not start.")

    def execute_command(self):
        if self.current_command_index >= len(self.commands):
            self.get_logger().info("All commands executed.")
            return

        if not self.is_executing:
            command = self.commands[self.current_command_index]
            self.get_logger().info(f"Executing command {self.current_command_index + 1}/{len(self.commands)}: {command}")
            self.is_executing = True

            if len(command) != len(self.joint_names):
                self.get_logger().error(f"Command {command} has {len(command)} values, expected {len(self.joint_names)}")
                self.is_executing = False
                self.current_command_index += 1
                return

            # Create MoveGroup goal
            goal = MoveGroupGoal()
            goal.request.group_name = "arm"
            goal.request.num_planning_attempts = 10
            goal.request.allowed_planning_time = 5.0
            goal.request.start_state.is_diff = True  # Use current state as start

            # Set joint constraints
            joint_state = JointState()
            joint_state.name = self.joint_names
            joint_state.position = command
            goal.request.goal_constraints.append({
                "joint_constraints": [
                    {"joint_name": name, "position": pos, "tolerance_above": 0.05, "tolerance_below": 0.05, "weight": 1.0}
                    for name, pos in zip(self.joint_names, command)
                ]
            })

            # Send goal to MoveGroup action server
            self.move_group_client.send_goal_async(
                goal,
                feedback_callback=self.feedback_callback
            ).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Log feedback if needed
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            self.is_executing = False
            self.current_command_index += 1
            return

        self.get_logger().info("MoveGroup goal accepted")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("MoveIt execution succeeded")
        else:
            self.get_logger().error(f"MoveIt execution failed with error code: {result.error_code.val}")
        self.is_executing = False  # Reset regardless of success to proceed to next command

    def check_command_completion(self):
        if not self.is_executing or not hasattr(self, 'combined_joint_states') or not hasattr(self, 'true_joint_states'):
            return

        target_positions = self.commands[self.current_command_index]
        current_positions = self.combined_joint_states.position
        tolerance = 0.05

        if len(current_positions) >= len(target_positions):
            all_close = all(abs(current_positions[i] - target_positions[i]) < tolerance for i in range(len(target_positions)))
            if all_close:
                self.get_logger().info(f"Command {self.current_command_index + 1} completed")
                self.is_executing = False
                self.current_command_index += 1
                time.sleep(0.5)
                self.execute_command()

    def log_to_csv(self):
        if not hasattr(self, 'combined_joint_states') or not hasattr(self, 'true_joint_states'):
            return

        current_time = time.time()
        true_joints = ' '.join(self.true_joint_states.name)
        true_positions = ' '.join(map(str, self.true_joint_states.position))
        true_velocities = ' '.join(map(str, self.true_joint_states.velocity))
        combined_joints = ' '.join(self.combined_joint_states.name)
        combined_positions = ' '.join(map(str, self.combined_joint_states.position))
        combined_velocities = ' '.join(map(str, self.combined_joint_states.velocity))
        command = (self.commands[self.current_command_index] if self.is_executing and
                   self.current_command_index < len(self.commands) else "Idle")
        moveit_success = "Yes" if not self.is_executing else "In Progress"

        with open(self.csv_file_path, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                command, current_time,
                true_joints, true_positions, true_velocities,
                combined_joints, combined_positions, combined_velocities,
                moveit_success
            ])

def main(args=None):
    rclpy.init(args=args)
    try:
        node = JointSyncMoveItNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()