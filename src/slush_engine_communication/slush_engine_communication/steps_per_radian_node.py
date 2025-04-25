import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import math

class StepsPerRadianPublisher(Node):
    def __init__(self):
        super().__init__('steps_per_radian_publisher')

        # Parameters
        self.declare_parameter('motor_name', 'R0_Yaw')  # Motor to calibrate
        self.declare_parameter('step_increment', 10)    # Steps to increment per iteration

        # Get parameter values
        self.motor_name = self.get_parameter('motor_name').get_parameter_value().string_value
        self.step_increment = self.get_parameter('step_increment').get_parameter_value().integer_value

        # List of all motors (matching SlushNode)
        self.all_motors = [
            "R0_Yaw",
            "R1_Pitch",
            "R2_Pitch",
            "R3_Yaw",
            "R4_Pitch",
            "ServoGear"
        ]

        # Validate motor_name
        if self.motor_name not in self.all_motors:
            self.get_logger().error(f"Invalid motor name: {self.motor_name}. Must be one of {self.all_motors}")
            exit(1)

        # State variables
        self.total_steps = 0
        self.is_calibrating = False
        self.full_turn_detected = False
        self.motor_positions = {motor: 0.0 for motor in self.all_motors}  # Track last position for each motor

        # Publishers and subscribers
        self.command_pub = self.create_publisher(JointState, '/slush_commands', 10)
        self.debug_pub = self.create_publisher(String, '/calibration_debug', 10)
        self.result_pub = self.create_publisher(JointState, '/calibration_result', 10)
        self.start_sub = self.create_subscription(
            Bool, '/start_calibration', self.start_calibration_callback, 10)
        self.stop_sub = self.create_subscription(
            Bool, '/stop_calibration', self.stop_calibration_callback, 10)

        # Timer for sending incremental commands
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.get_logger().info(
            f"StepsPerRadianPublisher initialized for motor {self.motor_name}. "
            f"Step increment: {self.step_increment}. "
            f"Publish to /start_calibration to begin."
        )

    def start_calibration_callback(self, msg):
        if msg.data and not self.is_calibrating:
            self.is_calibrating = True
            self.total_steps = 0
            self.full_turn_detected = False
            self.motor_positions = {motor: 0.0 for motor in self.all_motors}  # Reset positions
            self.publish_command()
            self.get_logger().info(
                f"Started calibration for {self.motor_name}. "
                f"Publishing {self.step_increment} steps per iteration."
            )
            debug_msg = String()
            debug_msg.data = f"Calibration started. Total steps for {self.motor_name}: {self.total_steps}"
            self.debug_pub.publish(debug_msg)

    def stop_calibration_callback(self, msg):
        if msg.data and self.is_calibrating:
            self.full_turn_detected = True
            self.is_calibrating = False
            self.calculate_steps_per_radian()
            self.get_logger().info("Calibration stopped.")

    def timer_callback(self):
        if self.is_calibrating and not self.full_turn_detected:
            self.publish_command()
            debug_msg = String()
            debug_msg.data = f"Total steps for {self.motor_name}: {self.total_steps}"
            self.debug_pub.publish(debug_msg)

    def publish_command(self):
        # Update position for the motor being calibrated
        self.motor_positions[self.motor_name] = float(self.total_steps)
        self.total_steps += self.step_increment

        # Create JointState message with positions for all motors
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.all_motors
        msg.position = [self.motor_positions[motor] for motor in self.all_motors]
        self.command_pub.publish(msg)

    def calculate_steps_per_radian(self):
        # One full turn = 2π radians
        steps_per_radian = self.total_steps / (2 * math.pi)
        self.get_logger().info(
            f"Calibration complete for {self.motor_name}. "
            f"Total steps for one revolution: {self.total_steps}. "
            f"Steps per radian: {steps_per_radian:.2f}"
        )

        # Publish result
        result_msg = JointState()
        result_msg.header.stamp = self.get_clock().now().to_msg()
        result_msg.name = [self.motor_name]
        result_msg.position = [steps_per_radian]
        self.result_pub.publish(result_msg)

        debug_msg = String()
        debug_msg.data = f"Steps per radian for {self.motor_name}: {steps_per_radian:.2f}"
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StepsPerRadianPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()