import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


try:
    from Slush.Board import sBoard
    from Slush.Motor import Motor
except ImportError:
    print("Slush library not installed")
    exit(1)

class SlushNode(Node):
    def __init__(self):
        super().__init__('slush_node')
        self.board = sBoard()
        self.motors = {
            "R0_Yaw": Motor(4),
            "R1_Pitch": Motor(1),
            "R2_Pitch": Motor(0),
            "R3_Yaw": Motor(2),
            "R4_Pitch": Motor(3),
            "ServoGear": Motor(5)
        }
        for name, motor in self.motors.items():
            motor.setMicroSteps(16)  # Adjust as needed
            motor.setCurrent(50, 50, 50, 50)  # Adjust as needed
            self.get_logger().info(f"Configured motor {name}")

        self.sub = self.create_subscription(
            JointState, '/slush_commands', self.command_callback, 10)

    def command_callback(self, msg):
        for name, steps in zip(msg.name, msg.position):
            if name in self.motors:
                self.motors[name].goTo(int(steps))  # Steps as integer
                self.get_logger().debug(f"Moved {name} to {steps} steps")

def main(args=None):
    rclpy.init(args=args)
    node = SlushNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()