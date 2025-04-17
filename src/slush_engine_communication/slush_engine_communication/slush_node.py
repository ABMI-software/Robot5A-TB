import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

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
            "R0_Yaw": Motor(0),
            "R1_Pitch": Motor(2),
            "R2_Pitch": Motor(3),
            "R3_Yaw": Motor(4),
            "R4_Pitch": Motor(5),
            "ServoGear": Motor(6)
        }
        self.last_positions = {name: None for name in self.motors}  # Store last commanded position
        for name, motor in self.motors.items():
            motor.resetDev()                  # Reset the motor
            motor.setMicroSteps(16)           # Set microstepping to 1/16 for smoother motion
            motor.setMaxSpeed(500)            # Set a low max speed for slow motion
            motor.setAccel(30)                # Set acceleration
            motor.setDecel(30)                # Set deceleration
            motor.setCurrent(50, 50, 50, 50)  # Set motor currents (adjust as needed)
            self.get_logger().info(f"Configured motor {name}")

        self.sub = self.create_subscription(
            JointState, '/slush_commands', self.command_callback, 10)
        self.debug_pub = self.create_publisher(String, '/slush_debug', 10)

        self.get_logger().info("SlushNode initialized")

    def command_callback(self, msg):
        for name, steps in zip(msg.name, msg.position):
            if name in self.motors:
                steps_int = int(steps)  # Convert position to integer steps
                # Invert steps for R0_Yaw and R3_Yaw
                if name in ["R0_Yaw", "R3_Yaw"]:
                    steps_int = -steps_int
                # Check if this is the first command or if the position has changed
                if self.last_positions[name] is None or steps_int != self.last_positions[name]:
                    self.motors[name].move(steps_int)  # Send move command
                    self.last_positions[name] = steps_int  # Update last position
                    debug_msg = String()
                    debug_msg.data = f"Moved {name} to {steps_int} steps"
                    self.debug_pub.publish(debug_msg)
                else:
                    debug_msg = String()
                    debug_msg.data = f"Ignored repeated command for {name} at {steps_int} steps"
                    self.debug_pub.publish(debug_msg)

    def shutdown_hook(self):
        """Free all motors on shutdown."""
        self.get_logger().info("Shutting down SlushNode, freeing motors")
        for name, motor in self.motors.items():
            try:
                motor.free()  # Release the motor
                self.get_logger().info(f"Freed motor {name}")
            except Exception as e:
                self.get_logger().error(f"Failed to free motor {name}: {str(e)}")

    def __del__(self):
        """Destructor to ensure motors are freed."""
        try:
            self.get_logger().info("Destroying SlushNode, ensuring motors are freed")
            for name, motor in self.motors.items():
                try:
                    motor.free()  # Release the motor
                    self.get_logger().info(f"Freed motor {name} in destructor")
                except Exception as e:
                    self.get_logger().error(f"Failed to free motor {name} in destructor: {str(e)}")
        except Exception:
            # Logger may be unavailable during destruction
            print("Error in SlushNode destructor")

def main(args=None):
    rclpy.init(args=args)
    node = SlushNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.shutdown_hook()  # Explicitly call shutdown hook
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()