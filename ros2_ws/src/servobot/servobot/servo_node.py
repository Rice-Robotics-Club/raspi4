import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gpiozero import AngularServo


class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_node")

        self.pin = self.declare_parameter("pin", rclpy.Parameter.Type.INTEGER).value
        self.mock = (
            self.declare_parameter("mock", rclpy.Parameter.Type.BOOL).value or False
        )

        self.get_logger().info(
            f"initializing {'MOCK' if self.mock else ''} {self.get_name()} w/ pin: {self.pin}"
        )

        self.servo = (
            AngularServo(self.pin, min_angle=0, max_angle=135)
            if not self.mock
            else None
        )
        self.subscription = self.create_subscription(
            Float64, "servo_angle", self.angle_callback, 10
        )

    def angle_callback(self, msg: Float64):
        """sets servo angle from message value

        Args:
            msg (Float64): float interface for servo_angle topic w/ range [0, 135]
        """
        angle = msg.data

        if angle <= 135 or angle >= 0:
            if self.servo:
                self.servo.angle = angle
            self.get_logger().info(
                f"setting servo at pin: {self.pin} to angle: {angle}"
            )
        else:
            self.get_logger().info(
                f"angle {angle} out of range for servo at pin: {self.pin}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
