import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from adafruit_servokit import ServoKit


class MultiServoNode(Node):
    def __init__(self):
        super().__init__("multi_servo_node")

        self.count = (
            self.declare_parameter("count", rclpy.Parameter.Type.BOOL).value or 1
        )

        self.get_logger().info(
            f"initializing {self.get_name()}"
        )

        self.pca = ServoKit(channels=self.count)

        self.subscription = self.create_subscription(
            Float64MultiArray, "servo_angles", self.angle_callback, 10
        )

    def angle_callback(self, msg: Float64MultiArray):
        """sets servo angles from message value

        Args:
            msg (Float64MultiArray): float array interface for servo_angles topic
        """
        for i in range(self.count):
            angle = msg.data[i] or 0.0
            self.pca.servo[i].angle = angle
            self.get_logger().info(f"setting servo #{i} to angle: {angle}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()