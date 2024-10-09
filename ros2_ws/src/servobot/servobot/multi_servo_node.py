import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from adafruit_servokit import ServoKit
import typing


class MultiServoNode(Node):
    def __init__(self):
        super().__init__("multi_servo_node")

        self.count: int = (
            self.declare_parameter("count", rclpy.Parameter.Type.INTEGER).value or 1
        )

        self.angle_offsets: list[float] = self.declare_parameter(
            "offsets", rclpy.Parameter.Type.DOUBLE_ARRAY
        ).value or [67.5 * 12]

        self.get_logger().info(f"initializing {self.get_name()}")

        self.pca = ServoKit(channels=16)

        for i in range(16):
            self.pca.servo[i].set_pulse_width_range(500, 2500)

        self.servo_angles = self.create_subscription(
            Float64MultiArray, "servo_angles", self.angle_callback, 10
        )

    def servo_angles_callback(self, msg: Float64MultiArray):
        """sets servo angles from float array stored in message

        Args:
            msg (Float64MultiArray): float array interface for /servo_angles topic
        """
        for i in range(min(self.count, len(msg.data))):
            angle = msg.data[i]
            if (
                angle >= 0.0 - self.angle_offsets[i]
                and angle <= 135.0 - self.angle_offsets[i]
            ):
                self.pca.servo[i].angle = angle + self.angle_offsets[i]
                self.get_logger().info(f"setting servo #{i} to angle: {angle}")
            else:
                self.get_logger().info(f"angle {angle} out of range for servo #{i}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
