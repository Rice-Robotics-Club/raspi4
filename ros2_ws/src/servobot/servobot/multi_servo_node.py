import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from adafruit_servokit import ServoKit
import typing

class MultiServoNode(Node):
    def __init__(self):
        super().__init__("multi_servo_node")

        self.count: int = self.declare_parameter("count", 1).value

        self.test: bool = self.declare_parameter("test", False).value

        self.angle_offsets: list[float] = self.declare_parameter(
            "offsets", [67.5 * 12]
        ).value
        
        self.angle_multipliers: list[float] = self.declare_parameter(
            "multipliers", [1.0 * 12]
        ).value

        self.get_logger().info(f"initializing {self.get_name()}")

        if not self.test:
            self.pca = ServoKit(channels=16)

            for i in range(16):
                self.pca.servo[i].set_pulse_width_range(500, 2500)

        self.servo_angles = self.create_subscription(
            Float64MultiArray, "servo_angles", self.servo_angles_callback, 10
        )

    def servo_angles_callback(self, msg: Float64MultiArray):
        """sets servo angles from float array stored in message

        Args:
            msg (Float64MultiArray): float array interface for /servo_angles topic
        """
        for i in range(min(self.count, len(msg.data))):
            angle = (msg.data[i] + self.angle_offsets[i]) * self.angle_multipliers[i]
            if (
                angle >= 0.0
                and angle <= 135.0
            ):
                if not self.test:
                    self.pca.servo[i].angle = angle
                    
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
