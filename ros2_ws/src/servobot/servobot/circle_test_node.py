import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
import math
import typing


class CircleTestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.get_logger().info(f"initializing {self.get_name()}")
        
        self.radius: float = self.declare_parameter("radius", 1.0).value
        self.height: float = self.declare_parameter("height", 4.5).value
        period: float = self.declare_parameter("period", 5.0).value

        self.leg_positions = self.create_publisher(Float64MultiArray, "/leg_positions", 10)
        
        interval = 0.01
        self.timer = self.create_timer(interval, self.timer_callback)

        self.msg = Float64MultiArray()
        self.msg.data = []
        self.angle = 0.0
        self.delta = (math.tau * interval) / period

    def timer_callback(self) -> None:
        self.msg.data = [
            self.radius * math.cos(self.angle),
            -self.height,
            self.radius * math.sin(self.angle),
        ]
        self.angle += self.delta
        self.leg_positions.publish(self.msg)
        self.get_logger().info(f"input position: {self.msg.data}")
        


def main(args=None):
    rclpy.init(args=args)
    node = CircleTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
