import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from rclpy.parameter import Parameter
import math
import typing


class CircleTestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.get_logger().info(f"initializing {self.get_name()}")

        self.radius = self.declare_parameter("radius", 1.0)
        self.height = self.declare_parameter("height", 4.5)
        self.period = self.declare_parameter("period", 5.0)
        self.interval = self.declare_parameter("interval", 0.01)
        self.add_on_set_parameters_callback(self.set_parameters)

        self.leg_positions = self.create_publisher(
            Float64MultiArray, "/leg_positions", 10
        )

        self.timer = self.create_timer(self.interval, self.timer_callback)
        
        self.set_parameters([
            self.radius,
            self.height,
            self.period,
            self.interval
        ])

        self.msg = Float64MultiArray()
        self.msg.data = []

    def set_parameters(self, params: list[Parameter]):
        params = {p.name: p for p in params}
        
        self.radius = params["radius"].value
        self.height = params["height"].value
        self.period = params["period"].value
        self.interval = params["interval"].value
        
        self.delta = (math.tau * self.interval) / self.period
        self.timer.timer_period_ns = self.interval * 1000000000.0

    def timer_callback(self) -> None:
        l = [
        	0.7 + self.radius * math.sin(self.angle),
            -self.height,
            self.radius * math.cos(self.angle),
        ] * 4
        self.msg.data = l
        self.angle += self.delta
        self.leg_positions.publish(self.msg)
        # self.get_logger().info(f"input position: {l}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CircleTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
