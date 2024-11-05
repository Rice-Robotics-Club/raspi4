import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import math
import typing


class CircleTestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.radius = self.declare_parameter("radius", 1.0).value
        self.height = self.declare_parameter("height", 4.5).value
        self.period = self.declare_parameter("period", 5.0).value
        self.interval = self.declare_parameter("interval", 0.01).value
        self.add_on_set_parameters_callback(self.set_param)

        self.leg_positions = self.create_publisher(
            Float64MultiArray, "/leg_positions", 10
        )

        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.angle = 0.0
        self.delta = (math.tau * self.interval) / self.period
        self.timer.timer_period_ns = self.interval * 1000000000.0

        self.msg = Float64MultiArray()
        self.msg.data = []

    def set_param(self, params: list[Parameter]):
        param = params.pop()
        
        match param.name:
            case "radius":
                self.radius = param.value
            case "height":
                self.height = param.value
            case "period":
                self.period = param.value
            case "interval":
                self.interval = param.value
        
        self.delta = (math.tau * self.interval) / self.period
        self.timer.timer_period_ns = self.interval * 1000000000.0
        
        return SetParametersResult(successful=True)

    def timer_callback(self) -> None:
        
        l = [
        	0.7,
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
