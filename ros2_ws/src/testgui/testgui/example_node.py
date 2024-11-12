import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ExampleNode(Node):
    def __init__(self):
        super().__init__("example_node")
        self.parameter = self.declare_parameter("parameter", 0.1)
        self.publisher = self.create_publisher(Float64, "/topic", 10)
        self.create_timer(
            1,
            lambda: self.get_logger().info(
                f"parameter {self.get_parameter('parameter').get_parameter_value().double_value}"
            ),
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
