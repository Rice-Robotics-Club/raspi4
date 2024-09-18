import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ControlNode(Node):
    def __init__():
        super().__init__("control_node")
        
    def 


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
