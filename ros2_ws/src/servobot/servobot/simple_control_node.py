import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class SimpleControlNode(Node):
    def __init__(self):
        super().__init__("simple_control_node")

        self.tick = self.create_publisher(Int64, "tick", 10)
        self.timer = self.create_timer(
            self.declare_parameter("interval", 1.0).value, self.timer_callback
        )
        
        self.msg = Int64()
        self.msg.data = 0

    def timer_callback(self):
        """ for simplified walking motion, sends increasing integer to iterate through 
            pre-defined motor angles for each leg
        """
        self.tick.publish(self.msg)
        self.msg.data += 1
        


def main(args=None):
    rclpy.init(args=args)
    node = SimpleControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
