import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
import math

POSITIONS = [
    [1.1, -4.8, 1.6],
    [0.6, -4.8, 2.1],
    [0.1, -4.8, 1.6],
    [0.6, -5.4, 1.6]
]

class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        
        self.get_logger().info(f"initializing {self.get_name()}")
        
        self.pub = self.create_publisher(Float64MultiArray, "/leg_positions", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.msg = Float64MultiArray()
        self.msg.data = []
        self.angle = 0.0
        
    def timer_callback(self):
        self.msg.data = [0.6 + 1.0 * math.cos(self.angle), -4.5, 1.6 + 1.0 * math.sin(self.angle)]
        # self.angle += .1
        self.pub.publish(self.msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()