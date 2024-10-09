import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node

POSITIONS = [
    [1.6, -4.5, 1.6],
    [0.6, -4.5, 2.6],
    [-0.4, -4.5, 1.6],
    [0.6, -4.5, 0.6]
]

class TestNode(Node):
    def __init__(self):
        super.__init__("test_node")
        
        self.get_logger().info(f"initializing {self.get_name()}")
        
        self.pub = self.create_publisher(Float64MultiArray, "/leg_positions", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.msg = Float64MultiArray()
        self.msg.data = []
        self.i = 0
        
    def timer_callback(self):
        self.msg.data[0] = POSITIONS[self.i % len(POSITIONS)][0]
        self.msg.data[1] = POSITIONS[self.i % len(POSITIONS)][1]
        self.msg.data[2] = POSITIONS[self.i % len(POSITIONS)][2]
        self.i += 1
        self.pub.publish(self.msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()