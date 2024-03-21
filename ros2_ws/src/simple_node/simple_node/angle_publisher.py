# We can adapt this into two nodes later, an "open position" node and a "closed position." Right now it should just alternate between two positions every 3 seconds.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class AnglePublisher(Node):
    def __init__(self):
        super().__init__('angle_publisher')
        self.publisher = self.create_publisher(Float64, 'servo_angle', 10)
        self.angle = 0.0

    def publish_angle(self):
        while rclpy.ok():
            msg = Float64()
            msg.data = self.angle
            self.publisher.publish(msg)
            self.get_logger().info(f'Published angle: {self.angle}')
            if self.angle == 0.0:
                self.angle = 90.0
            else:
                self.angle = 0.0
            time.sleep(3)

def main(args=None):
    rclpy.init(args=args)
    node = AnglePublisher()
    node.publish_angle()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
