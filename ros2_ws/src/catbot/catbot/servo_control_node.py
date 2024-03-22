import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.subscription = self.create_subscription(
            Float64, 'servo_angle', self.angle_callback, 10)

    def angle_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f'Received angle: {angle}')
        # Controls the servo based on the received angle

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
