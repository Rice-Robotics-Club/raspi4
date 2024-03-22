import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class ServoOpenNode(Node):
    def __init__(self):
        super().__init__('servo_open_node')
        self.publisher = self.create_publisher(Float64, 'servo_angle', 10)

    def set_servo_angle(self, angle):
        msg = Float64()
        msg.data = angle
        self.publisher.publish(msg)
        self.get_logger().info(f'Set servo angle to: {angle} degrees')

def main(args=None):
    rclpy.init(args=args)
    node = ServoOpenNode()
    node.set_servo_angle(90.0)  # Set the servo to 90 degrees
    time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
