import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gpiozero import AngularServo
from time import sleep

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.subscription = self.create_subscription(
            Float64, 'servo_angle', self.angle_callback, 10)
        self.servo = AngularServo(11, min_angle=-90, max_angle=90)  # Assuming the servo is connected to GPIO pin 11

    def angle_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f'Received angle: {angle}')
        if angle >= -90 and angle <= 90:
            self.servo.angle = angle
        else:
            self.get_logger().info('Angle out of range (-90 to 90)')
        sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
