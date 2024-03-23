import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gpiozero import Servo
from time import sleep

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.subscription = self.create_subscription(
            Float64, 'servo_angle', self.angle_callback, 10)
        self.servo = Servo(25)  # Assuming the servo is connected to GPIO pin 0

    def angle_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f'Received angle: {angle}')
        # gpiozero needs a value between -1 and 1
        servo_value = angle/90 - 1
        self.servo.value = servo_value
        sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
