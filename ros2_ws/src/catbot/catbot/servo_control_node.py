import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gpiozero import Servo
from .root_servo_control import ServoWrapper
from time import sleep

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        
        self.declare_parameter('pin', rclpy.Parameter.Type.INT)
        
        self.servo = Servo(self.get_parameter('pin'))
        self.subscription = self.create_subscription(
            Float64, 'servo_angle', self.angle_callback, 10)


    # not using this rn - just testing in init TODO make angle setting work
    def angle_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f'Received angle: {angle}')
        # gpiozero needs a value between -1 and 1
        angle_to_servo = (angle/90.0) - 1.0
        self.servo.value = angle_to_servo

        sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()