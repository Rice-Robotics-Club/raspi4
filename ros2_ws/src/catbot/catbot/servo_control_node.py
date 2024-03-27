import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from .root_servo_control import ServoWrapper
from time import sleep

class ServoControlNode(Node):
    def __init__(self, servo_wrapper: ServoWrapper):
        super().__init__('servo_control_node')
        self.servo = servo_wrapper
        self.subscription = self.create_subscription(
            Float64, 'servo_angle', self.angle_callback, 10)
        
        # just to see if it works :)
        self.servo.test(2)

    # not using this rn - just testing in init TODO make angle setting work
    def angle_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f'Received angle: {angle}')
        # gpiozero needs a value between -1 and 1
        servo_value = angle/90 - 1

        sleep(1)

def main(args=None):
    rclpy.init(args=args)
    servo = ServoWrapper(11) # TODO make node init with a non-predetermined id
    node = ServoControlNode(servo)

    rclpy.spin(node)

    node.destroy_node()
    servo.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()