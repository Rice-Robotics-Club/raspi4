import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from .root_servo_control import ServoWrapper
from time import sleep

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        
        self.declare_parameter('pin', rclpy.Parameter.Type.INTEGER)
        
        self.servo = ServoWrapper(self.get_parameter('pin').value)
        self.subscription = self.create_subscription(
            Float64, 'servo_angle', self.angle_callback, 10)
        
        # just to see if it works :)
        self.get_logger().info("testing time!")
        try:
            self.servo.test(2)
        except:
            self.get_logger().info("test failed!")

    def destroy_node(self):
        self.servo.shutdown()
        super().destroy_node()

    # not using this rn - just testing in init TODO make angle setting work
    def angle_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f'Received angle: {angle}')
        # gpiozero needs a value between -1 and 1
        servo = (angle/90.0) - 1.0

        sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()