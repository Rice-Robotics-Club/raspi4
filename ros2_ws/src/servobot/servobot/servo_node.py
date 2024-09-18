import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gpiozero import AngularServo

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        
        self.declare_parameter('pin', rclpy.Parameter.Type.INTEGER)
        
        self.servo = AngularServo(self.get_parameter('pin'), min_angle=0, max_angle=135)
        self.get_logger().info(f"initializing servo w/ pin {self.get_parameter('pin')}")
        
        self.subscription = self.create_subscription(
            Float64, 'servo_angle', self.angle_callback, 10)

    # not using this rn - just testing in init TODO make angle setting work
    def angle_callback(self, msg: Float64):
        angle = msg.data
        
        if angle <= 135 or angle >= 0:
          self.servo.angle = angle
          self.get_logger().info(f"setting servo at pin {self.get_parameter('pin')} to angle {angle}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
