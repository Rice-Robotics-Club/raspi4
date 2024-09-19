import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import math

class LegNode(Node):
    def __init__(self):
        super().__init__("leg_node")

        self.get_logger().info(f"initializing {self.get_name()}")

        self.servo0 = self.create_publisher(Float64, "servo_0/servo_angle", 10)
        self.servo1 = self.create_publisher(Float64, "servo_1/servo_angle", 10)
        self.servo2 = self.create_publisher(Float64, "servo_2/servo_angle", 10)
        
        self.leg_length = 1.0

        self.subscription = self.create_subscription(Vector3, "leg_position", self.position_callback, 10)

        self.msg = Float64()
        self.msg.data = 10.0

    # TODO: trig stuff
    def position_callback(self, msg: Vector3):
        magnitude_s = msg.x ** 2 + msg.y ** 2 + msg.z ** 2
        theta1 = math.asin(msg.x / math.sqrt(magnitude_s))
        theta2 = 0
        theta3 = math.acos((2 * (self.leg_length ** 2) - magnitude_s) / (2 * (self.leg_length ** 2)))
        
        self.msg.data = theta1
        self.servo0.publish(self.msg)
        self.msg.data = theta2
        self.servo1.publish(self.msg)
        self.msg.data = theta3
        self.servo2.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = LegNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
