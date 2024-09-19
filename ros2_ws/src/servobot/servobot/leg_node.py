import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3


class LegNode(Node):
    def __init__(self):
        super().__init__("leg_node")

        self.get_logger().info(f"initializing {self.get_name()}")

        self.servo0 = self.create_publisher(Float64, "servo_0/servo_angle", 10)
        self.servo1 = self.create_publisher(Float64, "servo_1/servo_angle", 10)
        self.servo2 = self.create_publisher(Float64, "servo_2/servo_angle", 10)
        
        self.length0 = 1.0
        self.length1 = 1.0
        self.length2 = 1.0

        self.subscription = self.create_subscription(Vector3, "leg_position", self.position_callback, 10)

        self.msg = Float64()
        self.msg.data = 10.0

    # TODO: trig stuff
    def position_callback(self, msg: Vector3):
        
        
        self.servo0.publish(self.msg)
        self.servo1.publish(self.msg)
        self.servo2.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = LegNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
