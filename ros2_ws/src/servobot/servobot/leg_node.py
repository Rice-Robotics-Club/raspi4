import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int64
from geometry_msgs.msg import Vector3
import math
from enum import Enum


FRAMES = 8
MOTOR_1_POSITION_ARRAY = [0, 0, 0, 0, 0, 0, 0, 0]
MOTOR_2_POSITION_ARRAY = [0, 0, 0, 0, 0, 0, 0, 0]
MOTOR_3_POSITION_ARRAY = [0, 0, 0, 0, 0, 0, 0, 0]


class Limb(Enum):
    FRONT_LEFT = 0
    FRONT_RIGHT = 1
    BACK_LEFT = 2
    BACK_RIGHT = 3


class LegNode(Node):
    def __init__(self):
        super().__init__("leg_node")

        self.get_logger().info(f"initializing {self.get_name()}")

        self.limb = Limb(self.declare_parameter("limb"))
        self.offset = int(self.declare_parameter("start_index"))

        self.servo0 = self.create_publisher(Float64, "servo_0/servo_angle", 10)
        self.servo1 = self.create_publisher(Float64, "servo_1/servo_angle", 10)
        self.servo2 = self.create_publisher(Float64, "servo_2/servo_angle", 10)

        self.subscription = self.create_subscription(
            Vector3, "leg_position", self.position_callback, 10
        )
        self.subscription = self.create_subscription(
            Int64, "tick", self.tick_callback, 10
        )

        self.msg = Float64()
        self.msg.data = 10.0

    # TODO: inverse kinematics
    def position_callback(self, msg: Vector3):
        """ uses inverse kinematics to transform cartesian coordinate position of foot to the
            motor angles required to achieve this position

        Args:
            msg (Vector3): float vector representing position of foot relative to robot
        """
        magnitude_s = msg.x**2 + msg.y**2 + msg.z**2
        theta1 = math.asin(msg.x / math.sqrt(magnitude_s))
        theta2 = 0
        theta3 = math.acos(
            (2 * (self.leg_length**2) - magnitude_s) / (2 * (self.leg_length**2))
        )

        self.publish(theta1, theta2, theta3)

    def tick_callback(self, msg: Int64):
        """ uses a pre-defined set of angles to move leg im primitive walking motion

        Args:
            msg (Int64): index that increases every time callback occurs
        """
        index = (self.offset + msg.data) % FRAMES
        theta1 = MOTOR_1_POSITION_ARRAY[index]
        theta2 = MOTOR_2_POSITION_ARRAY[index]
        theta3 = MOTOR_3_POSITION_ARRAY[index]

        self.publish(theta1, theta2, theta3)

    def publish(self, theta1: float, theta2: float, theta3: float):
        """ publishes angles to each servo angle topic

        Args:
            theta1 (float): angle for servo 1
            theta2 (float): angle for servo 2
            theta3 (float): angle for servo 3
        """
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
