import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from .ik_controller import IKController
from math import sin, cos, pi
import typing


class IKNode(Node):
    def __init__(self):
        super().__init__("leg_node")

        self.get_logger().info(f"initializing {self.get_name()}")

        a1: float = self.declare_parameter("a1", 1.6).value
        a2: float = self.declare_parameter("a2", 1.0).value
        a3: float = self.declare_parameter("a3", 0.6).value
        l1: float = self.declare_parameter("l1", 2.8).value
        l2: float = self.declare_parameter("l2", 1.8).value

        self.controller = IKController(a1, a2, a3, l1, l2)

        self.servo_angles = self.create_publisher(
            Float64MultiArray, "/servo_angles", 10
        )
        self.leg_positions = self.create_subscription(
            Float64MultiArray, "/leg_positions", self.leg_positions_callback, 10
        )

        self.angles_msg = Float64MultiArray()
        self.angles_msg.data = []

    def leg_positions_callback(self, positions_msg: Float64MultiArray) -> None:
        """uses inverse kinematics to transform cartesian coordinate position of foot to the
            motor angles required to achieve this position

        Args:
            msg (Float64MultiArray): float vectors representing position of feet relative to robot
        """
        array: list[float] = positions_msg.data
        size: int = len(array)

        fl = (
            self.controller.solve_leg(tuple(array[0:3]), leg=0)
            if size >= 3
            else tuple()
        )

        fr = (
            self.controller.solve_leg(tuple(array[3:6]), leg=1)
            if size >= 6
            else tuple()
        )
        bl = (
            self.controller.solve_leg(tuple(array[6:9]), leg=2)
            if size >= 9
            else tuple()
        )
        br = (
            self.controller.solve_leg(tuple(array[9:12]), leg=3)
            if size >= 12
            else tuple()
        )

        self.angles_msg.data = [*fl, *fr, *bl, *br]

        self.servo_angles.publish(self.angles_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
