import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from .ik_controller import IKController
import typing


class IKNode(Node):
    def __init__(self):
        super().__init__("leg_node")

        a1 = self.declare_parameter("a1", 1.6)
        a2 = self.declare_parameter("a2", 1.0)
        a3 = self.declare_parameter("a3", 0.6)
        l1 = self.declare_parameter("l1", 4.325)
        l2 = self.declare_parameter("l2", 3.0)

        self.controller = IKController(a1.value, a2.value, a3.value, l1.value, l2.value)

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
        positions: list[float] = positions_msg.data
        
        output: list[float] = []
        
        for leg in range(len(positions) // 3):
            output.extend(self.controller.solve_leg(list(positions[(leg * 3):(3 + leg * 3)]), leg=leg))

        self.angles_msg.data = output

        self.servo_angles.publish(self.angles_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
