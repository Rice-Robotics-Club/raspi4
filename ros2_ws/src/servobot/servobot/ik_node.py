import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from enum import Enum
from ik_controller import IKController


class IKNode(Node):
    def __init__(self):
        super().__init__("leg_node")

        self.get_logger().info(f"initializing {self.get_name()}")
        
        self.controller = IKController(1.6, 1.0, 0.6, 2.8, 1.8)

        self.angles = self.create_publisher(Float64MultiArray, "servo_angles", 10)

        self.subscription = self.create_subscription(
            Float64MultiArray, "leg_positions", self.position_callback, 10
        )

        self.msg = Float64MultiArray()
        self.msg.data = 10.0

    def position_callback(self, msg: Float64MultiArray):
        """uses inverse kinematics to transform cartesian coordinate position of foot to the
            motor angles required to achieve this position

        Args:
            msg (Float64MultiArray): float vectors representing position of feet relative to robot
        """
        vals = msg.data
        
        if len(vals) != 12:
            raise Exception("incorrect number of position arguments")
        
        fl_pos = tuple(msg.data[0:3])
        fr_pos = tuple(msg.data[3:6])
        bl_pos = tuple(msg.data[6:9])
        br_pos = tuple(msg.data[9:])
        fl = self.controller.solve_leg(fl_pos, leg=0)
        fr = self.controller.solve_leg(fr_pos, leg=1)
        bl = self.controller.solve_leg(bl_pos, leg=2)
        br = self.controller.solve_leg(br_pos, leg=3)
        self.publish(fl, fr, bl, br)

    def publish(
        self, fl: tuple[float], fr: tuple[float], bl: tuple[float], br: tuple[float]
    ):
        """ publishes servo angles to multi_servo_node

        Args:
            fl (tuple[float]): tuple of fl angles
            fr (tuple[float]): tuple of fr angles
            bl (tuple[float]): tuple of bl angles
            br (tuple[float]): tuple of br angles
        """
        self.msg.data = [*fl, *fr, *bl, *br]
        self.angles.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
