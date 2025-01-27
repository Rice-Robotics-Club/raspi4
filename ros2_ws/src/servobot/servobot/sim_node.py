import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import typing

from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

from omni.isaac.core.utils.extensions import enable_extension

enable_extension("omni.isaac.ros2_bridge")
simulation_app.update()

import omni

class SimNode(Node):
    def __init__(self):
        super().__init__("multi_servo_node")

        self.servo_angles = self.create_subscription(
            Float64MultiArray, "servo_angles", self.servo_angles_callback, 10
        )

    def servo_angles_callback(self, msg: Float64MultiArray):
        """sets servo angles from float array stored in message

        Args:
            msg (Float64MultiArray): float array interface for /servo_angles topic
        """
        pass
        

def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
