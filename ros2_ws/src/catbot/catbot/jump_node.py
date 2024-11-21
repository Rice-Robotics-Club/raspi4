import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from .controllers.odrive_controller import ODriveController

class JumpNode(Node):
    def __init__(self):
        super().__init__("control_node")
        self.gear_ratio = self.declare_parameter("gear_ratio", 8.0).value
        self.max_torque = self.declare_parameter("max_torque", 12.4).value
        self.poising_torque = 0.24  # Poising torque [Nm]
        self.brace_torque = self.declare_parameter("brace_torque", 2.0).value
        self.bracing_angle = 70 * (
            1 / 360
        )  # Convert bracing angle to rotations
        self.standard_angle = 45 * (
            1 / 360
        )  # Convert standard angle to rotations

        self.motor0 = ODriveController(self, namespace="odrive_axis0")
        # self.motor1 = ODriveController(self, namespace="odrive_axis1")

        self.jump_action = ActionServer(self, None, "jump", self.jump)

        self.motor0.wait_for_axis_state()
        # self.motor1.wait_for_axis_state()

        self.zero_angle1 = 0.0  # Reference angle for the top motor
        self.zero_angle2 = 0.0  # Reference angle for the bottom motor
        
        self.extended_angle1 = 1.0

    def jump(self, goal_handle):
        self.positions_phase()
        # self.winding_phase()
        self.pouncing_bracing_phase()
        self.landing_phase()

    def wait_seconds(self, seconds: float):
        self.get_clock().sleep_for(Duration(seconds=seconds))

    def positions_phase(self):
        self.motor0.set_position(self.standard_angle + self.zero_angle1)
        # self.motor1.set_position(self.standard_angle + self.zero_angle2)
        self.get_logger().info("positions phase: motors set to standard angles")
        self.wait_seconds(2)

    def winding_phase(self):
        # self.motor1.set_torque(self.poising_torque)
        self.get_logger().info("Winding phase: bottom motor winding the spring")
        self.wait_seconds(3)
        
    def jumping_phase(self):
        self.motor0.set_position(self.extended_angle1 + self.zero_angle1)
        # self.motor1.set_position(self.extended_angle2 + self.zero_angle2)

    def pouncing_bracing_phase(self):
        self.motor0.set_torque(self.brace_torque)
        # self.motor1.set_torque(self.brace_torque)
        self.get_logger().info(
            "pouncing/bracing phase: motors fully extend the spring"
        )
        self.wait_seconds(0.3)

    def landing_phase(self):
        self.motor0.set_position(self.standard_angle - self.zero_angle1)
        # self.motor1.set_position(self.standard_angle - self.zero_angle2)
        self.get_logger().info("landing phase: Motors set to standard angle")
        self.phase = "Positions"
        self.get_logger().info(
            "landing completed. Resetting to Positions phase for next cycle"
        )


def main(args=None):
    rclpy.init(args=args)
    jump_node = JumpNode()
    rclpy.spin(jump_node)
    jump_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
