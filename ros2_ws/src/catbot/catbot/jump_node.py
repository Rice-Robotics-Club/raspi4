import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from .controllers.odrive_controller import ODriveController
from catbot_msg.action import Jump


class JumpNode(Node):
    def __init__(self):
        super().__init__("control_node")
        self.declare_parameter("gear_ratio", 8.0)
        self.declare_parameter("max_torque", 12.4)
        self.declare_parameter("brace_torque", 2.0)
        self.declare_parameter("poising_torque", 0.24)
        self.declare_parameter("bracing_angles", 70.0)
        self.declare_parameter("normal_angles", 45.0)

        self.update_parameters()

        self.motor0 = ODriveController(self, namespace="odrive_axis0")
        self.motor1 = ODriveController(self, namespace="odrive_axis1")

        # makes jump callable
        self.jump_action = ActionServer(self, Jump, "jump", self.jump)

        self.motor0.wait_for_axis_state()
        self.motor1.wait_for_axis_state()

        self.zero_angle1 = 0.0  # Reference angle for the top motor
        self.zero_angle2 = 0.0  # Reference angle for the bottom motor

        self.extended_angle1 = 360 * (self.gear_ratio / 360)

    def update_parameters(self):
        self.gear_ratio = self.get_parameter("gear_ratio").value
        self.max_torque = self.get_parameter("max_torque").value
        self.poising_torque = self.get_parameter("poising_torque").value
        self.brace_torque = self.get_parameter("brace_torque").value
        self.bracing_angle = self.angle_to_position(
            self.get_parameter("bracing_angle").value
        )
        self.normal_angle = self.angle_to_position(
            self.get_parameter("normal_angle").value
        )
        self.bracing_angle = self.angle_to_position(
            self.get_parameter("bracing_angle").value
        )

    def angle_to_position(self, angle: float) -> float:
        return angle / (self.gear_ratio * 360)

    def wait_seconds(self, seconds: float):
        self.get_clock().sleep_for(Duration(seconds=seconds))

    def jump(self, goal_handle):
        self.positions_phase()
        self.winding_phase()
        self.pouncing_bracing_phase()
        self.landing_phase()
        goal_handle.succeed()
        return Jump.Result()

    def positions_phase(self):
        self.motor0.set_position(self.standard_angle + self.zero_angle1)
        self.motor1.set_position(self.standard_angle + self.zero_angle2)
        self.get_logger().info("positions phase: motors set to standard angles")
        self.wait_seconds(2)

    def winding_phase(self):
        self.motor1.set_torque(self.poising_torque)
        self.get_logger().info("Winding phase: bottom motor winding the spring")
        self.wait_seconds(3)

    def jumping_phase(self):
        self.motor0.set_position(self.extended_angle1 + self.zero_angle1)
        self.motor1.set_position(self.extended_angle2 + self.zero_angle2)

    def pouncing_bracing_phase(self):
        self.motor0.set_torque(self.brace_torque)
        self.motor1.set_torque(self.brace_torque)
        self.get_logger().info(
            "pouncing/bracing phase: motors fully extend the spring"
        )
        self.wait_seconds(0.3)

    def landing_phase(self):
        self.motor0.set_position(self.standard_angle - self.zero_angle1)
        self.motor1.set_position(self.standard_angle - self.zero_angle2)
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
