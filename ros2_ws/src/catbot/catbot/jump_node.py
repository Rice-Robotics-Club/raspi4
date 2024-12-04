import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from .controllers.odrive_controller import ODriveController
from catbot_msg.action import Jump
import typing

class JumpNode(Node):
    def __init__(self):
        super().__init__("jump_node")
        self.declare_parameter("gear_ratio", 8.0)
        self.declare_parameter("max_torque", 12.4)
        self.declare_parameter("winding_torque", 2.0)
        self.declare_parameter("brace_torque", 2.0)
        self.declare_parameter("normal_pos0", -1.0)
        self.declare_parameter("normal_pos1", 0.5)
        self.declare_parameter("min_pos0", 0.0)
        self.declare_parameter("min_pos1", 0.0)
        self.declare_parameter("max_pos0", -2.0)
        self.declare_parameter("max_pos1", 1.0)

        self.update_parameters()

        self.motor0 = ODriveController(self, namespace="odrive_axis0")
        self.motor1 = ODriveController(self, namespace="odrive_axis1")

        self.phases = [
            self.update_parameters,
            self.positions_phase,
            self.poising_phase,
            # self.winding_phase,
            self.jumping_phase,
            self.landing_phase,
        ]
        
        self.motor0.wait_for_axis_state()
        self.motor1.wait_for_axis_state()
        
        self.jump_action = ActionServer(self, Jump, "jump", self.jump)

    def update_parameters(self):
        self.gear_ratio = self.get_parameter("gear_ratio").value
        self.max_torque = self.get_parameter("max_torque").value
        self.winding_torque = self.get_parameter("winding_torque").value
        self.brace_torque = self.get_parameter("brace_torque").value
        self.normal_pos0 = self.get_parameter("normal_pos0").value
        self.min_pos0 = self.get_parameter("min_pos0").value
        self.max_pos0 = self.get_parameter("max_pos0").value
        self.normal_pos1 = self.get_parameter("normal_pos1").value
        self.min_pos1 = self.get_parameter("min_pos1").value
        self.max_pos1 = self.get_parameter("max_pos1").value

    def angle_to_position(self, angle: float) -> float:
        return angle / (self.gear_ratio * 360)

    def wait_seconds(self, seconds: float):
        self.get_clock().sleep_for(Duration(seconds=seconds))

    def jump(self, goal_handle: rclpy.action.server.ServerGoalHandle) -> Jump.Result:
        for phase in self.phases:
            if not goal_handle.is_cancel_requested:
                phase()
                goal_handle.publish_feedback(Jump.Feedback(phase=f"{phase.__name__} completed"))
            else:
                goal_handle.abort()
                return Jump.Result()
        goal_handle.succeed()
        return Jump.Result()

    def positions_phase(self):
        self.motor0.set_position(self.normal_pos0)
        self.motor1.set_position(self.normal_pos1)
        self.wait_seconds(1)

    def poising_phase(self):
        self.motor0.set_position(self.min_pos0)
        self.motor1.set_position(self.min_pos1)
        self.wait_seconds(2)

    def winding_phase(self):
        self.motor1.set_torque(self.winding_torque)
        self.wait_seconds(3)

    def jumping_phase(self):
        self.motor0.set_position(self.max_pos0)
        self.motor1.set_position(self.max_pos1)
        self.wait_seconds(0.3)

    def bracing_phase(self):
        self.motor0.set_torque(self.brace_torque)
        self.motor1.set_torque(self.brace_torque)
        self.wait_seconds(1.0)

    def landing_phase(self):
        self.motor0.set_position(self.normal_pos0)
        self.motor1.set_position(self.normal_pos1)


def main(args=None):
    rclpy.init(args=args)
    jump_node = JumpNode()
    rclpy.spin(jump_node)
    jump_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
