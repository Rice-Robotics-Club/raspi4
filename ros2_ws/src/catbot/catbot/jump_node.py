import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from .controllers.odrive_controller import ODriveController
from catbot_msg.action import Jump
from odrive.enums import AxisState as AxisStates
from rclpy.action import CancelResponse
import typing


class JumpNode(Node):
    def __init__(self):
        super().__init__("jump_node")

        # declares all parameters for this node
        self.declare_parameter("gear_ratio", 8.0)
        self.declare_parameter("max_torque", 12.4)
        self.declare_parameter("winding_torque", 2.0)
        self.declare_parameter("brace_torque", 2.0)
        
        self.declare_parameter("normal_pos0", -1.0)
        self.declare_parameter("normal_pos1", 0.5)
        
        self.declare_parameter("min_pos0", 0.0)
        self.declare_parameter("min_pos1", 0.0)
        
        self.declare_parameter("torque_pos0", -0.5)
        self.declare_parameter("torque_pos1", 0.2)
        
        self.declare_parameter("max_pos0", -2.0)
        self.declare_parameter("max_pos1", 1.0)

        # initalizes fields corresponding to each parameter
        self.update_parameters()

        # initializes the ODriveController objects for each motor
        self.motor0 = ODriveController(self, namespace="odrive_axis0")
        self.motor1 = ODriveController(self, namespace="odrive_axis1")

        # defines the sequence of phases for a jump
        self.phases = [
            self.update_parameters,  # should always be first, since phases depend on parameters
            self.set_axis_closed_loop_control,  # enables closed loop control if previously set to idle
            # jump phases
            self.positions_phase,
            self.poising_phase,
            # self.torque_phase,
            # self.winding_phase, # not needed for now, since we don't have a spring
            self.jumping_phase,
            self.landing_phase,
        ]

        # waits for the motors to be ready, and also sets them to closed loop control initially
        self.motor0.wait_for_axis_state()
        self.motor1.wait_for_axis_state()

        # initializes the action server for the jump action, which always accepts cancel requests
        self.jump_action = ActionServer(
            self,
            Jump,
            "jump",
            self.jump_execute_callback,
            cancel_callback=self.cancel_callback,
        )
        
    def cancel_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        """Automatically accepts cancel requests for the jump action, and sets both odrives to idle.

        Args:
            goal_handle (rclpy.action.server.ServerGoalHandle): unused parameter, since this accepts all cancel requests.

        Returns:
            CancelResponse: CancelResponse.ACCEPT
        """
        self.set_axis_idle()
        return CancelResponse.ACCEPT

    def update_parameters(self):
        """Updates all parameters for this node. Should be called at the beginning of each jump."""
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
        self.torque_pos0 = self.get_parameter("torque_pos0").value
        self.torque_pos1 = self.get_parameter("torque_pos1").value

    def angle_to_position(self, angle: float) -> float:
        """Converts an angle in degrees to a motor position in rotations.

        Args:
            angle (float): The angle in degrees

        Returns:
            float: The motor position in rotations
        """
        return angle / (self.gear_ratio * 360)

    def wait_seconds(self, seconds: float):
        """Sleeps for a given number of seconds.

        Args:
            seconds (float): The number of seconds to sleep
        """
        self.get_clock().sleep_for(Duration(seconds=seconds))

    def jump_execute_callback(
        self, goal_handle: rclpy.action.server.ServerGoalHandle
    ) -> Jump.Result:
        """Executes the jump action, phase by phase.

        Args:
            goal_handle (rclpy.action.server.ServerGoalHandle): The goal handle for the jump action

        Returns:
            Jump.Result: The result of the jump action, indicating whether it was completed
        """
        for phase in self.phases:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.set_axis_idle()
                return Jump.Result(complete=False)

            phase()
            goal_handle.publish_feedback(
                Jump.Feedback(phase=f"{phase.__name__} completed")
            )

        goal_handle.succeed()
        return Jump.Result(complete=True)

    def set_axis_idle(self):
        """Sets both ODrives to idle."""
        self.motor0.request_axis_state(AxisStates.IDLE)
        self.motor1.request_axis_state(AxisStates.IDLE)

    def set_axis_closed_loop_control(self):
        """Sets both ODrives to closed loop control."""
        self.motor0.request_axis_state(AxisStates.CLOSED_LOOP_CONTROL)
        self.motor1.request_axis_state(AxisStates.CLOSED_LOOP_CONTROL)

    def positions_phase(self):
        """Moves both linkages to their default positions."""
        self.motor0.set_position(self.normal_pos0)
        self.motor1.set_position(self.normal_pos1)
        self.wait_seconds(1)

    def poising_phase(self):
        """Moves both linkages to their minimum positions, in preparation for the jump."""
        self.motor0.set_position(self.min_pos0)
        self.motor1.set_position(self.min_pos1)
        self.wait_seconds(2)
        
    def torque_phase(self):
        self.motor0.set_torque(self.max_torque)
        self.motor1.set_torque(self.max_torque)
        
        while self.motor0.position > self.torque_pos0 and self.motor1.position < self.torque_pos1:
            self.wait_seconds(0.01)
            
        self.motor0.set_torque(self.max_torque)
        self.motor1.set_torque(self.max_torque)

    def winding_phase(self):
        """Winds up the spring for the jump. Not needed for now, since we don't have a spring."""
        self.motor1.set_torque(self.winding_torque)
        self.wait_seconds(3)

    def jumping_phase(self):
        """Executes the jump by moving both linkages to their maximum positions.
        This assumes that the filtered position control mode does not limit the torque and velocity below the motor's maximum values.
        """
        self.motor0.set_position(self.max_pos0)
        self.motor1.set_position(self.max_pos1)
        self.wait_seconds(0.3)

    def bracing_phase(self):
        """Braces the jump by setting the torque to a fixed value, which should be less than the maximum torque.
        """
        self.motor0.set_torque(self.brace_torque)
        self.motor1.set_torque(self.brace_torque)
        self.wait_seconds(1.0)

    def landing_phase(self):
        """Moves both linkages back to their default positions."""
        self.motor0.set_position(self.normal_pos0)
        self.motor1.set_position(self.normal_pos1)


def main(args=None):
    rclpy.init(args=args)
    jump_node = JumpNode()
    rclpy.spin(jump_node, executor=rclpy.executors.MultiThreadedExecutor())
    jump_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
