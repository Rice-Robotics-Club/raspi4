import rclpy
import rclpy.action
import rclpy.callback_groups
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
import rclpy.timer
from .controllers.odrive_controller import ODriveController
from .controllers.fk_controller import FKController
from catbot_msg.action import Jump
from odrive.enums import AxisState as AxisStates
from rclpy.action import CancelResponse
from collections.abc import Callable
import numpy as np
import typing


class JumpNode(Node):
    def __init__(self):
        super().__init__("jump_node")

        # declares all parameters for this node
        self.declare_parameter("gear_ratio", 8.0)
        self.declare_parameter("max_torque", 10.0)

        self.declare_parameter("normal_pos0", -1.0)
        self.declare_parameter("normal_pos1", 0.5)

        self.declare_parameter("min_pos0", 0.0)
        self.declare_parameter("min_pos1", 0.0)

        # initalizes fields corresponding to each parameter
        self.update_parameters()

        self.rate = self.create_rate(60)

        a1 = 0.129
        a2 = 0.080
        a3 = 0.104
        a4 = 0.180
        l1 = 0.225
        l2 = 0.159

        self.fk = FKController(a1, a2, a3, a4, l1, l2)

        # initializes the ODriveController objects for each motor
        # self.motor0 = ODriveController(
        #     self,
        #     namespace="odrive_axis0",
        #     gear_ratio=self.gear_ratio,
        #     angle_offset=2.70526030718,
        #     callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
        # )
        # self.motor1 = ODriveController(
        #     self,
        #     namespace="odrive_axis1",
        #     gear_ratio=self.gear_ratio,
        #     angle_offset=5.84685330718,
        #     callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
        # )

        # defines the sequence of phases for a jump
        self.phases: list[Callable[[Jump.Goal], None]] = [
            self.update_parameters,  # should always be first, since phases depend on parameters
            self.set_axis_closed_loop_control,  # enables closed loop control if previously set to idle
            self.poising_phase,
            self.jumping_phase,
            self.landing_phase,
            self.set_axis_idle,
        ]

        # waits for the motors to be ready, and also sets them to closed loop control initially
        # self.motor0.wait_for_axis_state()
        # self.motor1.wait_for_axis_state()

        # initializes the action server for the jump action, which always accepts cancel requests
        self.jump_action = ActionServer(
            self,
            Jump,
            "jump",
            self.jump_execute_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
        )

    def update_parameters(self, result: Jump.Goal = None):
        """Updates all parameters for this node. Should be called at the beginning of each jump."""
        self.gear_ratio = self.get_parameter("gear_ratio").value
        self.max_torque = self.get_parameter("max_torque").value
        self.normal_pos0 = self.get_parameter("normal_pos0").value
        self.min_pos0 = self.get_parameter("min_pos0").value
        self.normal_pos1 = self.get_parameter("normal_pos1").value
        self.min_pos1 = self.get_parameter("min_pos1").value

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
                self.get_logger().info(f"goal {goal_handle.goal_id} canceled")
                goal_handle.canceled()
                return Jump.Result(complete=False)

            phase(goal_handle.request)

            goal_handle.publish_feedback(
                Jump.Feedback(phase=f"{phase.__name__} completed")
            )

        goal_handle.succeed()
        return Jump.Result(complete=True)

    def set_axis_idle(self, request: Jump.Goal):
        """Sets both ODrives to idle."""
        self.get_logger().info("setting ODrives to IDLE")
        # self.motor0.request_axis_state(AxisStates.IDLE)
        # self.motor1.request_axis_state(AxisStates.IDLE)

    def set_axis_closed_loop_control(self, request: Jump.Goal):
        """Sets both ODrives to closed loop control."""
        self.get_logger().info("setting ODrives to CLOSED_LOOP_CONTROL")
        # self.motor0.request_axis_state(AxisStates.CLOSED_LOOP_CONTROL)
        # self.motor1.request_axis_state(AxisStates.CLOSED_LOOP_CONTROL)

    def poising_phase(self, request: Jump.Goal):
        """Moves both linkages to their minimum positions, in preparation for the jump."""
        # self.motor0.set_position(self.min_pos0)
        # self.motor1.set_position(self.min_pos1)
        self.wait_seconds(2)

    def jumping_phase(self, request: Jump.Goal):
        """Calculates leg jacobian based on current motor angles, and uses transpose of
        jacobian to calculate torques required to exert downward force at the foot. Then
        scales up this torque vector to maximum torque limits.
        """
        max_torque = min(self.max_torque, request.max_torque)

        i = 0.0
        while i < 3 * np.pi / 2:
            # th1 = self.motor0.angle
            # th2 = self.motor1.angle

            # try:
            #     torques = (
            #         self.fk.jacobian(th1, th2).T @ np.array([[0], [-1]])
            #     ).flatten()

            #     torques *= max_torque / torques.max()

            #     self.motor0.set_torque(-(torques[0]))
            #     self.motor1.set_torque(-(torques[1]))
            # except:
            #     break

            i += 0.1

            self.rate.sleep()

    def landing_phase(self, request: Jump.Goal):
        """Moves both linkages back to their default positions."""
        # self.motor0.set_position(self.normal_pos0)
        # self.motor1.set_position(self.normal_pos1)
        self.wait_seconds(10)


def main(args=None):
    rclpy.init(args=args)
    jump_node = JumpNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(jump_node, executor=executor)
    jump_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
