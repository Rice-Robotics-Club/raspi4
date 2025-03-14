import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
import rclpy.timer
from .controllers.odrive_controller import ODriveController
from catbot_msg.action import Jump
from odrive.enums import AxisState as AxisStates
from rclpy.action import CancelResponse
import numpy as np
import math
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

        self.a1 = 0.129
        self.a2 = 0.080
        self.a3 = 0.104
        self.a4 = 0.180
        self.l1 = 0.225
        self.l2 = 0.159

        # initializes the ODriveController objects for each motor
        self.motor0 = ODriveController(
            self,
            namespace="odrive_axis0",
            gear_ratio=self.gear_ratio,
            angle_offset=2.70526030718,
        )
        self.motor1 = ODriveController(
            self,
            namespace="odrive_axis1",
            gear_ratio=self.gear_ratio,
            angle_offset=5.84685330718,
        )

        # defines the sequence of phases for a jump
        self.phases = [
            self.update_parameters,  # should always be first, since phases depend on parameters
            self.set_axis_closed_loop_control,  # enables closed loop control if previously set to idle
            self.poising_phase,
            self.jumping_phase,
            self.landing_phase,
            self.set_axis_idle
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
        self.normal_pos0 = self.get_parameter("normal_pos0").value
        self.min_pos0 = self.get_parameter("min_pos0").value
        self.normal_pos1 = self.get_parameter("normal_pos1").value
        self.min_pos1 = self.get_parameter("min_pos1").value

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
                self.get_logger().info(f"goal {goal_handle.goal_id} canceled")
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

    def poising_phase(self):
        """Moves both linkages to their minimum positions, in preparation for the jump."""
        self.motor0.set_position(self.min_pos0)
        self.motor1.set_position(self.min_pos1)
        self.wait_seconds(2)

    def jumping_phase(self):
        rate = rclpy.timer.Rate(self.create_timer(0.01, None), self.context)
        
        while self.motor0.angle < 3 * math.pi / 2:
            torques = (self.jacobianT() @ np.array([[0], [-1]])).T
        
            ratio = self.max_torque / torques.max()
            
            torques *= ratio
            
            self.get_logger().info(f"{torques[0]} {torques[1]}")

            # self.motor0.set_torque(-(torques[0]))
            # self.motor1.set_torque(-(torques[1]))
            
            rate.sleep()
        
    def forward(self) -> np.ndarray:
        """Calculates the foot position based on the current angles of the motors.

        Returns:
            np.ndarray: 2D column vector of the foot position
        """
        a1 = self.a1
        a2 = self.a2
        a3 = self.a3
        a4 = self.a4
        l1 = self.l1
        l2 = self.l2
        th1 = self.motor0.angle
        th2 = self.motor1.angle

        return np.array(
            [
                [
                    -l1
                    * math.cos(
                        math.acos(
                            -(
                                -(a1**2)
                                + 2 * a1 * a2 * math.cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            / (
                                2
                                * a4
                                * math.sqrt(
                                    a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                                )
                            )
                        )
                        - math.asin(
                            a2
                            * math.sin(th1)
                            / math.sqrt(
                                a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                            )
                        )
                    )
                    + l2 * math.cos(th2)
                ],
                [
                    -l1
                    * math.sin(
                        math.acos(
                            -(
                                -(a1**2)
                                + 2 * a1 * a2 * math.cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            / (
                                2
                                * a4
                                * math.sqrt(
                                    a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                                )
                            )
                        )
                        - math.asin(
                            a2
                            * math.sin(th1)
                            / math.sqrt(
                                a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                            )
                        )
                    )
                    + l2 * math.sin(th2)
                ],
            ]
        )

    def jacobianT(self) -> np.ndarray:
        """Calculates the Jacobian transpose matrix of the leg based on the current angles of the motors.

        Returns:
            np.ndarray: 2x2 Jacobian matrix
        """
        a1 = self.a1
        a2 = self.a2
        a3 = self.a3
        a4 = self.a4
        l1 = self.l1
        l2 = self.l2
        th1 = self.motor0.angle
        th2 = self.motor1.angle

        # it works, and faster than the previous way
        return np.array(
            [
                [
                    l1
                    * (
                        -(
                            -a1
                            * a2**2
                            * math.sin(th1) ** 2
                            / (a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2)
                            ** (3 / 2)
                            + a2
                            * math.cos(th1)
                            / math.sqrt(
                                a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                            )
                        )
                        / math.sqrt(
                            -(a2**2)
                            * math.sin(th1) ** 2
                            / (a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2)
                            + 1
                        )
                        - (
                            a1
                            * a2
                            * math.sin(th1)
                            / (
                                a4
                                * math.sqrt(
                                    a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                                )
                            )
                            + a1
                            * a2
                            * (
                                -(a1**2)
                                + 2 * a1 * a2 * math.cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            * math.sin(th1)
                            / (
                                2
                                * a4
                                * (a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2)
                                ** (3 / 2)
                            )
                        )
                        / math.sqrt(
                            1
                            - (
                                -(a1**2)
                                + 2 * a1 * a2 * math.cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            ** 2
                            / (
                                4
                                * a4**2
                                * (a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2)
                            )
                        )
                    )
                    * math.sin(
                        math.acos(
                            -(
                                -(a1**2)
                                + 2 * a1 * a2 * math.cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            / (
                                2
                                * a4
                                * math.sqrt(
                                    a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                                )
                            )
                        )
                        - math.asin(
                            a2
                            * math.sin(th1)
                            / math.sqrt(
                                a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                            )
                        )
                    ),
                    -l1
                    * (
                        -(
                            -a1
                            * a2**2
                            * math.sin(th1) ** 2
                            / (a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2)
                            ** (3 / 2)
                            + a2
                            * math.cos(th1)
                            / math.sqrt(
                                a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                            )
                        )
                        / math.sqrt(
                            -(a2**2)
                            * math.sin(th1) ** 2
                            / (a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2)
                            + 1
                        )
                        - (
                            a1
                            * a2
                            * math.sin(th1)
                            / (
                                a4
                                * math.sqrt(
                                    a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                                )
                            )
                            + a1
                            * a2
                            * (
                                -(a1**2)
                                + 2 * a1 * a2 * math.cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            * math.sin(th1)
                            / (
                                2
                                * a4
                                * (a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2)
                                ** (3 / 2)
                            )
                        )
                        / math.sqrt(
                            1
                            - (
                                -(a1**2)
                                + 2 * a1 * a2 * math.cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            ** 2
                            / (
                                4
                                * a4**2
                                * (a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2)
                            )
                        )
                    )
                    * math.cos(
                        math.acos(
                            -(
                                -(a1**2)
                                + 2 * a1 * a2 * math.cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            / (
                                2
                                * a4
                                * math.sqrt(
                                    a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                                )
                            )
                        )
                        - math.asin(
                            a2
                            * math.sin(th1)
                            / math.sqrt(
                                a1**2 - 2 * a1 * a2 * math.cos(th1) + a2**2
                            )
                        )
                    ),
                ],
                [
                    -l2 * math.sin(th2),
                    l2 * math.cos(th2),
                ],
            ]
        )

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
