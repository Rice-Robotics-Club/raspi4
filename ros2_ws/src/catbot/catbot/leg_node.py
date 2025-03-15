import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from .controllers.odrive_controller import ODriveController
from catbot_msg.action import Jump
from odrive.enums import AxisState as AxisStates
from rclpy.action import CancelResponse
import time
import typing
import math
import numpy as np


class LegNode(Node):
    def __init__(self):
        super().__init__("jump_node")

        self.declare_parameter("dt", 0.01)
        self.max_torque = self.declare_parameter("max_torque", 5.0).value

        self.a1 = 0.129
        self.a2 = 0.080
        self.a3 = 0.104
        self.a4 = 0.180
        self.l1 = 0.225
        self.l2 = 0.159
        
        self.motor0_max_angle = 3 * math.pi / 2

        self.p = 2.0
        self.target = np.array([[-0.03528624], [-0.05729014]])

        # initializes the ODriveController objects for each motor
        self.motor0 = ODriveController(
            self,
            namespace="odrive_axis0",
            gear_ratio=8.0,
            angle_offset=2.70526030718,
            callback_group=self.default_callback_group
        )
        self.motor1 = ODriveController(
            self,
            namespace="odrive_axis1",
            gear_ratio=8.0,
            angle_offset=5.84685330718,
            callback_group=self.default_callback_group
        )

        # waits for the motors to be ready, and also sets them to closed loop control initially
        self.motor0.wait_for_axis_state()
        self.motor1.wait_for_axis_state()

        self.timer = self.create_timer(self.get_parameter("dt").value, self._timer_callback)

    def _timer_callback(self):
        """Finds difference between target position and current position, and uses this
        foot force input to leg jacobian to calculate torques to move foot towards target.
        """
        
        torques = np.array([0, 0])
        
        try:
            torques = (self.jacobian().T @ np.array([[0], [-1]])).flatten()
            ratio = self.max_torque / torques.max()
            torques = [t * ratio for t in torques]
        except Exception:
            pass
        finally:
            if self.motor0.angle < self.motor0_max_angle:
                self.motor0.set_torque(-(torques[0]))
                self.motor1.set_torque(-(torques[1]))
            else:
                self.motor0.set_torque(0.0)
                self.motor1.set_torque(0.0)
                self.set_axis_idle()
                self.timer.cancel()


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

    def jacobian(self) -> np.ndarray:
        """Calculates the Jacobian matrix of the leg based on the current angles of the motors.

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
                    -l2 * math.sin(th2),
                ],
                [
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
                    l2 * math.cos(th2),
                ],
            ]
        )

    def wait_seconds(self, seconds: float):
        self.get_clock().sleep_for(Duration(seconds=seconds))

    def set_axis_idle(self):
        """Sets both ODrives to idle."""
        self.motor0.request_axis_state(AxisStates.IDLE)
        self.motor1.request_axis_state(AxisStates.IDLE)


def main(args=None):
    rclpy.init(args=args)
    leg_node = LegNode()
    rclpy.spin(leg_node, executor=rclpy.executors.MultiThreadedExecutor())
    leg_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
