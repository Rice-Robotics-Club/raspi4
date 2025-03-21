import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.duration import Duration
from .controllers.odrive_controller import ODriveController
from .controllers.fk_controller import FKController
from odrive.enums import AxisState as AxisStates
import math
import numpy as np
import typing


class LegNode(Node):
    def __init__(self):
        super().__init__("leg_node")

        self.declare_parameter("dt", 0.01)
        self.max_torque = self.declare_parameter("max_torque", 5.0).value

        a1 = 0.129
        a2 = 0.080
        a3 = 0.104
        a4 = 0.180
        l1 = 0.225
        l2 = 0.159
        
        self.fk = FKController(a1, a2, a3, a4, l1, l2)
        
        self.motor0_max_angle = 3 * math.pi / 2

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
        
        th1 = self.motor0.angle
        th2 = self.motor1.angle
        
        try:
            torques = (self.fk.jacobian(th1, th2).T @ np.array([[0], [-1]])).flatten()
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
