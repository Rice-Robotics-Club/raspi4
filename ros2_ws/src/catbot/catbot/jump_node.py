import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from .controllers.odrive_controller import ODriveController
from .controllers.fk_controller import FKController
from odrive.enums import AxisState as AxisStates
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
import numpy as np
from collections.abc import Callable
import typing


class JumpNode(Node):
    def __init__(self):
        super().__init__("jump_node")

        # declares all parameters for this node
        self.interval = self.declare_parameter("interval", 0.01).value
        self.gear_ratio = self.declare_parameter("gear_ratio", 8.0).value
        self.max_torque = self.declare_parameter("max_torque", 11.0).value

        # setpoint angles in radians
        self.min_angle0 = self.declare_parameter(
            "min_angle0", 2.70526030718
        ).value
        self.min_angle1 = self.declare_parameter(
            "min_angle1", 5.84685330718
        ).value
        self.mid_angle0 = self.declare_parameter(
            "mid_angle0", 3.49065847058
        ).value
        self.mid_angle1 = self.declare_parameter(
            "mid_angle1", 5.45415422548
        ).value
        self.max_angle0 = None
        self.max_angle1 = None

        # initalizes fields corresponding to each parameter
        self.update_parameters(None)

        a1 = 0.129
        a2 = 0.080
        a3 = 0.104
        a4 = 0.180
        l1 = 0.225
        l2 = 0.159

        starting_offset0 = 2.70526030718
        starting_offset1 = 5.84685330718

        self.fk = FKController(a1, a2, a3, a4, l1, l2)

        # initializes the ODriveController objects for each motor
        self.motor0 = ODriveController(
            self,
            namespace="odrive_axis0",
            gear_ratio=self.gear_ratio,
            angle_offset=self.min_angle0,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
        )
        self.motor1 = ODriveController(
            self,
            namespace="odrive_axis1",
            gear_ratio=self.gear_ratio,
            angle_offset=self.min_angle1,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
        )

        self.joy = self.create_subscription(Joy, "/joy", self._joy_callback, 10)

        # defines the sequence of phases for a jump
        self.phases: list[Callable[[], bool]] = [
            self.set_axis_idle,
            self.update_parameters,  # should always be first, since phases depend on parameters
            self.set_axis_closed_loop_control,  # enables closed loop control if previously set to idle
            self.poising_phase,
            self.jumping_phase,
            self.landing_phase,
        ]

        self.current_phase = 0

        # waits for the motors to be ready, and also sets them to closed loop control initially
        self.motor0.wait_for_axis_state()
        self.motor1.wait_for_axis_state()

        self.timer = self.create_timer(self.interval, self._timer_callback)

    def next_phase(self):
        self.current_phase = (self.current_phase + 1) % len(self.phases)

    def _timer_callback(self):
        if self.phases[self.current_phase]():
            self.next_phase()

    def _joy_callback(self, msg: Joy):
        if msg.buttons[1] == 1 and self.current_phase == 0:
            self.current_phase = 1
        if msg.buttons[2] == 1:
            self.current_phase = 0

    def set_axis_idle(self):
        """Sets both ODrives to idle."""
        self.get_logger().info("setting ODrives to IDLE", once=True)

        self.motor0.request_axis_state(AxisStates.IDLE)
        self.motor1.request_axis_state(AxisStates.IDLE)

        return False

    def update_parameters(self):
        """Updates all parameters for this node. Should be called at the beginning of each jump."""
        self.get_logger().info("updating parameter values", once=True)

        for p in self._parameters:
            self.__setattr__(p, self.get_parameter(p).value)

        return True

    def set_axis_closed_loop_control(self):
        """Sets both ODrives to closed loop control."""
        self.get_logger().info(
            "setting ODrives to CLOSED_LOOP_CONTROL", once=True
        )

        self.motor0.request_axis_state(AxisStates.CLOSED_LOOP_CONTROL)
        self.motor1.request_axis_state(AxisStates.CLOSED_LOOP_CONTROL)

        return True

    def poising_phase(self):
        """Moves both linkages to their minimum positions, in preparation for the jump."""
        self.get_logger().info("starting poising_phase", once=True)

        self.motor0.set_position(self.min_angle0)
        self.motor1.set_position(self.min_angle1)

        return self.motor0.is_about(self.min_angle0) or self.motor1.is_about(
            self.min_angle1
        )

    def jumping_phase(self):
        """Calculates leg jacobian based on current motor angles, and uses transpose of
        jacobian to calculate torques required to exert downward force at the foot. Then
        scales up this torque vector to maximum torque limits.
        """
        self.get_logger().info("starting jumping_phase", once=True)

        th1 = self.motor0.angle
        th2 = self.motor1.angle

        try:
            torques = (
                self.fk.jacobian(th1, th2).T @ np.array([[0], [-1]])
            ).flatten()

            torques *= self.max_torque / abs(max(torques, key=abs))

            self.motor0.set_torque(-(torques[0]))
            self.motor1.set_torque(-(torques[1]))
        except:
            return True

        return self.motor0.is_about(self.max_angle0) or self.motor1.is_about(
            self.max_angle1
        )

    def landing_phase(self):
        """Moves both linkages back to their default positions."""
        self.get_logger().info("starting landing_phase", once=True)

        self.motor0.set_position(self.mid_angle0)
        self.motor1.set_position(self.mid_angle1)

        return self.motor0.is_about(self.mid_angle0) or self.motor1.is_about(
            self.mid_angle1
        )


def main(args=None):
    rclpy.init(args=args)
    jump_node = JumpNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    
    try:
        rclpy.spin(jump_node, executor=executor)
    except:
        jump_node.set_axis_idle()
        jump_node.destroy_node()
        rclpy.shutdown()

    jump_node.set_axis_idle()
    jump_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
