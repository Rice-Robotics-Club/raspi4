import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from .controllers.odrive_controller import ODriveController
from .controllers.fk_controller import FKController
from odrive.enums import AxisState as AxisStates
from sensor_msgs.msg import Joy
import numpy as np
import typing


class JumpNode(Node):
    def __init__(self):
        super().__init__("jump_node")

        # declares all parameters for this node
        self.declare_parameter("interval", 0.01)
        self.declare_parameter("gear_ratio", 8.0)
        self.declare_parameter("max_torque", 11.0)

        self.declare_parameter("landing_pos0", -1.0)
        self.declare_parameter("landing_pos1", 0.5)

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

        starting_offset0 = 2.70526030718
        starting_offset1 = 5.84685330718

        self.fk = FKController(a1, a2, a3, a4, l1, l2)

        # initializes the ODriveController objects for each motor
        self.motor0 = ODriveController(
            self,
            namespace="odrive_axis0",
            gear_ratio=self.gear_ratio,
            angle_offset=starting_offset0,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
        )
        self.motor1 = ODriveController(
            self,
            namespace="odrive_axis1",
            gear_ratio=self.gear_ratio,
            angle_offset=starting_offset1,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
        )

        self.joy = self.create_subscription(Joy, "/joy", self._joy_callback, 10)

        # defines the sequence of phases for a jump
        self.phases = [
            (self.set_axis_idle, lambda: False),
            (
                self.update_parameters,
                lambda: True,
            ),  # should always be first, since phases depend on parameters
            (
                self.set_axis_closed_loop_control,
                lambda: True,
            ),  # enables closed loop control if previously set to idle
            (
                self.poising_phase,
                lambda: self.motor0.is_about(starting_offset0)
                or self.motor1.is_about(starting_offset1),
            ),
            (self.jumping_phase, lambda: self.motor0.angle > (5 * np.pi / 4)),
            (
                self.landing_phase,
                lambda: self.motor0.is_about(starting_offset0)
                or self.motor1.is_about(starting_offset1),
            ),
        ]

        self.current_phase = 0

        # waits for the motors to be ready, and also sets them to closed loop control initially
        self.motor0.wait_for_axis_state()
        self.motor1.wait_for_axis_state()

        self.timer = self.create_timer(self.interval, self._timer_callback)

    def next_phase(self):
        self.current_phase = (self.current_phase + 1) % len(self.phases)

    def _timer_callback(self):
        func, next = self.phases[self.current_phase]
        func()

        if next():
            self.next_phase()

    def _joy_callback(self, msg: Joy):
        if msg.buttons[1] == 1 and self.current_phase == 0:
            self.current_phase = 1
        if msg.buttons[2] == 1:
            self.current_phase = 0

    def update_parameters(self):
        """Updates all parameters for this node. Should be called at the beginning of each jump."""
        self.interval = self.get_parameter("interval").value
        self.gear_ratio = self.get_parameter("gear_ratio").value
        self.max_torque = self.get_parameter("max_torque").value
        self.landing_pos0 = self.get_parameter("landing_pos0").value
        self.min_pos0 = self.get_parameter("min_pos0").value
        self.landing_pos1 = self.get_parameter("landing_pos1").value
        self.min_pos1 = self.get_parameter("min_pos1").value

    def set_axis_idle(self):
        """Sets both ODrives to idle."""
        self.get_logger().info("setting ODrives to IDLE")
        self.motor0.request_axis_state(AxisStates.IDLE)
        self.motor1.request_axis_state(AxisStates.IDLE)

    def set_axis_closed_loop_control(self):
        """Sets both ODrives to closed loop control."""
        self.get_logger().info("setting ODrives to CLOSED_LOOP_CONTROL")
        self.motor0.request_axis_state(AxisStates.CLOSED_LOOP_CONTROL)
        self.motor1.request_axis_state(AxisStates.CLOSED_LOOP_CONTROL)

    def poising_phase(self):
        """Moves both linkages to their minimum positions, in preparation for the jump."""
        self.motor0.set_position(self.min_pos0)
        self.motor1.set_position(self.min_pos1)

    def jumping_phase(self):
        """Calculates leg jacobian based on current motor angles, and uses transpose of
        jacobian to calculate torques required to exert downward force at the foot. Then
        scales up this torque vector to maximum torque limits.
        """
        th1 = self.motor0.angle
        th2 = self.motor1.angle

        try:
            torques = (
                self.fk.jacobian(th1, th2).T @ np.array([[0], [-1]])
            ).flatten()

            torques *= self.max_torque / torques.max()

            self.motor0.set_torque(-(torques[0]))
            self.motor1.set_torque(-(torques[1]))
        except:
            self.next_phase()

    def landing_phase(self):
        """Moves both linkages back to their default positions."""
        self.motor0.set_position(self.landing_pos0)
        self.motor1.set_position(self.landing_pos1)


def main(args=None):
    rclpy.init(args=args)
    jump_node = JumpNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(jump_node, executor=executor)
    jump_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
