import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage, ControllerStatus
from odrive.enums import AxisState as AxisStates, InputMode, ControlMode
import math


class ODriveController:
    def __init__(
        self,
        parent: Node,
        namespace: str,
        gear_ratio: float,
        angle_offset: float,  # in radians
        callback_group: rclpy.callback_groups.CallbackGroup,
    ):
        self.parent = parent
        self.namespace = namespace

        self.gear_ratio = gear_ratio
        self.angle_offset = angle_offset

        self.angle = angle_offset  # in radians
        self.velocity = 0.0  # in radians / second
        self.torque = 0.0  # in Nm

        self.axis_state_client = self.parent.create_client(
            AxisState,
            f"/{self.namespace}/request_axis_state",
            callback_group=callback_group,
        )
        self.axis_state_request = AxisState.Request()

        self.controller_status_subscription = self.parent.create_subscription(
            ControllerStatus,
            f"/{self.namespace}/controller_status",
            self._controller_status_callback,
            10,
            callback_group=callback_group,
        )

        self.control_message_publisher = self.parent.create_publisher(
            ControlMessage,
            f"/{self.namespace}/control_message",
            10,
            callback_group=callback_group,
        )
        self.control_message = ControlMessage()
        self.control_message.input_mode = InputMode.PASSTHROUGH

    def wait_for_axis_state(self):
        """
        Blocking function that waits until /request_axis_state is up, and returns
        only when the axis state has been set to closed loop control
        """
        self.axis_state_client.wait_for_service(timeout_sec=None)

    def _controller_status_callback(self, msg: ControllerStatus):
        """Updates position, velocity and torque fields on every publish of
        /controller_status

        Args:
            msg (ControllerStatus): interface provided by ODrive containing motor values
        """
        self.angle = ((
            (-msg.pos_estimate / self.gear_ratio) * math.tau
        ) + self.angle_offset) % math.tau
        self.velocity = msg.vel_estimate
        self.torque = msg.torque_estimate

        # self.parent.get_logger().info(f"{self.namespace} angle: {self.angle}")

    def request_axis_state(self, state: AxisStates):
        self.axis_state_request.axis_requested_state = int(state)
        future = self.axis_state_client.call_async(self.axis_state_request)
        rclpy.spin_until_future_complete(self.parent, future=future)

    def set_torque(self, torque: float):
        """Publishes a torque control message to the ODrive

        Args:
            torque (float): value
        """
        self.control_message.control_mode = ControlMode.TORQUE_CONTROL
        self.control_message.input_mode = InputMode.PASSTHROUGH
        self.control_message.input_torque = torque
        self.control_message_publisher.publish(self.control_message)

    def ramp_torque(self, torque: float):
        """Publishes a torque control message to the ODrive

        Args:
            torque (float): value
        """
        self.control_message.control_mode = ControlMode.TORQUE_CONTROL
        self.control_message.input_mode = InputMode.TORQUE_RAMP
        self.control_message.input_torque = torque
        self.control_message_publisher.publish(self.control_message)

    def set_velocity(self, velocity: float):
        """Publishes a velocity control message to the ODrive

        Args:
            velocity (float): value
        """
        self.control_message.control_mode = ControlMode.VELOCITY_CONTROL
        self.control_message.input_mode = InputMode.PASSTHROUGH
        self.control_message.input_vel = velocity
        self.control_message_publisher.publish(self.control_message)

    def ramp_velocity(self, velocity: float):
        """Publishes a velocity control message to the ODrive

        Args:
            velocity (float): value
        """
        self.control_message.control_mode = ControlMode.VELOCITY_CONTROL
        self.control_message.input_mode = InputMode.VEL_RAMP
        self.control_message.input_vel = velocity
        self.control_message_publisher.publish(self.control_message)

    def set_position(self, position: float):
        """Publishes a position control message to the ODrive

        Args:
            position (float): value
        """
        self.control_message.control_mode = ControlMode.POSITION_CONTROL
        self.control_message.input_mode = InputMode.POS_FILTER
        self.control_message.input_pos = position
        self.control_message_publisher.publish(self.control_message)
        
    def is_about(self, target: float):
        return (abs(self.angle - target) % math.tau) < .1
