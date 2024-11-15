import rclpy
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage, ControllerStatus

class ODriveController:
    def __init__(self, parent: Node, namespace: str):
        self.parent = parent
        self.namespace = namespace
        
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        
        self.axis_state_client = self.parent.create_client(AxisState, f'/{self.namespace}/request_axis_state')
        self.axis_state_request = AxisState.Request()
        
        self.controller_status_subscription = self.parent.create_subscription(
            ControllerStatus,
            f'/{self.namespace}/controller_status',
            self._controller_status_callback,
            10
        )
        
        self.control_message_publisher = self.parent.create_publisher(ControlMessage, f'/{self.namespace}/control_message', 10)
        self.control_message = ControlMessage()
        self.control_message.control_mode = 3
        self.control_message.input_mode = 1
        
    def wait_for_axis_state(self):
        self.axis_state_client.wait_for_service(timeout_sec=None)
        self.axis_state_request.axis_requested_state = 8
        future = self.axis_state_client.call_async(self.axis_state_request)
        rclpy.spin_until_future_complete(self.parent, future)
    
    def _controller_status_callback(self, msg: ControllerStatus):
        self.position = msg.pos_estimate
        self.velocity = msg.vel_estimate
        self.torque = msg.torque_estimate
        
    def set_torque(self, torque: float):
        self.control_message.control_mode = 1
        self.control_message.input_mode = 1
        self.control_message.input_torque = torque
        self.control_message_publisher.publish(self.control_message)
        
    def set_velocity(self, velocity: float):
        self.control_message.control_mode = 2
        self.control_message.input_mode = 1
        self.control_message.input_vel = velocity
        self.control_message_publisher.publish(self.control_message)
        
    def set_position(self, position: float):
        self.control_message.control_mode = 3
        self.control_message.input_mode = 1
        self.control_message.input_pos = position
        self.control_message_publisher.publish(self.control_message)
        
        