import rclpy
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage, ControllerStatus

class ControlNode(Node):
    def __init__(self):
        super().__init__('simple_node')

        # Create a client for the set_axis_state service
        self.state = self.create_client(AxisState, 'request_axis_state')
        while not self.state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for AxisState service...')
        self.axis_request = AxisState.Request()

        # Set up subscribers
        self.controller_status0 = self.create_subscription(
            ControllerStatus,
            '/odrive_axis0/controller_status',
            self.controller_status_callback_axis0,
            10
        )
        self.controller_status1 = self.create_subscription(
            ControllerStatus,
            '/odrive_axis1/controller_status',
            self.controller_status_callback_axis1,
            10
        )

        # Create ControlMessage objects for both motors
        self.msg0 = ControlMessage()
        self.msg0.control_mode = 2
        self.msg0.input_mode = 1
        self.msg1 = ControlMessage()
        self.msg1.control_mode = 2
        self.msg1.input_mode = 1

        # Reference angles for the motors will be updated in the callbacks
        self.zeroangle0 = None
        self.zeroangle1 = None

        # Set up publishers to the ODrives
        self.control0 = self.create_publisher(ControlMessage, 'control_message_0', 10)
        self.control1 = self.create_publisher(ControlMessage, 'control_message_1', 10)

        # Create timer
        self.timer = self.create_timer(0.1, self.control_message)

        # Logging frequency control
        self.log_counter_axis0 = 0
        self.log_counter_axis1 = 0
        self.log_frequency = 25  # Adjust this value to change the logging frequency

    def controller_status_callback_axis0(self, msg):
        if self.zeroangle0 is None:
            self.zeroangle0 = msg.pos_estimate
        self.log_counter_axis0 += 1
        if self.log_counter_axis0 % self.log_frequency == 0:
            self.get_logger().info(f'Axis 0: pos_estimate={msg.pos_estimate}, vel_estimate={msg.vel_estimate}, torque_estimate={msg.torque_estimate}')
            self.log_counter_axis0 = 0

    def controller_status_callback_axis1(self, msg):
        if self.zeroangle1 is None:
            self.zeroangle1 = msg.pos_estimate
        self.log_counter_axis1 += 1
        if self.log_counter_axis1 % self.log_frequency == 0:
            self.get_logger().info(f'Axis 1: pos_estimate={msg.pos_estimate}, vel_estimate={msg.vel_estimate}, torque_estimate={msg.torque_estimate}')
            self.log_counter_axis1 = 0

    def control_message(self):
        if self.zeroangle0 is not None and self.zeroangle1 is not None:
            self.msg0.control_mode = 3
            self.msg1.control_mode = 3
            self.msg0.input_pos = 45 * (3.1415 / 180) - self.zeroangle0
            self.msg1.input_pos = 45 * (3.1415 / 180) - self.zeroangle1
            self.control0.publish(self.msg0)
            self.control1.publish(self.msg1)

    def set_axis_state(self, s):
        self.axis_request.axis_requested_state = s
        self.future = self.state.call_async(self.axis_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Result: %r' % (self.future.result().axis_state))

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    node.set_axis_state(8)  # CLOSED_LOOP_CONTROL
    rclpy.spin(node)
    node.set_axis_state(1)  # AXIS_STATE_IDLE
    rclpy.shutdown()

if __name__ == '__main__':
    main()
