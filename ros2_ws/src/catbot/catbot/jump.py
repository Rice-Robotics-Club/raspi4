import rclpy
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage, ControllerStatus

class ControlNode(Node):
    def __init__(self):
        super().__init__('simple_node')

        self.phase = 'Proning'  # Initial phase
        self.poising_torque = 0.24  # [Nm]
        self.poising_timer = None
        self.pouncing_timer = None
        self.bracing_angle = 70 * (3.1415 / 180)  # [rad]
        self.bracing_timer = None
        self.standard_angle = 45 * (3.1415 / 180)  # [rad]

        # Create a client for the set_axis_state service
        self.state0 = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        self.state1 = self.create_client(AxisState, '/odrive_axis1/request_axis_state')
        while not self.state0.wait_for_service(timeout_sec=1.0) or not self.state0.wait_for_service(timeout_sec=1.0):
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
        self.msg0.control_mode = 2  # Set to pass-through mode initially
        self.msg0.input_mode = 1
        self.msg1 = ControlMessage()
        self.msg1.control_mode = 2  # Set to pass-through mode initially
        self.msg1.input_mode = 1

        # Reference angles for the motors will be updated in the callbacks
        self.zeroangle0 = None
        self.zeroangle1 = None

        # Set up publishers to the ODrives
        self.control0 = self.create_publisher(ControlMessage, 'control_message_0', 10)
        self.control1 = self.create_publisher(ControlMessage, 'control_message_1', 10)

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
        if self.phase == 'Proning':
            if self.zeroangle0 is not None and self.zeroangle1 is not None:
                # Set motors to position control
                self.msg0.control_mode = 3
                self.msg1.control_mode = 3
                # Set both motors to 0 position (proned position)
                self.msg0.input_pos = 0 - self.zeroangle0
                self.msg1.input_pos = 0 - self.zeroangle1
                # Publish control messages
                self.control0.publish(self.msg0)
                self.control1.publish(self.msg1)
                # Update phase to Poising
                self.phase = 'Poising'
                self.get_logger().info('Transitioning to Poising phase')
                
        elif self.phase == 'Poising':
            if not self.poising_timer:
                # TODO: Swing out the servo (implementation depends on servo control)
                # Set bottom motor to torque control
                self.msg1.control_mode = 1
                # Set bottom motor torque to poising_torque
                self.msg1.input_torque = self.poising_torque
                # Publish control message for bottom motor
                self.control1.publish(self.msg1)
                self.get_logger().info('Poising: Torque set for bottom motor')
                # Set a timer to transition to the next phase after 0.5 seconds
                self.poising_timer = self.create_timer(0.5, self.transition_to_pouncing)
            else:
                # The poising timer is already set, so just wait for it to expire
                pass
                
        elif self.phase == 'Pouncing':
            if not self.pouncing_timer:
                # Swing in the servo (implementation depends on servo control)
                # Set top motor to position control
                self.msg0.control_mode = 3
                # Set bottom motor to torque control
                self.msg1.control_mode = 1
                # Set top leg angle to 70 degrees
                self.msg0.input_pos = self.bracing_angle - self.zeroangle0
                # Publish control messages
                self.control0.publish(self.msg0)
                self.control1.publish(self.msg1)
                self.get_logger().info('Pouncing: Top motor set to 70 degrees, bottom motor set to max torque')

                # Set a timer to transition to the next phase after 0.125 seconds
                self.pouncing_timer = self.create_timer(0.125, self.transition_to_bracing)
            else:
                # The pouncing timer is already set, so just wait for it to expire
                pass
                
        elif self.phase == 'Bracing':
            if not self.bracing_timer:
                # Set both motors to position control
                self.msg0.control_mode = 3
                self.msg1.control_mode = 3
                # Set both motors to 70 degrees (bracing angle)
                self.msg0.input_pos = self.bracing_angle - self.zeroangle0
                self.msg1.input_pos = self.bracing_angle - self.zeroangle1
                # Publish control messages
                self.control0.publish(self.msg0)
                self.control1.publish(self.msg1)
                self.get_logger().info('Bracing: Both motors set to 70 degrees')

                # Set a timer to transition to the next phase after 0.125 seconds
                self.bracing_timer = self.create_timer(0.125, self.transition_to_landing)
            else:
                # The bracing timer is already set, so just wait for it to expire
                pass
                
        elif self.phase == 'Landing':
            # Set both motors to position control
            self.msg0.control_mode = 3
            self.msg1.control_mode = 3
            # Set both motors to the standard angle
            self.msg0.input_pos = self.standard_angle - self.zeroangle0
            self.msg1.input_pos = self.standard_angle - self.zeroangle1
            # Publish control messages
            self.control0.publish(self.msg0)
            self.control1.publish(self.msg1)
            self.get_logger().info('Landing: Both motors set to standard angle')

            # Reset phase to 'Proning' for the next jump
            self.phase = 'Proning'
            self.get_logger().info('Jump completed. Resetting to Proning phase for next jump')

        elif self.phase == 'Landing':
            # Set both motors to position control
            self.msg0.control_mode = 3
            self.msg1.control_mode = 3
            # Set both motors to the zeroangle position
            self.msg0.input_pos = 0 - self.zeroangle0
            self.msg1.input_pos = 0 - self.zeroangle1
            # Publish control messages
            self.control0.publish(self.msg0)
            self.control1.publish(self.msg1)
            self.get_logger().info('Landing: Both motors set to zeroangle position')
            self.get_logger().info('Jump completed.')

    def transition_to_pouncing(self):
        self.phase = 'Pouncing'
        self.get_logger().info('Transitioning to Pouncing phase')
        # Cancel the poising timer and reset it to None
        self.poising_timer.cancel()
        self.poising_timer = None
    
    def transition_to_bracing(self):
        self.phase = 'Bracing'
        self.get_logger().info('Transitioning to Bracing phase')
        # Cancel the pouncing timer and reset it to None
        self.pouncing_timer.cancel()
        self.pouncing_timer = None

    def transition_to_landing(self):
        self.phase = 'Landing'
        self.get_logger().info('Transitioning to Landing phase')
        # Cancel the bracing timer and reset it to None
        self.bracing_timer.cancel()
        self.bracing_timer = None

    
    def set_axis_state(self, s):
        self.axis_request.axis_requested_state = s
        
        # /odrive_axis0/
        self.future = self.state0.call_async(self.axis_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f'Result for odrive_axis0: {self.future.result().axis_state}')
        
        # /odrive_axis1/
        self.future = self.state1.call_async(self.axis_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f'Result for odrive_axis1: {self.future.result().axis_state}')

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    node.set_axis_state(8)  # CLOSED_LOOP_CONTROL
    rclpy.spin(node)
    node.set_axis_state(1)  # AXIS_STATE_IDLE
    rclpy.shutdown()

if __name__ == '__main__':
    main()
