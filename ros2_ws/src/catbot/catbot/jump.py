import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage, ControllerStatus
from std_msgs.msg import Float64
import threading

TORQUE_SPIKE_THRESHOLD = 0.05  # Placeholder for torque spike detection threshold
MAX_TORQUE = -0.48  # Maximum torque value

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.gear_ratio = None
        self.phase = 'Init'  # Starting phase
        self.poising_torque = 0.24  # Poising torque [Nm]
        self.bracing_angle = 70 * (3.1415 / 180)  # Convert bracing angle to radians
        self.standard_angle = 45 * (3.1415 / 180)  # Convert standard angle to radians

        # AxisState service clients for both axes
        self.state_client_axis0 = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        self.state_client_axis1 = self.create_client(AxisState, '/odrive_axis1/request_axis_state')
        while not self.state_client_axis0.wait_for_service(timeout_sec=1.0) or \
              not self.state_client_axis1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for AxisState services...')
        self.axis_request_axis0 = AxisState.Request()
        self.axis_request_axis1 = AxisState.Request()

        # Set up subscribers for each motor's controller status
        self.controller_status_subscriber_axis0 = self.create_subscription(
            ControllerStatus,
            '/odrive_axis0/controller_status',
            self.controller_status_callback_axis0,
            10
        )
        self.controller_status_subscriber_axis1 = self.create_subscription(
            ControllerStatus,
            '/odrive_axis1/controller_status',
            self.controller_status_callback_axis1,
            10
        )

        self.zero_angle1 = None  # Reference angle for the top motor
        self.zero_angle2 = None  # Reference angle for the bottom motor

        # Create ControlMessage objects for the motors
        # Top Motor
        self.motor_msg1 = ControlMessage()
        self.motor_msg1.control_mode = 3 # Set to position control initially
        self.motor_msg1.input_mode = 1 # Set to pass-through mode initially
        # Bottom Motor
        self.motor_msg2 = ControlMessage()
        self.motor_msg2.control_mode = 3  # Set to position control initially
        self.motor_msg2.input_mode = 1 # Set to pass-through mode initially

        # Set up publisher to the ODrive
        self.motor_control_publisher1 = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        self.motor_control_publisher2 = self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10)

        # Set up publisher for the servo
        self.servo_publisher = self.create_publisher(Float64, 'servo_angle', 10)

    def controller_status_callback_axis0(self, msg):
        # Callback for axis0 motor status updates
        if self.zero_angle1 is None:
            self.zero_angle1 = msg.pos_estimate
            self.get_logger().info(f'Set zero_angle1 for axis 0: {self.zero_angle1}')
            # Check if both zero angles are set to proceed to the next phase
            self.check_and_update_phase()

    def controller_status_callback_axis1(self, msg):
        # Callback for axis1 motor status updates
        if self.zero_angle2 is None:
            self.zero_angle2 = msg.pos_estimate
            self.get_logger().info(f'Set zero_angle2 for axis 1: {self.zero_angle2}')
            # Check if both zero angles are set to proceed to the next phase
            self.check_and_update_phase()
            
    def check_and_update_phase(self):
        # This method checks if both motors have their zero angles set and updates the phase accordingly
        if self.zero_angle1 is not None and self.zero_angle2 is not None:
            self.get_logger().info('Starting motor initialization sequence.')
    
            # Set motors to velocity control mode for initialization
            self.motor_msg1.control_mode = 2  # Assuming '2' is the mode for velocity control
            self.motor_msg2.control_mode = 2  # Adjust mode value as needed

            # Set a positive velocity for both motors for initial movement
            self.motor_msg1.input_vel = 0.5  # Placeholder velocity
            self.motor_msg2.input_vel = 0.5  # Placeholder velocity
    
            # Publish velocity commands to start motors
            self.motor_control_publisher1.publish(self.motor_msg1)
            self.motor_control_publisher2.publish(self.motor_msg2)
    
            # Schedule to change velocity to negative after a few seconds
            threading.Timer(3, self.set_negative_velocity).start()
    
    def set_negative_velocity(self):
        # Set a negative velocity for both motors for a brief period
        self.motor_msg1.input_vel = -0.3  # Placeholder negative velocity
        self.motor_msg2.input_vel = -0.3  # Placeholder negative velocity
    
        # Publish velocity commands to reverse motors
        self.motor_control_publisher1.publish(self.motor_msg1)
        self.motor_control_publisher2.publish(self.motor_msg2)
    
        # Schedule to stop motors and transition to Proning phase after the negative velocity period
        threading.Timer(2, self.stop_motors_and_transition).start()
    
    def stop_motors_and_transition(self):
        # Stop the motors by setting velocity to 0
        self.motor_msg1.input_vel = 0
        self.motor_msg2.input_vel = 0
        self.motor_control_publisher1.publish(self.motor_msg1)
        self.motor_control_publisher2.publish(self.motor_msg2)
    
        # Update phase to Proning and log transition
        self.phase = 'Proning'
        self.get_logger().info('Initialization complete. Transitioning to Proning phase.')

    def start_proning_phase(self):
        self.phase = 'Proning'
        self.get_logger().info('Starting Proning phase')
        self.proning_phase()

    def proning_phase(self):
        # Set motors to position control and proned position
        self.motor_msg1.control_mode = 3
        self.motor_msg2.control_mode = 3
        self.motor_msg1.input_pos = 0 - self.zero_angle1
        self.motor_msg2.input_pos = 0 - self.zero_angle2
        # Publish to each motor's publisher
        self.motor_control_publisher1.publish(self.motor_msg1)
        self.motor_control_publisher2.publish(self.motor_msg2)
        # Move servo to connect the legs
        self.set_servo_angle(180)
        # Transition to the next phase after a delay
        threading.Timer(2, self.transition_to_poising).start()

    def transition_to_poising(self):
        # Transition to the Poising phase
        self.phase = 'Poising'
        self.get_logger().info('Transitioning to Poising phase')
        self.poising_phase()

    def poising_phase(self):
        # Set bottom motor to torque control for poising
        self.motor_msg2.control_mode = 1  # Torque control mode
        self.motor_msg2.input_torque = self.poising_torque
        # Publish control message specifically for the bottom motor
        self.motor_control_publisher2.publish(self.motor_msg2)
        self.get_logger().info('Poising: Torque set for the bottom motor')
        # Schedule transition to Pouncing phase after a brief delay
        threading.Timer(0.5, self.transition_to_pouncing).start()

    def transition_to_pouncing(self):
        # Transition to the Pouncing phase
        self.phase = 'Pouncing'
        self.get_logger().info('Transitioning to Pouncing phase')
        self.pouncing_phase()
        
    def pouncing_phase(self):
        # Release the servo to allow the spring to actuate the bottom leg
        self.set_servo_angle(90)
        # Set bottom motor to provide maximum torque
        self.motor_msg2.control_mode = 1  # Torque control mode
        self.motor_msg2.input_torque = MAX_TORQUE
        # Set top motor to bracing angle
        self.motor_msg1.input_pos = self.bracing_angle - self.zero_angle1  # Ensure to use zero_angle1 for top motor's reference
        # Correctly publish control messages to each motor's respective publisher
        self.motor_control_publisher1.publish(self.motor_msg1)
        self.motor_control_publisher2.publish(self.motor_msg2)
        
        self.get_logger().info('Pouncing: Motor set to max torque')
        threading.Timer(1, self.transition_to_bracing).start()

    def transition_to_bracing(self):
        self.phase = 'Bracing'
        self.get_logger().info('Transitioning to Bracing phase')
        self.bracing_phase()

    def bracing_phase(self):
        # Set bottom motor to position control
        self.motor_msg2.control_mode = 3
        # Set motor to bracing angle
        self.motor_msg2.input_pos = self.bracing_angle - self.zero_angle2
        # Publish control message for the motor
        self.motor_control_publisher2.publish(self.motor_msg2)
        self.get_logger().info('Bracing: Positions set to bracing angles')
        # Schedule transition to Landing phase
        threading.Timer(1, self.transition_to_landing).start() 

    def transition_to_landing(self):
        self.phase = 'Landing'
        self.get_logger().info('Transitioning to Landing phase')
        self.landing_phase()
    
    def landing_phase(self):
        # Set motors to position control
        self.motor_msg1.control_mode = 3
        self.motor_msg2.control_mode = 3
        # Set motors to standard angle
        self.motor_msg1.input_pos = self.standard_angle - self.zero_angle1
        self.motor_msg2.input_pos = self.standard_angle - self.zero_angle2
        # Publish control messages for both motors
        self.motor_control_publisher1.publish(self.motor_msg1)
        self.motor_control_publisher2.publish(self.motor_msg2)
        self.get_logger().info('Landing: Motors set to standard angle')
        # Assuming immediate reset to Proning for the next cycle
        self.phase = 'Proning'
        self.get_logger().info('Jump completed. Resetting to Proning phase for next jump')

    def set_servo_angle(self, angle):
        angle_msg = Float64()
        angle_msg.data = angle
        self.servo_publisher.publish(angle_msg)
        
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    control_node = ControlNode()

    # Use a MultiThreadedExecutor to manage the node's callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(control_node)
    
    try:
        executor.spin()  # Keep node alive and responsive
    except KeyboardInterrupt:
        # Handle Ctrl+C 
        pass
        
    finally:
        # Cleanup before exiting
        executor.shutdown()
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
