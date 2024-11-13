import rclpy
from rclpy.node import Node
import threading

class JumpNode(Node):
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

    def start_positions_phase(self):
        self.phase = 'Positions'
        self.get_logger().info('Starting positions phase')
        self.positions_phase()

    def positions_phase(self):
        self.motor_msg1.control_mode = 3
        self.motor_msg2.control_mode = 3
        self.motor_msg1.input_pos = self.standard_angle - self.zero_angle1
        self.motor_msg2.input_pos = self.standard_angle - self.zero_angle2
        self.motor_control_publisher1.publish(self.motor_msg1)
        self.motor_control_publisher2.publish(self.motor_msg2)
        self.get_logger().info('positions phase: motors set to standard angles')
        threading.Timer(2, self.transition_to_winding).start()

    def transition_to_winding(self):
        self.phase = 'Winding'
        self.get_logger().info('Transitioning to winding phase')
        self.winding_phase()

    def winding_phase(self):
        self.motor_msg2.control_mode = 1
        self.motor_msg2.input_torque = self.poising_torque
        self.motor_control_publisher2.publish(self.motor_msg2)
        self.get_logger().info('Winding phase: bottom motor winding the spring')
        threading.Timer(3, self.transition_to_pouncing_bracing).start()

    def transition_to_pouncing_bracing(self):
        self.phase = 'Pouncing/Bracing'
        self.get_logger().info('switching to pouncing/bracing phase')
        self.pouncing_bracing_phase()

    def pouncing_bracing_phase(self):
        self.motor_msg1.control_mode = 1
        self.motor_msg2.control_mode = 1
        self.motor_msg1.input_torque = MAX_TORQUE
        self.motor_msg2.input_torque = MAX_TORQUE
        self.motor_control_publisher1.publish(self.motor_msg1)
        self.motor_control_publisher2.publish(self.motor_msg2)
        self.get_logger().info('pouncing/bracing phase: motors fully extend the spring')
        threading.Timer(2, self.transition_to_landing).start()

    def transition_to_landing(self):
        self.phase = 'Landing'
        self.get_logger().info('transition to landing phase')
        self.landing_phase()

    def landing_phase(self):
        self.motor_msg1.control_mode = 3
        self.motor_msg2.control_mode = 3
        self.motor_msg1.input_pos = self.standard_angle - self.zero_angle1
        self.motor_msg2.input_pos = self.standard_angle - self.zero_angle2
        self.motor_control_publisher1.publish(self.motor_msg1)
        self.motor_control_publisher2.publish(self.motor_msg2)
        self.get_logger().info('landing phase: Motors set to standard angle')
        self.phase = 'Positions'
        self.get_logger().info('landing completed. Resetting to Positions phase for next cycle')
