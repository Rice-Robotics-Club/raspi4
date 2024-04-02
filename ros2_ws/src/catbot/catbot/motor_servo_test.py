import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from odrive_can.msg import ControllerStatus, ControlMessage

TORQUE_SPIKE_THRESHOLD = 0.05  # Need to find the actual working threshold for torque spike detection
MAX_TORQUE = -0.48
SERVO_MOVE_TIME = 1.0  # Time in seconds for the servo to move to 180 degrees
TORQUE_RAMP_TIME = 1.0  # Time in seconds for the torque to ramp up

# this node will init, spin until testing process is complete (leg jumps), then shutdown
class MotorServoTestNode(Node):
    def __init__(self):
        super().__init__('motor_servo_test_node')
        
        # Set initial state
        self.max_torque_reached = False
        self.servo_timer = None
        self.torque_timer = None
        
        # Create a publisher for the motor control messages
        self.motor_control_publisher = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        
        # Create a publisher for the servo control messages
        self.servo_angle_publisher = self.create_publisher(Float64, 'servo_angle', 10)
        
        # Create a subscription to the ODrive controller status
        self.controller_status_subscription = self.create_subscription(
            ControllerStatus,
            '/odrive_axis0/controller_status',
            self.controller_status_callback,
            10
        )

        # Start the motor with a constant negative velocity
        self.send_motor_velocity(-2.0) 

    def controller_status_callback(self, msg):
        if not self.max_torque_reached:
            if msg.torque_estimate > TORQUE_SPIKE_THRESHOLD:
                # Set motor velocity to zero and schedule servo to move to 180 degrees
                self.send_motor_velocity(0.0)
                self.cancel_timers()  # Cancel any existing timers
                self.servo_timer = self.create_timer(SERVO_MOVE_TIME, self.move_servo_to_180)
            elif msg.torque_estimate >= MAX_TORQUE:
                # Upon reaching max torque, move servo to 90 degrees
                self.max_torque_reached = True
                self.cancel_timers()  # Cancel any existing timers
                self.torque_timer = self.create_timer(TORQUE_RAMP_TIME, self.move_servo_to_90)

    def move_servo_to_180(self):
        # Set servo to 180 degrees and schedule torque ramp
        self.set_servo_angle(180)
        self.cancel_timers()  # Cancel any existing timers
        self.torque_timer = self.create_timer(TORQUE_RAMP_TIME, self.ramp_up_torque)

    def ramp_up_torque(self):
        # Set motor to max torque
        self.send_motor_max_torque(MAX_TORQUE)

    def move_servo_to_90(self):
        # Set servo to 90 degrees
        self.set_servo_angle(90)

    def cancel_timers(self):
        # Cancel the servo timer if it exists
        if self.servo_timer:
            self.servo_timer.cancel()
            self.servo_timer = None
        # Cancel the torque timer if it exists
        if self.torque_timer:
            self.torque_timer.cancel()
            self.torque_timer = None

    def send_motor_velocity(self, velocity):
        control_msg = ControlMessage()
        control_msg.control_mode = 2  # Control mode for velocity
        control_msg.input_vel = velocity
        self.motor_control_publisher.publish(control_msg)

    def send_motor_max_torque(self, torque):
        control_msg = ControlMessage()
        control_msg.control_mode = 3  # Control mode for torque
        control_msg.input_torque = torque
        self.motor_control_publisher.publish(control_msg)

    def set_servo_angle(self, angle):
        angle_msg = Float64()
        angle_msg.data = angle
        self.servo_angle_publisher.publish(angle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorServoTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
