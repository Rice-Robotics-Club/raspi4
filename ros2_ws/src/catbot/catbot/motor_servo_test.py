import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from odrive_can.msg import ControllerStatus, ControlMessage

TORQUE_SPIKE_THRESHOLD = 0.05  # Need to find actual working threshold for torque spike detection
MAX_TORQUE = -0.48

class MotorServoTestNode(Node):
    def __init__(self):
        super().__init__('motor_servo_test_node')
        
        # Set initial state
        self.max_torque_reached = False
        
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
                # Set motor velocity to zero
                self.send_motor_velocity(0.0)
                # Set servo to 180 degrees
                self.set_servo_angle(180)
            elif msg.torque_estimate >= MAX_TORQUE:
                # Set servo to 90 degrees
                self.set_servo_angle(90)
                self.max_torque_reached = True
                # Set motor to max torque
                self.send_motor_max_torque(MAX_TORQUE)

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
