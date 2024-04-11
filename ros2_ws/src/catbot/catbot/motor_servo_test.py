import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from odrive_can.msg import ControllerStatus, ControlMessage
from odrive_can.srv import AxisState

TORQUE_SPIKE_THRESHOLD = 0.05  # Need to find the actual working threshold for torque spike detection
MAX_TORQUE_NEGATIVE = -0.05 #-0.48 TEST VAL
TEST_TORQUE = 0.1
SERVO_MOVE_TIME = 1.0  # Time in seconds for the servo to move to 180 degrees
TORQUE_RAMP_TIME = 1.0  # Time in seconds for the torque to ramp up
NANOSEC_TO_SEC = 1000000000
LEG_TO_MOTOR_RATIO = 30 # 1 motor rotation = 1/30 leg rotation

# this node will init, spin until testing process is complete (leg jumps), then shutdown
class MotorServoTestNode(Node):
    def __init__(self):
        super().__init__('motor_servo_test_node')
        
        # Set initial state
        self.curr_torque_estimate = 0
        self.curr_pos_estimate = 0 # note this may not be true on boot up!

        # states during jump test
        self.started = False
        self.curr_loops = 0

        # Create clients for the request_axis_state services
        self.axis_state0 = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        
        # Wait for the services to be available
        while not self.axis_state0.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for AxisState services...')
            
        # Create a request for the request_axis_state services
        self.axis_request = AxisState.Request()
        
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

        self.set_desired_states_timer = self.create_timer(1, self.set_desired_states)

    def controller_status_callback(self, msg):
        self.curr_torque_estimate = msg.torque_estimate
        self.curr_pos_estimate = msg.pos_estimate

    def set_desired_states(self):
        self.curr_loops = self.curr_loops + 1

        # safety measure
        if self.curr_pos_estimate < (- (LEG_TO_MOTOR_RATIO) / 4.0): 
            # once leg is released, leg will rotate past 90 degrees - once that happens, set to 90
            self.send_motor_pos((- (LEG_TO_MOTOR_RATIO) / 4.0))

        # self.send_motor_pos(float(self.curr_loops / 2))
        
        if self.curr_loops < 3:
            self.set_servo_angle(180)
        elif self.curr_loops < 5:
            self.send_motor_torque(MAX_TORQUE_NEGATIVE)
        elif self.curr_loops < 7:
            # last stage of test process
            self.set_servo_angle(90)
        else:
            self.send_motor_pos((- (LEG_TO_MOTOR_RATIO) / 4.0))

    def send_motor_pos(self, pos):
        control_msg = ControlMessage()
        control_msg.input_mode = 1
        control_msg.control_mode = 3  # Control mode for pos
        control_msg.input_pos = float(pos)
        control_msg.input_vel = 0.0 # feedforward
        control_msg.input_torque = 0.0 # feedforward
        self.motor_control_publisher.publish(control_msg)
        self.get_logger().info('sent pos: ' + str(pos))


    def send_motor_torque(self, torque : float):
        control_msg = ControlMessage()
        control_msg.input_mode = 1
        control_msg.control_mode = 1  # Control mode for torque
        control_msg.input_pos = 0.0
        control_msg.input_vel = 0.0
        # control_msg.input_vel = torque / abs(torque)
        control_msg.input_torque = torque # feedforward
        self.motor_control_publisher.publish(control_msg)

    def set_servo_angle(self, angle):
        angle_msg = Float64()
        angle_msg.data = float(angle)
        self.servo_angle_publisher.publish(angle_msg)
    
    def set_axis_state(self, s):
        self.axis_request.axis_requested_state = s
        
        # /odrive_axis0/
        self.future = self.axis_state0.call_async(self.axis_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Result: %r' % (self.future.result().axis_state))

def main(args=None):
    rclpy.init(args=args)
    node = MotorServoTestNode()
    node.set_servo_angle(180)
    time.sleep(2) # wait for other nodes to set up
    node.set_axis_state(8) # CLOSED_LOOP_CONTROL
    node.set_servo_angle(0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
