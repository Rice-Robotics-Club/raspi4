import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from odrive_can.msg import ControllerStatus, ControlMessage
from odrive_can.srv import AxisState

class MotorServoTestNode(Node):
    def __init__(self):
        super().__init__('motor_servo_test_node')
        
        # Create clients for the request_axis_state services for both motors
        self.axis_state0 = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        self.axis_state1 = self.create_client(AxisState, '/odrive_axis1/request_axis_state')
        
        # Wait for the services to be available
        while not self.axis_state0.wait_for_service(timeout_sec=1.0) or not self.axis_state1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for AxisState services...')
            
        # Create publishers for the motor control messages
        self.motor_control_publisher0 = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        self.motor_control_publisher1 = self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10)
        
        # Create a publisher for the servo control messages
        self.servo_angle_publisher = self.create_publisher(Float64, 'servo_angle', 10)
      
        # Create a subscription to the ODrive controller status
        self.controller_status_subscription0 = self.create_subscription(
            ControllerStatus,
            '/odrive_axis0/controller_status',
            lambda msg: self.controller_status_callback(msg, motor_id=0),
            10
        )
      
        self.controller_status_subscription1 = self.create_subscription(
            ControllerStatus,
            '/odrive_axis1/controller_status',
            lambda msg: self.controller_status_callback(msg, motor_id=1),
            10
        )

    def controller_status_callback(self, msg, motor_id):
        if motor_id == 0:
            self.curr_torque_estimate0 = msg.torque_estimate
            self.curr_pos_estimate0 = msg.pos_estimate
            # Handle motor 0 status
        elif motor_id == 1:
            self.curr_torque_estimate1 = msg.torque_estimate
            self.curr_pos_estimate1 = msg.pos_estimate
            # Handle motor 1 status
        self.get_logger().info(f'Motor {motor_id}: Torque {msg.torque_estimate}, Position {msg.pos_estimate}')
    
    def send_control_message(self, motor_id, mode, pos=0.0, vel=0.0, torque=0.0):
        """
        Send control message to specified motor.

        motor_id: ID of the motor to control (0 or 1).
        mode: Control mode (1 for torque, 2 for velocity, 3 for position).
        pos: Desired position (relevant if mode is 3).
        vel: Desired velocity (relevant if mode is 2 or for feedforward in position control).
        torque: Desired torque (relevant if mode is 1 or for feedforward in other controls).
        """
        control_msg = ControlMessage()
        control_msg.control_mode = mode
        control_msg.input_pos = float(pos)
        control_msg.input_vel = float(vel)
        control_msg.input_torque = float(torque)

        if motor_id == 0:
            self.motor_control_publisher0.publish(control_msg)
        elif motor_id == 1:
            self.motor_control_publisher1.publish(control_msg)
        else:
            self.get_logger().error(f'Invalid motor_id: {motor_id}')

    def set_servo_angle(self, angle):
      """
      Set the servo angle
  
      angle: 0-180 degress
      """
    angle_msg = Float64()
    angle_msg.data = angle
    self.servo_angle_publisher.publish(angle_msg)
    self.get_logger().info(f'Set servo angle to {angle} degrees')
  
    def set_axis_state(self, motor_id, state):
      """
      Set the state of the specified motor axis.
  
      motor_id: Motor ID (0 for the first motor, 1 for the second motor)
      state: Desired axis state
      """
      axis_request = AxisState.Request()
      axis_request.axis_requested_state = state
  
      future = self.axis_state0.call_async(axis_request) if motor_id == 0 else self.axis_state1.call_async(axis_request)
        
      rclpy.spin_until_future_complete(self, future)
    
      if future.result() is not None:
          self.get_logger().info(f'Result for motor {motor_id}: {future.result().axis_state}')
      else:
          self.get_logger().error(f'Failed to set axis state for motor {motor_id}.')

def main(args=None):
    rclpy.init(args=args)
    node = MotorServoTestNode()

    node.set_servo_angle(180)
    time.sleep(2)  # wait for other nodes to set up

    # Set both motors to CLOSED_LOOP_CONTROL
    node.set_axis_state(0, 8)  # for the first motor
    node.set_axis_state(1, 8)  # for the second motor

    # Example control commands
    node.send_control_message(motor_id=0, mode=3, pos=1.0)
    time.sleep(1)
    node.send_control_message(motor_id=1, mode=2, vel=0.5)
    time.sleep(1)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
