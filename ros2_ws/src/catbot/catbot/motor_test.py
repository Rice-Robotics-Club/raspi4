import rclpy
import subprocess
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage
    

class MotorTest(Node):
    def __init__(self):
        super().__init__('motor_test')
        
        # Create clients for the request_axis_state services
        self.axis_state = []
        self.axis_state[0] = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        self.axis_state[1] = self.create_client(AxisState, '/odrive_axis1/request_axis_state')
        
        # Wait for the services to be available
        while not self.axis_state_0.wait_for_service(timeout_sec=1.0) or not self.axis_state_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for AxisState services...')
            
        # Create a request for the request_axis_state services
        self.axis_request = AxisState.Request()
        
        # Create a message for the control_message topics
        self.control_msg = ControlMessage()
        self.control_msg.control_mode = 2
        self.control_msg.input_mode = 1
        self.control_msg.input_pos = 0.0
        self.control_msg.input_vel = 2.0
        self.control_msg.input_torque = 0.0
        
        # Create publishers for the control_message topics
        self.control = []
        self.control[0] = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        self.control[1] = self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10)
        self.timer = self.create_timer(0.1, self.control_message)
    
    def control_message(self):
        # /odrive_axis0/
        self.control_msg.input_vel = 2.0
        self.control[0].publish(self.control_msg)
        
        # /odrive_axis1/
        self.control_msg.input_vel = 1.0
        self.control[1].publish(self.control_msg)
    
    def set_axis_state(self, s):
        self.axis_request.axis_requested_state = s
        
        # /odrive_axis0/
        self.future = self.axis_state_0.call_async(self.axis_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Result: %r' % (self.future.result().axis_state))
        
        # /odrive_axis1/
        self.future = self.axis_state_1.call_async(self.axis_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Result: %r' % (self.future.result().axis_state))
        

def main(args=None):
    rclpy.init(args=args)
    node = MotorTest()
    node.set_axis_state(8) # CLOSED_LOOP_CONTROL
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
