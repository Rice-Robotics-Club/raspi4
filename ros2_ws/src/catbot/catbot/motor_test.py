import rclpy
import subprocess
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage
    

class MotorTest(Node):
    def __init__(self):
        super().__init__('motor_test')
        
        # Create a client for the set_axis_state service
        self.state = self.create_client(AxisState, 'request_axis_state')
        while not self.state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for AxisState service...')
        self.axis_request = AxisState.Request()
        
        # Create a publisher for the 
        self.msg = ControlMessage()
        self.msg.control_mode = 2
        self.msg.input_mode = 1
        self.msg.input_pos = 0.0
        self.msg.input_vel = 2.0
        self.msg.input_torque = 0.0
        
        self.control = []
        self.control.append(self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10))
        self.control.append(self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10))
        self.timer = self.create_timer(0.1, self.control_message)
    
    def control_message(self):
        self.msg.input_vel = 2.0
        self.control[0].publish(self.msg)
        self.msg.input_vel = 1.0
        self.control[1].publish(self.msg)
    
    def set_axis_state(self, s):
        self.axis_request.axis_requested_state = s
        self.future = self.state.call_async(self.axis_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Result: %r' % (self.future.result().axis_state))
        

def main(args=None):
    rclpy.init(args=args)
    node = MotorTest()
    node.set_axis_state(8) # CLOSED_LOOP_CONTROL
    rclpy.spin(node)
    node.set_axis_state(1) # AXIS_STATE_IDLE
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
