import rclpy
import subprocess
from rclpy.node import Node
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage
    

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        
        # Create a client for the set_axis_state service
        self.state = self.create_client(AxisState, 'request_axis_state')
        while not self.state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for AxisState service...')
        self.axis_request = AxisState.Request()
        
        # Create ControlMessage objects for both motors
        #Top motor
        self.msg0 = ControlMessage()
        self.msg0.control_mode = 2
        self.msg0.input_mode = 1
        self.msg0.input_pos = 0.0
        self.msg0.input_vel = 0.0
        self.msg0.input_torque = 0.0
        #Bottom Motor
        self.msg1 = ControlMessage()
        self.msg1.control_mode = 2
        self.msg1.input_mode = 1
        self.msg1.input_pos = 0.0
        self.msg1.input_vel = 0.0
        self.msg1.input_torque = 0.0

        # Set up publishers to the ODrives
        self.control = []
        self.control.append(self.create_publisher(ControlMessage, 'control_message_0', 10))   #Top Motor
        self.control.append(self.create_publisher(ControlMessage, 'control_message_1', 10))   #Bottom Motor\
        # Create timer
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
    node = SimpleNode()
    node.set_axis_state(8) # CLOSED_LOOP_CONTROL
    rclpy.spin(node)
    node.set_axis_state(1) # AXIS_STATE_IDLE
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()