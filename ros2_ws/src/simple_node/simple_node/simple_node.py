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
        
        # Create a publisher for the 
        self.control = []
        self.control.append(self.create_publisher(ControlMessage, 'control_message_0', 10))
        self.control.append(self.create_publisher(ControlMessage, 'control_message_1', 10))
        self.timer = self.create_timer(0.1, self.control_message)
    
    def control_message(self, msg, node_id=0):
        self.control[node_id].publish(msg)
    
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
    
    message = ControlMessage()
    message.control_mode = 2
    message.input_mode = 1
    message.input_pos = 0.0
    message.input_vel = 2.0
    message.input_torque = 0.0
    node.control_message(message, 0)
    
    message = ControlMessage()
    message.control_mode = 2
    message.input_mode = 1
    message.input_pos = 0.0
    message.input_vel = 1.0
    message.input_torque = 0.0
    node.control_message(message, 1)
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
