import rclpy
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
        self.control = self.create_publisher(ControlMessage, 'control_message', 10)
        self.timer = self.create_timer(0.1, self.control_message)
        
    def control_message(self):
        msg = ControlMessage()
        msg.control_mode = 2
        msg.input_mode = 1
        msg.input_pos = 0.0
        msg.input_vel = 1.0
        msg.input_torque = 0.0
        self.control.publish(msg)
    
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
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
