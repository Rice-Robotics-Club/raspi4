import rclpy
from rclpy.node import Node
from odrive_can.srv import AxisState

class Stop(Node):
    def __init__(self):
        super().__init__('stop')
        
        # Create clients for the request_axis_state services
        self.axis_state0 = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        self.axis_state1 = self.create_client(AxisState, '/odrive_axis1/request_axis_state')
        
        # Wait for the services to be available
        while not self.axis_state0.wait_for_service(timeout_sec=1.0) or not self.axis_state1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for AxisState services...')
            
        # Create a request for the request_axis_state services
        self.axis_request = AxisState.Request()
        self.axis_request.axis_requested_state = 1 # AXIS_STATE_IDLE
        
        # /odrive_axis0/
        self.future = self.axis_state0.call_async(self.axis_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Stop result: %r' % (self.future.result().axis_state))
        
        # /odrive_axis1/
        self.future = self.axis_state1.call_async(self.axis_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Stop result: %r' % (self.future.result().axis_state))
        
def main(args=None):
    rclpy.init(args=args)
    node = Stop()
    rclpy.spin_once(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
