import rclpy
from rclpy.node import Node
from odrive_can.srv import AxisState

class Stop(Node):
    def __init__(self):
        super().__init__('stop')
        
        # Create a client for the set_axis_state service
        self.state = self.create_client(AxisState, 'request_axis_state')
        while not self.state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for AxisState service...')
        self.axis_request = AxisState.Request()
        self.axis_request.axis_requested_state = 1
        self.future = self.state.call_async(self.axis_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Result: %r' % (self.future.result().axis_state))
        
def main(args=None):
    rclpy.init(args=args)
    node = Stop()
    rclpy.spin_once(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
