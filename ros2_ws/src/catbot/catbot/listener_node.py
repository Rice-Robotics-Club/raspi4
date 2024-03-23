import rclpy
from rclpy.node import Node
from odrive_can.msg import ODriveStatus, ControllerStatus

class ODriveStatusListener(Node):
    def __init__(self):
        super().__init__('odrive_status_listener')
        # Set up subscribers for axis0
        self.odrive_status_subscriber_axis0 = self.create_subscription(
            ODriveStatus,
            '/odrive_axis0/odrive_status',
            self.odrive_status_callback_axis0,
            10
        )
        self.controller_status_subscriber_axis0 = self.create_subscription(
            ControllerStatus,
            '/odrive_axis0/controller_status',
            self.controller_status_callback_axis0,
            10
        )
        # Set up subscribers for axis1
        self.odrive_status_subscriber_axis1 = self.create_subscription(
            ODriveStatus,
            '/odrive_axis1/odrive_status',
            self.odrive_status_callback_axis1,
            10
        )
        self.controller_status_subscriber_axis1 = self.create_subscription(
            ControllerStatus,
            '/odrive_axis1/controller_status',
            self.controller_status_callback_axis1,
            10
        )

    def odrive_status_callback_axis0(self, msg):
        self.get_logger().info(f'ODrive Axis0 Status: {msg}')

    def controller_status_callback_axis0(self, msg):
        self.get_logger().info(f'Controller Axis0 Status: {msg}')

    def odrive_status_callback_axis1(self, msg):
        self.get_logger().info(f'ODrive Axis1 Status: {msg}')

    def controller_status_callback_axis1(self, msg):
        self.get_logger().info(f'Controller Axis1 Status: {msg}')

def main(args=None):
    rclpy.init(args=args)
    odrive_status_listener = ODriveStatusListener()
    rclpy.spin(odrive_status_listener)
    odrive_status_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
