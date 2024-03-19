import rclpy
from rclpy.node import Node
from odrive_can.msg import ODriveStatus, ControllerStatus

class ODriveStatusListener(Node):
    def __init__(self):
        super().__init__('odrive_status_listener')
        self.odrive_status_subscriber = self.create_subscription(
            ODriveStatus,
            'odrive_status',
            self.odrive_status_callback,
            10
        )
        self.controller_status_subscriber = self.create_subscription(
            ControllerStatus,
            'controller_status',
            self.controller_status_callback,
            10
        )

    def odrive_status_callback(self, msg):
        self.get_logger().info(f'ODrive Status: {msg}')

    def controller_status_callback(self, msg):
        self.get_logger().info(f'Controller Status: {msg}')

def main(args=None):
    rclpy.init(args=args)
    odrive_status_listener = ODriveStatusListener()
    rclpy.spin(odrive_status_listener)
    odrive_status_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
