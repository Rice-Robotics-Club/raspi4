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

        self.odrive_log_file_axis0 = open('odrive_status_log_axis0.txt', 'a')
        self.controller_log_file_axis0 = open('controller_status_log_axis0.txt', 'a')
        self.odrive_log_file_axis1 = open('odrive_status_log_axis1.txt', 'a')
        self.controller_log_file_axis1 = open('controller_status_log_axis1.txt', 'a')

    def odrive_status_callback_axis0(self, msg):
        log_message = f'ODrive Axis0 Status: {msg}\n'
        self.get_logger().info(log_message)
        self.odrive_log_file_axis0.write(log_message)

    def controller_status_callback_axis0(self, msg):
        log_message = f'Controller Axis0 Status: {msg}\n'
        self.get_logger().info(log_message)
        self.controller_log_file_axis0.write(log_message)

    def odrive_status_callback_axis1(self, msg):
        log_message = f'ODrive Axis1 Status: {msg}\n'
        self.get_logger().info(log_message)
        self.odrive_log_file_axis1.write(log_message)

    def controller_status_callback_axis1(self, msg):
        log_message = f'Controller Axis1 Status: {msg}\n'
        self.get_logger().info(log_message)
        self.controller_log_file_axis1.write(log_message)

    def close_log_files(self):
        self.odrive_log_file_axis0.close()
        self.controller_log_file_axis0.close()
        self.odrive_log_file_axis1.close()
        self.controller_log_file_axis1.close()

def main(args=None):
    rclpy.init(args=args)
    odrive_status_listener = ODriveStatusListener()
    rclpy.spin(odrive_status_listener)
    odrive_status_listener.close_log_files()
    odrive_status_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
