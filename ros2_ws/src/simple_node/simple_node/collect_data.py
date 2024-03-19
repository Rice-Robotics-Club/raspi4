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
      
        # Open log files for appending messages
        self.odrive_log_file = open('odrive_status_log.txt', 'a')
        self.controller_log_file = open('controller_status_log.txt', 'a')

    def odrive_status_callback(self, msg):
        log_message = f'ODrive Status: {msg}\n'
        self.get_logger().info(log_message)
        self.odrive_log_file.write(log_message)

    def controller_status_callback(self, msg):
        log_message = f'Controller Status: {msg}\n'
        self.get_logger().info(log_message)
        self.controller_log_file.write(log_message)

    def close_log_files(self):
        self.odrive_log_file.close()
        self.controller_log_file.close()

def main(args=None):
    rclpy.init(args=args)
    odrive_status_listener = ODriveStatusListener()
    rclpy.spin(odrive_status_listener)
    odrive_status_listener.close_log_files()  # Close log files before shutting down
    odrive_status_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
These log files can be accessed later. They should be in the directory where the script was run. We can also modify these two lines:

        self.odrive_log_file = open('odrive_status_log.txt', 'a')
        self.controller_log_file = open('controller_status_log.txt', 'a')

by providing an absolute path where they publish to. 
'''
