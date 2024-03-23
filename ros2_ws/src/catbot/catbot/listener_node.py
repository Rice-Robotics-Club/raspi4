import rclpy
from rclpy.node import Node
from odrive_can.msg import ControllerStatus

class ODriveStatusListener(Node):
    def __init__(self):
        super().__init__('odrive_status_listener')
        # Set up subscribers for axis0
        self.controller_status_subscriber_axis0 = self.create_subscription(
            ControllerStatus,
            '/odrive_axis0/controller_status',
            self.controller_status_callback_axis0,
            10
        )
        # Set up subscribers for axis1
        self.controller_status_subscriber_axis1 = self.create_subscription(
            ControllerStatus,
            '/odrive_axis1/controller_status',
            self.controller_status_callback_axis1,
            10
        )

    def controller_status_callback_axis0(self, msg):
        self.get_logger().info(f'Controller Axis0 Status: pos_estimate={msg.pos_estimate}, vel_estimate={msg.vel_estimate}, torque_estimate={msg.torque_estimate}')

    def controller_status_callback_axis1(self, msg):
        self.get_logger().info(f'Controller Axis1 Status: pos_estimate={msg.pos_estimate}, vel_estimate={msg.vel_estimate}, torque_estimate={msg.torque_estimate}')

def main(args=None):
    rclpy.init(args=args)
    odrive_status_listener = ODriveStatusListener()
    rclpy.spin(odrive_status_listener)
    odrive_status_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
