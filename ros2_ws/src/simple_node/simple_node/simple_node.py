import rclpy
from rclpy.node import Node
from odrive_can.msgs import ControllerStatus
from odrive.enums import ProcedureResult

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Simple Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
