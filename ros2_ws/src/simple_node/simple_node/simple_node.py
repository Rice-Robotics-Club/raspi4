import rclpy
from rclpy.node import Node
def main(args=None):
    rclpy.init(args=args)
    node = Node('my_node_name')
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
