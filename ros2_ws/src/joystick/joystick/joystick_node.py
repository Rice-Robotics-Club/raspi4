import rclpy
from rclpy.node import Node
import evdev
from geometry_msgs.msg import Twist

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        
        self.declare_parameter('device_name', '8BitDo Pro 2')
        
        devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
        
        devices = list(filter(lambda d: self.get_parameter('device_name').value == d.name, devices))
        
        if len(devices) == 0:
            self.get_logger().error(f"No device found with name {self.get_parameter('device_name').value}")
            return
        
        self.device = devices[0]
        
        self.get_logger().info(f"Found device {self.device.name}")
        
        self.joystick_publisher = self.create_publisher(Twist, '/joy_vel', 10)
        
        self.joy_msg = Twist()
        
        for event in self.device.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                match event.code:
                    case evdev.ecodes.ABS_X:
                        self.joy_msg.angular.z = (127.0 - event.value) / 128.0
                    case evdev.ecodes.ABS_Z:
                        self.joy_msg.linear.x = (event.value - 127.0) / 128.0
                    case evdev.ecodes.ABS_RZ:
                        self.joy_msg.linear.y = (127.0 - event.value) / 128.0
            
            self.joystick_publisher.publish(self.joy_msg)
                
        
def main(args=None):
    rclpy.init(args=args)
    joystick_node = JoystickNode()
    rclpy.spin(joystick_node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()