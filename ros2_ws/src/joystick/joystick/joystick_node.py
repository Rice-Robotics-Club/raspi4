import rclpy
from rclpy.node import Node
import evdev
from geometry_msgs.msg import Twist

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        
        self.declare_parameter('device_name', '8BitDo Pro 2')
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.joystick_publisher = self.create_publisher(Twist, '/joy_vel', 10)
        self.joy_msg = Twist()
        
        self.set_device(self.get_parameter('device_name').value)
            
    def parameter_callback(self, params: list[rclpy.parameter.Parameter]) -> None:
        for param in params:
            if param.name == 'device_name':
                self.set_device(param.value)
        
    def set_device(self, name: str) -> None:
        devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
        
        devices = list(filter(lambda d: name == d.name, devices))
        
        if len(devices) == 0:
            self.get_logger().error(f"No device found with name {name}")
            return
        
        self.get_logger().info(f"Found device {name}")
        self.device = devices[0]
        self.joystick_loop()
        
    def joystick_loop(self) -> None:
        for event in self.device.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                match event.code:
                    case 0:
                        self.joy_msg.angular.z = (32767.0 -event.value) / 32768.0
                    case 3:
                        self.joy_msg.linear.x = (event.value - 32767.0) / 32768.0
                    case 4:
                        self.joy_msg.linear.y = (32767.0-event.value) / 32768.0
            
            self.joystick_publisher.publish(self.joy_msg)
                
        
def main(args=None):
    rclpy.init(args=args)
    joystick_node = JoystickNode()
    rclpy.spin(joystick_node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()