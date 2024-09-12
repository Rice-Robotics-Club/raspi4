import rclpy
from rclpy.node import Node
from gpiozero import AngularServo
import time

class ServoTestNode(Node):
	def __init__(self):
		super().__init__('servo_test')

		self.gpio_pin = 18
		self.set_angle = 0
		self.timer = self.create_timer(1.0, self.timer_callback)

		self.servo = AngularServo(self.gpio_pin, min_angle=0, max_angle=135)
		self.set_angle = 0
		self.servo.angle = 0

	def timer_callback(self):
		# increment servo angle
		if self.set_angle >= 135:
			self.servo.angle = 0
			self.set_angle = 0
		else:
			self.servo.angle = self.set_angle
			self.set_angle += 20

def main(args=None):
	rclpy.init(args=args)
	servo_test_node = ServoTestNode()
	# try:
	rclpy.spin(servo_test_node)
	# except:
	# 	servo_test_node.get_logger().log(message="oof", severity=40)

	rclpy.shutdown()

if __name__ == '__main__':
	main()
