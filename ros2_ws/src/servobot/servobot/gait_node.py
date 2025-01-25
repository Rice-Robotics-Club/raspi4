from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import rclpy
import math
import typing

class GaitNode(Node):
    def __init__(self):
        super().__init__("gait_node")

        self.swing_height = self.declare_parameter("swing_height", 1.0).value # inches
        self.swing_length = self.declare_parameter("swing_length", 1.0).value # inches
        self.swing_duty = self.declare_parameter("swing_duty", 0.5).value # 0.5 is 50% duty cycle
        self.gait_period = self.declare_parameter("gait_period", 1.0).value # seconds
        self.leg_phase_offsets = self.declare_parameter("leg_phase_offsets", [0.0, 0.2, 0.5, 0.7]).value # [0.0, 1.0]
        self.timer_interval = self.declare_parameter("timer_interval", 0.01).value # seconds

        self.leg_positions = self.create_publisher(
            Float64MultiArray, "/leg_positions", 10
        )
        
        # self.motion_cmd = self.create_subscription(
        #     Float64MultiArray, "/motion_cmd", self.motion_cmd_callback, 10
        # )

        self.timer = self.create_timer(self.timer_interval, self.timer_callback)

        self.msg = Float64MultiArray()
        self.msg.data = []
        self.t = 0
        
    def f(self, leg: int, t: float, vel: tuple[float, float], origin: tuple[float, float, float]) -> tuple[float, float, float]:
        t_leg = (t + self.leg_phase_offsets[leg] * self.gait_period) % self.gait_period
        swing_time = self.gait_period * self.swing_duty
        
        if t_leg < swing_time / 2:
            h = t_leg / (swing_time / 2)
        elif t_leg < swing_time:
            h = 1 - (t_leg - swing_time / 2) / (swing_time / 2)
        else:
            h = 0
            
        z = self.swing_height * h
        
        if t_leg < swing_time:
            l = -math.cos(math.pi * t_leg / swing_time)
        else:
            l = math.cos(math.pi * (t_leg - swing_time) / (self.gait_period - swing_time))
            
        amplitude = math.sqrt(vel[0] ** 2 + vel[1] ** 2)
            
        x = l * self.swing_length * vel[0] / amplitude
        y = l * self.swing_length * vel[1] / amplitude
        
        return origin[0] + x, origin[1] + y, origin[2] + z
        

    def timer_callback(self) -> None:
        positions = []
        for i in range(4):
            positions += list(self.f(i, self.t, (0, 1), (0, 0, -5.0))) 
    
        self.msg.data = positions
        self.leg_positions.publish(self.msg)
        self.get_logger().info(f"Published: {self.msg.data}")
        
        self.t += self.timer_interval
        
def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()