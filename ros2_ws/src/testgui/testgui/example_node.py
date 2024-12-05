import rclpy
import rclpy.action
import rclpy.duration
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Float64
from catbot_msg.action import Jump
from rclpy.action import ActionServer, GoalResponse, CancelResponse


class ExampleNode(Node):
    def __init__(self):
        super().__init__("example_node")
        self.parameter = self.declare_parameter("parameter", 0.1)
        ActionServer(
            self,
            Jump,
            "/jump",
            execute_callback=self.jump,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
        )
        self.create_timer(
            1,
            lambda: self.get_logger().info(
                f"parameter {self.get_parameter('parameter').value}"
            ),
        )

        self.phases = [
            lambda: self.get_logger().info("Phase 1"),
            lambda: self.get_logger().info("Phase 2"),
            lambda: self.get_logger().info("Phase 3"),
        ]

    def jump(self, goal_handle: rclpy.action.server.ServerGoalHandle) -> Jump.Result:
        for phase in self.phases:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Jump.Result(complete=False)
            
            phase()
            goal_handle.publish_feedback(
                Jump.Feedback(phase="Phase completed")
            )
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))
                
        goal_handle.succeed()
        return Jump.Result(complete=True)


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node, rclpy.executors.MultiThreadedExecutor())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
