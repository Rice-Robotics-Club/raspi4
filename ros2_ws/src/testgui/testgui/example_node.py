import rclpy
import rclpy.action
import rclpy.duration
import rclpy.executors
from rclpy.node import Node
from catbot_msg.action import Jump
from rclpy.action import ActionServer, CancelResponse


class ExampleNode(Node):
    def __init__(self):
        super().__init__("example_node")
        self.declare_parameter("parameter", 0.1)

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
        
        ActionServer(
            self,
            Jump,
            "/jump",
            execute_callback=self.jump_execute_callback,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
        )

    def jump_execute_callback(
        self, goal_handle: rclpy.action.server.ServerGoalHandle
    ) -> Jump.Result:
        """Executes the jump action, phase by phase.

        Args:
            goal_handle (rclpy.action.server.ServerGoalHandle): Goal handle for the jump action

        Returns:
            Jump.Result: Result of the jump action, indicating whether it was completed
        """
        for phase in self.phases:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Jump.Result(complete=False)

            phase()
            goal_handle.publish_feedback(Jump.Feedback(phase="Phase completed"))
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

        goal_handle.succeed()
        return Jump.Result(complete=True)


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node, rclpy.executors.MultiThreadedExecutor()) # Needed to handle cancellation callbacks alongside execute callback
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
