import tkinter as tk
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionClient
from catbot_msg.action import Jump
import rclpy.parameter
from ros2node.api import get_node_names
from .models.node_model import NodeModel
from .views.node_view import NodeView
import threading


class GUINode(Node):
    def __init__(self, root: tk.Tk):
        super().__init__("gui_node")
        self.window = root
        self.nodes: list[NodeModel] = []
        self.node_views: list[NodeView] = []
        self.goal_handle: rclpy.action.client.ClientGoalHandle = None

        self.menu = tk.Menu(self.window)
        self.window.config(menu=self.menu)
        self.menu.add_command(label="Refresh (Ctrl-R)", command=self.refresh)

        self.jump_client = ActionClient(self, Jump, "/jump")

        self.window.title(self.get_name())

        container = tk.LabelFrame(
            self.window, text="Jump Test", pady=10, padx=10
        )
        buttons = tk.Frame(container)

        self.jump = tk.Button(
            buttons,
            text="Jump (Enter)",
            bg="green",
            activebackground="lightgreen",
            fg="white",
            command=self.call_jump,
        )
        self.jump.pack(side=tk.LEFT, padx=10, pady=10)

        self.cancel = tk.Button(
            buttons,
            text="Cancel (Space)",
            bg="red",
            activebackground="pink",
            fg="white",
            command=self.cancel_jump,
        )
        self.cancel.pack(side=tk.LEFT, padx=10, pady=10)
        buttons.pack(fill=tk.X)

        self.status = tk.Text(
            container,
            height=5,
            foreground="black",
        )
        self.status.bind("<Key>", lambda e: "break")

        self.status.pack(side=tk.TOP, padx=10, pady=10, fill=tk.X)

        container.pack(fill=tk.X, padx=10, pady=10)

        self.refresh()

        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.window.bind("<Control-r>", lambda event: self.refresh())
        self.window.bind("<Return>", lambda event: self.call_jump())
        self.window.bind("<space>", lambda event: self.cancel_jump())

    def call_jump(self):
        if self.goal_handle:
            return
        threading.Thread(target=self._call_jump).start()

    def _call_jump(self):
        self.jump["state"] = tk.DISABLED
        self.window.update()
        self.jump_client.wait_for_server()
        future = self.jump_client.send_goal_async(
            Jump.Goal(), feedback_callback=self._jump_feedback
        )
        rclpy.spin_until_future_complete(self, future)
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.status.insert("1.0", f"Request: Jump Rejected\n")
            self.jump["state"] = tk.NORMAL
            return
        future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future)
        self.status.insert("1.0", f"Result: {future.result().result}\n")
        self.jump["state"] = tk.NORMAL
        self.goal_handle = None
        self.window.update()

    def _jump_feedback(self, feedback):
        self.status.insert("1.0", f"Feedback: {feedback.feedback.phase}\n")
        self.window.update()

    def cancel_jump(self):
        threading.Thread(target=self._cancel_jump).start()
            
    def _cancel_jump(self):
        if self.goal_handle:
            self.status.insert("1.0", f"Request: Jump Cancelled\n")
            self.jump["state"] = tk.NORMAL
            self.window.update()
            future = self.goal_handle.cancel_goal_async()

    def refresh(self):
        if self.goal_handle:
            return
        threading.Thread(target=self._refresh).start()
        
    def _refresh(self):
        for node_view in self.node_views:
            node_view.destroy()

        self.nodes = list(
            map(
                lambda n: NodeModel(self, n),
                filter(
                    lambda n: n.name != self.get_name(),
                    get_node_names(node=self),
                ),
            )
        )

        self.node_views = list(
            map(lambda n: NodeView(self.window, n), self.nodes)
        )

        for node_view in self.node_views:
            node_view.pack(side=tk.TOP, fill=tk.BOTH, padx=10, pady=10)

    def close(self):
        self.window.destroy()
        self.destroy_node()


def main():
    rclpy.init(args=None)
    root = tk.Tk()
    node = GUINode(root)
    root.mainloop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
