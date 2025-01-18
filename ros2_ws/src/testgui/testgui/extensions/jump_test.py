import tkinter as tk
from rclpy.action import ActionClient
from catbot_msg.action import Jump
import threading
import rclpy
from rclpy.node import Node


class JumpTest(tk.LabelFrame):
    def __init__(self, master: tk.Widget, parent: Node):
        super().__init__(master=master, text="Jump Test", padx=0, pady=0)
        self.window = master
        self.parent = parent

        self.goal_handle: rclpy.action.client.ClientGoalHandle = None
        self.jump_client = ActionClient(self.parent, Jump, "/jump")

        buttons = tk.Frame(self)

        self.jump = tk.Button(
            buttons,
            text="Jump (Enter)",
            bg="green",
            activebackground="lightgreen",
            fg="white",
            command=self.call_jump,
        )
        self.jump.pack(side=tk.LEFT, padx=10, pady=0)

        self.cancel = tk.Button(
            buttons,
            text="Cancel (Space)",
            bg="red",
            activebackground="pink",
            fg="white",
            command=self.cancel_jump,
        )
        self.cancel.pack(side=tk.LEFT, padx=10, pady=0)
        buttons.pack(fill=tk.X)

        self.status = tk.Text(
            self,
            height=5,
            foreground="black",
        )
        self.status.bind("<Key>", lambda e: "break")

        self.status.pack(side=tk.TOP, padx=10, pady=10, fill=tk.X)

        self.pack(fill=tk.X, padx=10, pady=0)

    def call_jump(self):
        if self.goal_handle:
            return
        threading.Thread(target=self._call_jump).start()

    def _call_jump(self):
        self.jump["state"] = tk.DISABLED

        self.jump_client.wait_for_server()
        future = self.jump_client.send_goal_async(
            Jump.Goal(), feedback_callback=self._jump_feedback
        )
        rclpy.spin_until_future_complete(self.parent, future)

        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.status.insert("1.0", f"Request: Jump Rejected\n")
            self.jump["state"] = tk.NORMAL
            return

        future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.parent, future)

        self.status.insert("1.0", f"Result: {future.result().result}\n")
        self.jump["state"] = tk.NORMAL
        self.goal_handle = None

    def _jump_feedback(self, feedback):
        self.status.insert("1.0", f"Feedback: {feedback.feedback.phase}\n")

    def cancel_jump(self):
        threading.Thread(target=self._cancel_jump).start()

    def _cancel_jump(self):
        if self.goal_handle:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(lambda f: self.jump["state"] == tk.NORMAL)
