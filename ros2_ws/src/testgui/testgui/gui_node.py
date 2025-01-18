import tkinter as tk
import rclpy
import rclpy.action
import rclpy.executors
from rclpy.node import Node
import rclpy.parameter
from ros2node.api import get_node_names
from .models.node_model import NodeModel
from .views.node_view import NodeView
from .extensions.jump_test import JumpTest
import threading


class GUINode(Node):
    def __init__(self, root: tk.Tk):
        super().__init__("gui_node")
        self.window = root
        self.node_models: list[NodeModel] = []
        self.node_views: list[NodeView] = []

        # Menu Bar, provides button for refreshing node list
        self.menu = tk.Menu(self.window)
        self.window.config(menu=self.menu)
        self.menu.add_command(label="Refresh (Ctrl-R)", command=self.refresh)
        
        # Sets window title
        self.window.title(self.get_name())
        
        # Initialize extensions
        self.jump_test = JumpTest(self.window, self)

        # Initialize node list
        self.refresh()

        # Bind window close event to close method
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        
        # Place keyboard shortcuts for the GUI here
        self.window.bind("<Control-r>", lambda event: self.refresh())
        
        # Place keyboard shortcuts for extensions here
        self.window.bind("<Return>", lambda event: self.jump_test.call_jump())
        self.window.bind("<space>", lambda event: self.jump_test.cancel_jump())

    def refresh(self):
        """Refreshes the node list and node views. First destroys all node views, then 
        searches for all nodes on the network, then creates a model and view for each node.
        """
        # Destroy all present node views
        for node_view in self.node_views:
            node_view.destroy()

        # Replace all node models with new ones for every node found by get_node_names
        self.node_models = list(
            map(
                lambda n: NodeModel(self, n),
                filter(
                    lambda n: n.name != self.get_name(),
                    get_node_names(node=self),
                ),
            )
        )

        # Create a view for each node model
        self.node_views = list(
            map(lambda n: NodeView(self.window, n), self.node_models)
        )

        # Add all new node views to the window
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
