import tkinter as tk
import rclpy
from rclpy.node import Node
import rclpy.parameter
from ros2node.api import get_node_names
from .models.node_model import NodeModel
from .views.node_view import NodeView


class GUINode(Node):
    def __init__(self):
        super().__init__("gui_node")
        self.window = tk.Tk()
        self.nodes: list[NodeModel] = []
        self.node_views: list[NodeView] = []

        self.window.title(self.get_name())
        tk.Button(self.window, text="REFRESH", command=self.refresh).pack(side="left")
        tk.Button(self.window, text="JUMP", command=self.jump).pack(side="left")
        tk.Button(self.window, text="STOP", command=self.stop).pack(side="left")

        self.refresh()

        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.window.mainloop()

    def stop(self):
        pass

    def jump(self):
        pass

    def refresh(self):
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
            node_view.pack()

    def close(self):
        self.window.destroy()
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GUINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
