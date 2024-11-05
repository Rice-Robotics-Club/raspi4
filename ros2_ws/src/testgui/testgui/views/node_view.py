import tkinter as tk
from ..models.node_model import NodeModel


class NodeView(tk.LabelFrame):
    def __init__(self, master: tk.Widget, node_model: NodeModel):
        super().__init__(master=master, text=node_model.name)

        self.node_model = node_model
        self.entries: dict[str, tk.Entry] = {}

        for name, param in node_model.parameters.items():
            tk.Label(self, text=name).pack()
            self.entries[name] = tk.Entry(self)
            self.entries[name].insert(0, str(param.data))
            self.entries[name].pack()

        tk.Button(self, text="update", command=self.update).pack()

    def update(self):
        self.node_model.set_parameters(
            {name: entry.get() for name, entry, in self.entries.items()}
        )
