import tkinter as tk
from ..models.node_model import NodeModel


class NodeView(tk.LabelFrame):
    def __init__(self, master: tk.Widget, node_model: NodeModel):
        super().__init__(master=master, text=node_model.name, padx=10, pady=10)

        self.node_model = node_model
        self.entries: dict[str, tk.Entry] = {}

        container = tk.Frame(self, pady=10)
        
        for name, param in node_model.parameters.items():
            frame = tk.Frame(container)
            
            tk.Label(frame, text=name, anchor=tk.W, justify=tk.LEFT).pack(fill=tk.X)
            self.entries[name] = tk.Entry(frame)
            self.entries[name].insert(0, str(param.data))
            self.entries[name].pack()
            
            frame.pack(side=tk.LEFT)
            
        container.pack(fill=tk.X)

        tk.Button(self, text="update", command=self.update).pack(fill=tk.X)

    def update(self):
        self.node_model.set_parameters(
            {name: entry.get() for name, entry, in self.entries.items()}
        )
