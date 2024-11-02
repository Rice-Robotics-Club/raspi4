import tkinter as tk
import rclpy
from rclpy.node import Node
import rclpy.parameter
from ros2node.api import get_node_names, NodeName
from rcl_interfaces.srv import SetParameters, ListParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

parameter_types = [
    bool,
    int,
    float,
    str,
    bytes,
    list[bool],
    list[int],
    list[float],
    list[str],
]


def get_value(pv: ParameterValue):
    match pv.type:
        case 0:
            return pv.bool_value
        case 1:
            return pv.integer_value
        case 2:
            return pv.double_value
        case 3:
            return pv.string_value
        case 4:
            return pv.byte_array_value
        case 5:
            return pv.bool_array_value
        case 6:
            return pv.integer_array_value
        case 7:
            return pv.double_array_value
        case 8:
            return pv.string_array_value
        
def set_value(pv: ParameterValue, value):
    match pv.type:
        case 0:
            pv.bool_value = value
        case 1:
            pv.integer_value = value
        case 2:
            pv.double_value = value
        case 3:
            pv.string_value = value
        case 4:
            pv.byte_array_value = value
        case 5:
            pv.bool_array_value = value
        case 6:
            pv.integer_array_value = value
        case 7:
            pv.double_array_value = value
        case 8:
            pv.string_array_value = value


def parse_value(p: Parameter, value: str):
    match p.value.type:
        case 0:
            return bool(value)
        case 1:
            return int(value)
        case 2:
            return float(value)
        case 3:
            return str(value)
        case 4:
            return bytes(value)
        case 5:
            return list(bool(v) for v in value.split("\\s*,\\s*"))
        case 6:
            return list(int(v) for v in value.split("\\s*,\\s*"))
        case 7:
            return list(float(v) for v in value.split("\\s*,\\s*"))
        case 8:
            return list(str(v) for v in value.split("\\s*,\\s*"))


class NodeModel:
    def __init__(self, parent: Node, node_name: NodeName):
        self.parent = parent
        self.name = node_name.name
        self.namespace = node_name.namespace

        self.set_client = self.parent.create_client(
            SetParameters, f"{node_name.full_name}/set_parameters"
        )
        self.get_client = self.parent.create_client(
            GetParameters, f"{node_name.full_name}/get_parameters"
        )
        self.list_client = self.parent.create_client(
            ListParameters, f"{node_name.full_name}/list_parameters"
        )

        self.set_client_req = SetParameters.Request()
        self.get_client_req = GetParameters.Request()
        self.list_client_req = ListParameters.Request()
        self.refresh()

    def refresh(self):
        self.future = self.list_client.call_async(self.list_client_req)
        rclpy.spin_until_future_complete(self.parent, self.future)
        self.parameter_names: list[str] = self.future.result().result.names
        self.parameter_names.remove('use_sim_time')
        self.get_client_req.names = self.parameter_names
        self.future = self.get_client.call_async(self.get_client_req)
        rclpy.spin_until_future_complete(self.parent, self.future)
        self.parameter_values: list[ParameterValue] = self.future.result().values
        
        self.parent.get_logger().info(f"{self.parameter_names}")
        self.parent.get_logger().info(f"{self.parameter_values}")

        self.parameters = {
            self.parameter_names[i]: Parameter(
                name=self.parameter_names[i], value=self.parameter_values[i]
            )
            for i in range(len(self.parameter_names))
        }

    def type_int(self, type: type) -> int:
        return parameter_types.index(type)

    def set_parameter(self, parameter: str, value):
        if parameter not in self.parameters:
            return

        if self.type_int(type(value)) != self.parameters[parameter].value.type:
            return

        set_value(self.parameters[parameter].value, value)
        self.set_client_req.parameters = list(self.parameters.values())
        self.future = self.set_client.call_async(self.set_client_req)
        rclpy.spin_until_future_complete(self.parent, self.future)


class GUINode(Node):
    def __init__(self):
        super().__init__("gui_node")
        self.refresh()
        self.req = SetParameters.Request()

        self.window = tk.Tk()
        tk.Button(self.window, text="refresh", command=self.refresh).pack()
        
        for node in self.nodes:
            self.add_node(node)
        
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.window.mainloop()

    def add_node(self, node: NodeModel):
        tk.Label(self.window, text=f"node: {node.name}").pack()

        for parameter in node.parameters.values():
            tk.Label(self.window, text=f"param: {parameter.name}")
            entry = tk.Entry(self.window)
            entry.insert(0, get_value(parameter.value))
            entry.pack()
            tk.Button(
                self.window,
                text="change",
                command=node.set_parameter(
                    parameter.name, parse_value(parameter, entry.get())
                ),
            ).pack()

    def refresh(self):
        self.nodes = list(map(lambda n: NodeModel(self, n), get_node_names(node=self)))

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
