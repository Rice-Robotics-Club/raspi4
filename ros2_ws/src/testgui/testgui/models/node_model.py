import rclpy
from rclpy.node import Node
import rclpy.parameter
from ros2node.api import NodeName
from rcl_interfaces.srv import SetParameters, ListParameters, GetParameters
from rcl_interfaces.msg import ParameterValue
from .param_model import ParameterModel


class NodeModel:
    def __init__(self, parent: Node, node_name: NodeName):
        """_summary_

        Args:
            parent (Node): the rclpy Node object used to create service clients on,
                since this is not a rclpy Node
            node_name (NodeName): the object containing the node's name and namespace
        """
        self.parent = parent
        self.name: str = node_name.name
        self.namespace: str = node_name.namespace
        self.full_name: str = node_name.full_name
        self.parameters: dict[str, ParameterModel] = {}

        self.set_client = self.parent.create_client(
            SetParameters, f"{self.full_name}/set_parameters"
        )
        self.get_client = self.parent.create_client(
            GetParameters, f"{self.full_name}/get_parameters"
        )
        self.list_client = self.parent.create_client(
            ListParameters, f"{self.full_name}/list_parameters"
        )

        self.set_parameters_req = SetParameters.Request()

        self.get_parameters()

    def get_parameters(self):
        """This method first calls the /list_parameters service in order to get the
        parameter names, and passes these names to the /get_parameters service to get
        parameter values. Each parameter is then added as a ParameterModel wrapper class
        to the self.parameters dictionary, indexed by the parameter name.
        """
        # calls /list_parameters to get parameter names
        self.future = self.list_client.call_async(ListParameters.Request())
        rclpy.spin_until_future_complete(self.parent, self.future)
        parameter_names: list[str] = self.future.result().result.names

        # excludes this default perameter that we don't want to see in GUI
        parameter_names.remove("use_sim_time")

        # calls /get_parameters to get parameter values
        self.future = self.get_client.call_async(
            GetParameters.Request(names=parameter_names)
        )
        rclpy.spin_until_future_complete(self.parent, self.future)
        parameter_values: list[ParameterValue] = self.future.result().values

        # wraps each resulting list into a dictionary of ParameterModel wrapper objects
        self.parameters = {
            parameter_names[i]: ParameterModel(
                name=parameter_names[i], value=parameter_values[i]
            )
            for i in range(len(parameter_names))
        }

    def set_parameters(self, parameters: dict[str, str]):
        """Updates this object's parameters as well as those of the actual ROS2 Node

        Args:
            parameters (dict[str, str]): dictionary mapping parameter names to string
                representations of the values to be assigned, taken from GUI
        """
        for name, value in parameters.items():
            if name in self.parameters:
                self.parameters[name].set_value_from_string(value)

        self.set_parameters_req.parameters = list(
            map(lambda p: p.get_parameter(), self.parameters.values())
        )
        self.future = self.set_client.call_async(self.set_parameters_req)
        rclpy.spin_until_future_complete(self.parent, self.future)
