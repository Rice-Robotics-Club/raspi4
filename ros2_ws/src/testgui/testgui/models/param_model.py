from enum import Enum
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


class ParamType(Enum):
    BOOL = 1
    INT = 2
    DOUBLE = 3
    STRING = 4
    BYTE_ARRAY = 5
    BOOL_ARRAY = 6
    INT_ARRAY = 7
    DOUBLE_ARRAY = 8
    STRING_ARRAY = 9


# TODO: perhaps this could be done better with inheritance
class ParameterModel:
    def __init__(self, name: str, value: ParameterValue):
        self.value = value
        self.name = name
        self.type = ParamType(value=self.value.type)
        self.data = self.get_value()

    @staticmethod
    def type_int(value):
        return parameter_types.index(type(value)) + 1

    def get_value(self):
        match self.type:
            case ParamType.BOOL:
                return self.value.bool_value
            case ParamType.INT:
                return self.value.integer_value
            case ParamType.DOUBLE:
                return self.value.double_value
            case ParamType.STRING:
                return self.value.string_value
            case ParamType.BYTE_ARRAY:
                return self.value.byte_array_value
            case ParamType.BOOL_ARRAY:
                return self.value.bool_array_value
            case ParamType.INT_ARRAY:
                return self.value.integer_array_value
            case ParamType.DOUBLE_ARRAY:
                return self.value.double_array_value
            case ParamType.STRING_ARRAY:
                return self.value.string_array_value

    def set_value(self, data):
        self.data = data
        match self.type:
            case ParamType.BOOL:
                self.value.bool_value = data
            case ParamType.INT:
                self.value.integer_value = data
            case ParamType.DOUBLE:
                self.value.double_value = data
            case ParamType.STRING:
                self.value.string_value = data
            case ParamType.BYTE_ARRAY:
                self.value.byte_array_value = data
            case ParamType.BOOL_ARRAY:
                self.value.bool_array_value = data
            case ParamType.INT_ARRAY:
                self.value.integer_array_value = data
            case ParamType.DOUBLE_ARRAY:
                self.value.double_array_value = data
            case ParamType.STRING_ARRAY:
                self.value.string_array_value = data

    def set_value_from_string(self, value: str) -> None:
        match self.type:
            case ParamType.BOOL:
                self.set_value(bool(value))
            case ParamType.INT:
                self.set_value(int(value))
            case ParamType.DOUBLE:
                self.set_value(float(value))
            case ParamType.STRING:
                self.set_value(str(value))
            case ParamType.BYTE_ARRAY:
                self.set_value(bytes(value))
            case ParamType.BOOL_ARRAY:
                self.set_value(list(bool(v) for v in value.split("\\s*,\\s*")))
            case ParamType.INT_ARRAY:
                self.set_value(list(int(v) for v in value.split("\\s*,\\s*")))
            case ParamType.DOUBLE_ARRAY:
                self.set_value(list(float(v) for v in value.split("\\s*,\\s*")))
            case ParamType.STRING_ARRAY:
                self.set_value(list(str(v) for v in value.split("\\s*,\\s*")))

    def get_parameter(self) -> Parameter:
        return Parameter(name=self.name, value=self.value)
