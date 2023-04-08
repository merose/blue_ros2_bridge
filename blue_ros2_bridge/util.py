from builtin_interfaces.msg import Time

from rclpy.parameter import Parameter
from rclpy.time import Time

from rcl_interfaces.msg import ParameterDescriptor


class BaseNode:

    _PARAMETER_TYPE_MAP = {
        bool: Parameter.Type.BOOL.value,
        float: Parameter.Type.DOUBLE.value,
        int: Parameter.Type.INTEGER.value
    }

    _PARAMETER_CONVERSIONS = {
        bool: lambda value: value.bool_value,
        float: lambda value: value.double_value,
        int: lambda value: value.integer_value
    }

    def __init__(self, node):
        self.node = node
        self._parameters = set()

    def get_node(self):
        return self.node

    def get_stamp(self):
        return self.node.get_clock().now().to_msg()

    def get_time(self):
        return self.time_from_stamp(self.get_stamp())

    def declare_parameter(self, name, default=None, value_type=None,
                          description=''):
        python_type = self.get_value_type(value_type, default)
        param_type = self._PARAMETER_TYPE_MAP[python_type] \
            if python_type in self._PARAMETER_TYPE_MAP \
            else Parameter.Type.STRING.value
        descriptor = ParameterDescriptor(type=param_type,
                                         description=description)
        self.node.declare_parameter(name, value=default, descriptor=descriptor)
        self._parameters.add(name)

    def get_parameter(self, name, default=None, value_type=None, 
                      description=''):
        if name not in self._parameters:
            self.declare_parameter(name, value_type=value_type,
                                   default=default, description=description)
        python_type = self.get_value_type(value_type, default)
        parameter = self.node.get_parameter_or(name)
        if parameter.type_ == Parameter.Type.NOT_SET:
            return default
        else:
            parameter_value = parameter.get_parameter_value()
            return self._PARAMETER_CONVERSIONS[python_type](parameter_value) \
                if python_type in self._PARAMETER_CONVERSIONS \
                   else parameter_value.string_value

    def get_value_type(self, value_type, default):
        return value_type if value_type is not None \
            else type(default) if default is not None \
            else str

    def log_debug(self, msg):
        self.node.get_logger().debug(msg)

    def log_info(self, msg):
        self.node.get_logger().info(msg)

    def log_warning(self, msg):
        self.node.get_logger().warning(msg)

    def log_error(self, msg):
        self.node.get_logger().error(msg)

    def ros_time_from_stamp(self, stamp):
        return Time.from_msg(stamp)

    def time_from_stamp(self, stamp):
        return stamp.sec + stamp.nanosec/1E9

    def stamp_from_time(self, timestamp):
        stamp = Time()
        stamp.sec = int(timestamp)
        stamp.nanosec = int((timestamp - stamp.sec) * 1E9)
        return stamp
