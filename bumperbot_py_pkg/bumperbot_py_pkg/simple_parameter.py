import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter


class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")

        self.declare_parameter("int_param", 20)
        self.declare_parameter("srting_param", "Hello world!")

        self.add_on_set_parameters_callback(self.parameter_change_callback)
    
    def parameter_change_callback(self, params):
        result = SetParametersResult()

        for param in params:
            if param.name == "int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"int param is changed. New value is: {param.value} ")
                result.successful = True

            if param.name == "string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"int param is changed. New value is: {param.value} ")
                result.successful= True
        return result



def main():
    rclpy.init()
    node = SimpleParameter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()