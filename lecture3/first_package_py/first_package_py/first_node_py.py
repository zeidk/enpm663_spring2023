import rclpy
from rclpy.node import Node


# Non OOP version
##################

# def main(args=None):
#     rclpy.init(args=args)
#     node = Node('first_node_py')
#     node.get_logger().info('first_node_py node running')
#     rclpy.spin(node)
#     rclpy.shutdown()

# OOP version
##################

class TemplateNode(Node): # change the name of the class
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name} node running OOP version')
        


def main(args=None):
    rclpy.init(args=args)
    node = TemplateNode('first_node_py') # chnage this
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()