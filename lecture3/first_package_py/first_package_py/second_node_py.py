import rclpy
from rclpy.node import Node

# Non OOP version
##################


def main(args=None):
    rclpy.init(args=args)
    node = Node('second_node_py')
    node.get_logger().info('second_node_py node running')
    rclpy.spin(node)
    rclpy.shutdown()


# OOP version
##################

# class FirstNode(Node):
#     def __init__(self, node_name):
#         super().__init__(node_name)
#         self.get_logger().info(f'{node_name} node running')

# def main(args=None):
#     rclpy.init(args=args)
#     node = FirstNode('first_node_py')
#     rclpy.spin(node)
#     rclpy.shutdown()

if __name__ == '__main__':
    main()