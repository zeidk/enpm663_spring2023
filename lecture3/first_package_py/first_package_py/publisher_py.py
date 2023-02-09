import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class PublisherNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(String, 'chatter663', 10)
        # publishing every 2s
        self._timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f'{node_name} node running')
        self._msg = String()
        

    def timer_callback(self):
        '''timer_callback Callback function for the timer that publishes a message.
        '''
        
        self._msg.data = 'Help me Obi-Wan Kenobi, you are my only hope.'
        self._publisher.publish(self._msg)
        self.get_logger().info('Publishing: "%s"' % self._msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode('publisher_py')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
