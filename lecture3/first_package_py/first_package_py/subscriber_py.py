import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class SubscriberNode(Node):
    
    '''SubscriberNode This class is a node that subscribes to a string message and publishes the first word of the message.

    Arguments:
        Node -- Node class from rclpy
    '''
    def __init__(self, node_name):
        super().__init__(node_name)
        self._subscriber = self.create_subscription(String, 'chatter663', self.subscriber_callback, 10)
        # self._publisher = self.create_publisher(String, 'chatter2', 10)
        self.get_logger().info(f'{node_name} node running')

    def subscriber_callback(self, msg: String):
        '''subscriber_callback Callback function for the subscriber to the chatter663 topic.

        Arguments:
            msg -- String message from the chatter663 topic.
        '''

        self.get_logger().info('Receiving: "%s"' % msg.data)
        # first_word = str(msg.data).split()[0]
        # new_msg = String()
        # new_msg.data = first_word
        # self._publisher.publish(new_msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode('subscriber_py')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
