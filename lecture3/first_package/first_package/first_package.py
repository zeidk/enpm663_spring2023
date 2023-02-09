import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class PublisherNode(Node):
    
    ''' PublisherNode This class is a node that publishes a string message.

    Arguments:
        Node -- Node class from rclpy
    '''
    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(String, 'chatter663', 10)
        self._timer = self.create_timer(2, self.timer_callback)
        self.get_logger().info(f'{node_name} node running')
        self._msg = String()

    def timer_callback(self):
        self._msg.data = 'Help me Obi-Wan Kenobi, you are my only hope.'
        self._publisher.publish(self._msg)
        self.get_logger().info('Publishing: "%s"' % self._msg.data)


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

    def subscriber_callback(self, msg):
        self.get_logger().info('Receiving: "%s"' % msg.data)
        # first_word = str(msg.data).split()[0]
        # new_msg = String()
        # new_msg.data = first_word
        # self._publisher.publish(new_msg)
