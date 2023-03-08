#!/usr/bin/env python3

import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

from lecture7.competition_interface import CompetitionInterface


def main(args=None):
    rclpy.init(args=args)

    node = CompetitionInterface()
    # rclpy.spin(node)
    
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # node.start_competition()
    
    try:
        print("")
        node.get_logger().info('Starting competitor node, end with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.\n')
        
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
