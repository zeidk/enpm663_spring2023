#!/usr/bin/env python3

import rclpy
from first_package.first_package import SubscriberNode

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode('subscriber_py')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
