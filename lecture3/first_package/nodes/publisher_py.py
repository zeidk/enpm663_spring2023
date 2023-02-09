#!/usr/bin/env python3

import rclpy
from first_package.first_package import PublisherNode

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode('publisher_py')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
