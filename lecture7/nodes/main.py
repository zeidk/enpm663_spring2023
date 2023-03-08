#!/usr/bin/env python3

import rclpy
#!/usr/bin/env python3

from lecture7.competition_interface import CompetitionInterface


def main(args=None):
    rclpy.init(args=args)

    interface = CompetitionInterface()

    interface.start_competition()
    
    rclpy.spin(interface)
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
