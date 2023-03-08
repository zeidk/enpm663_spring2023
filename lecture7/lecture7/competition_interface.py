#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# State of the competition message
from ariac_msgs.msg import CompetitionState
# Order message
from ariac_msgs.msg import Order as OrderMsg
from ariac_msgs.msg import KittingTask as KittingTaskMsg
from ariac_msgs.msg import AssemblyTask as AssemblyTaskMsg
from ariac_msgs.msg import KittingPart as KittingPartMsg
from ariac_msgs.msg import AssemblyPart as AssemblyPartMsg

from competitor_interfaces.msg import FloorRobotTask as FloorRobotTaskMsg
from competitor_interfaces.msg import CompletedOrder as CompletedOrderMsg

from ariac_msgs.srv import MoveAGV as MoveAGVSrv
from std_srvs.srv import Trigger


# -----------------------------------------------------------------------------
class KittingTask:
    def __init__(self, kitting_task: KittingTaskMsg) -> None:
        self.agv_number = kitting_task.agv_number
        self.tray_id = kitting_task.tray_id
        self.destination = kitting_task.destination
        # list of KittingPart objects
        self.parts = list(map(lambda x: KittingPart(x), kitting_task.parts))


# -----------------------------------------------------------------------------
class AssemblyTask:
    def __init__(self, assembly_task: AssemblyTaskMsg) -> None:
        self.agv_number = assembly_task.agv_number
        self.station = assembly_task.station
        self.parts = list(map(lambda x: AssemblyPart(x), assembly_task.parts))


# -----------------------------------------------------------------------------
class CombinedTask:
    def __init__(self) -> None:
        pass


# -----------------------------------------------------------------------------
class KittingPart:
    def __init__(self, kitting_part: KittingPartMsg) -> None:
        self.part_type = kitting_part.part.type
        self.part_color = kitting_part.part.color
        self.quadrant = kitting_part.quadrant


# -----------------------------------------------------------------------------
class AssemblyPart:
    def __init__(self, assembly_part: AssemblyPartMsg) -> None:
        self.part_type = assembly_part.part.type
        self.part_color = assembly_part.part.color
        self.assembled_pose = assembly_part.assembled_pose
        self.install_direction = assembly_part.install_direction


# -----------------------------------------------------------------------------
class CombinedPart:
    def __init__(self) -> None:
        pass


# -----------------------------------------------------------------------------
class Order:
    ''' Order class for storing order information from the topic /ariac/orders.
    '''

    def __init__(self, msg: OrderMsg) -> None:
        self.order_id = msg.id
        self.order_type = msg.type
        self.order_priority = msg.priority

        if self.order_type == OrderMsg.KITTING:
            self.order_task = KittingTask(msg.kitting_task)
        elif self.order_type == OrderMsg.ASSEMBLY:
            self.order_task = AssemblyTask(msg.assembly_task)
        elif self.order_type == OrderMsg.COMBINED:
            self.order_task = CombinedTask(msg.combined_task)
        else:
            self.order_task = None
            
    def __str__(self) -> str:
        return f'Order ID: {self.order_id}, Order Type: {self.order_type}, Order Priority: {self.order_priority}'


# -----------------------------------------------------------------------------
class CompetitionInterface(Node):
    '''CompetitionInterface class for interfacing with ARIAC.

    Arguments:
        Node -- ROS2 node class to inherit from for creating a node.
    '''

    states = {
        CompetitionState.IDLE: 'idle',
        CompetitionState.READY: 'ready',
        CompetitionState.STARTED: 'started',
        CompetitionState.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionState.ENDED: 'ended',
    }

    def __init__(self) -> None:
        super().__init__('start_competition_node')

        self.competition_state = None
        self.orders = []
        self.announced_orders = []

        # Create subscription to competition state
        self.competition_state_sub = self.create_subscription(
            CompetitionState,
            '/ariac/competition_state',
            self.competition_state_cb,
            10)

        # Create subscription to orders
        self.orders_sub = self.create_subscription(
            OrderMsg,
            '/ariac/orders',
            self.orders_cb,
            10)
        
        # Create subscription to orders
        self.completed_order_sub = self.create_subscription(
            CompletedOrderMsg,
            '/competitor/completed_order',
            self.completed_order_cb,
            1)
        
        # Publishers
        self.floor_robot_task_pub = self.create_publisher(FloorRobotTaskMsg, '/competitor/floor_robot_task', 1)
        
        # services
        self.start_competition_client = self.create_client(Trigger, '/ariac/start_competition')
        self.end_competition_client = self.create_client(Trigger, '/ariac/end_competition')
        
        timer_period = 0.5  # seconds
        self.task_manager_timer = self.create_timer(timer_period, self.task_manager_cb)

    # -----------------------------------------------------------------------------

    def completed_order_cb(self, msg) -> None:
        '''Callback to process completed orders.

        Arguments:
            msg -- ROS2 message of type competitor_interfaces/CompletedOrder
        '''
        for order in self.announced_orders:
            if order.id == msg.order_id:
                if order.type == OrderMsg.KITTING:
                    agv = order.kitting_task.agv_number
                    self.lock_agv(agv)
                    self.move_agv(agv, order.kitting_task.destination)
                
                
    def task_manager_cb(self) -> None:
        '''Callback to process orders.

        Arguments:
            msg -- ROS2 message of type ariac_msgs/Order
        '''
        
        for order in self.orders:
            if order.type == OrderMsg.KITTING:
                msg = FloorRobotTaskMsg()
                msg.id = order.id
                msg.type = FloorRobotTaskMsg.KITTING
                msg.kitting_task = order.kitting_task
                msg.priority = order.priority
                # Publish the message
                self.floor_robot_task_pub.publish(msg)
                self.announced_orders.append(order)
                self.orders.remove(order)
        
        
    # -----------------------------------------------------------------------------

    def orders_cb(self, msg: OrderMsg) -> None:
        '''Callback function for /ariac/orders topic.

        Arguments:
            msg -- ROS2 message of type ariac_msgs/Order
        '''
        # self.orders.append(Order(msg))
        self.orders.append(msg)

    # -----------------------------------------------------------------------------

    def competition_state_cb(self, msg: CompetitionState) -> None:
        '''Callback function for /ariac/competition_state topic.

        Arguments:
            msg -- ROS2 message of type ariac_msgs/CompetitionState
        '''
        # Log if competition state has changed
        if self.competition_state != msg.competition_state:
            self.get_logger().info(
                f'Competition state is: {self.states[msg.competition_state]}',
                throttle_duration_sec=1.0)
        self.competition_state = msg.competition_state

    # -----------------------------------------------------------------------------

    def start_competition(self) -> None:
        '''Function which contains the service to start the competition.
        '''
        self.get_logger().info('Waiting for competition to be ready')

        # Wait for competition to be ready
        while (self.competition_state != CompetitionState.READY):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return

        self.get_logger().info('Competition is ready. Starting...')

        # Call ROS service to start competition
        while not self.start_competition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ariac/start_competition to be available...')

        # Create trigger request and call start competition service
        request = Trigger.Request()
        future = self.start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
        else:
            self.get_logger().info('Unable to start competition')

    # -----------------------------------------------------------------------------

    def end_competition(self) -> None:
        '''Function which contains the service to end the competition.
        '''

        # Call ROS service to start competition
        while not self.end_competition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ariac/end_competition to be available...')

        # Create trigger request and call end competition service
        request = Trigger.Request()
        future = self.end_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Ended competition.')
        else:
            self.get_logger().info('Unable to end competition')

    # -----------------------------------------------------------------------------
    def lock_agv(self, agv_num):
        service_name = '/ariac/agv' + str(agv_num) + '_lock_tray'
        lock_agv_client = self.create_client(Trigger, service_name)
        # Create trigger request and call lock agv service
        request = Trigger.Request()
        future = lock_agv_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Tray locked on agv{agv_num}.')
        else:
            self.get_logger().info(f'Unable to to lock agv{agv_num}')

    # -----------------------------------------------------------------------------
    def move_agv(self, agv_num, destination):
        '''Function which contains the service to move an AGV to a station.
        '''

        service_name = '/ariac/move_agv' + str(agv_num)
        move_agv_client = self.create_client(MoveAGVSrv, service_name)
        # Create trigger request and call starter service
        request = MoveAGVSrv.Request()
        request.location = destination
        future = move_agv_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'AGV is moving to {destination}')
        else:
            self.get_logger().info(f'Unable to complete {service_name} service')

    # -----------------------------------------------------------------------------
    def submit_order(self):
        pass
