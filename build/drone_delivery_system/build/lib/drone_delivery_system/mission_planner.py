#!/usr/bin/env python3
"""
Mission Planner Node
High-level mission planning and monitoring
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
import time


class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        
        # State subscriber
        self.state_sub = self.create_subscription(
            String,
            '/delivery/state',
            self.state_callback,
            10
        )
        
        # Service client to start delivery
        self.start_delivery_client = self.create_client(
            Empty,
            '/delivery/start'
        )
        
        self.current_state = 'unknown'
        
        self.get_logger().info('Mission Planner initialized')
        self.get_logger().info('Commands:')
        self.get_logger().info('  ros2 service call /delivery/start std_srvs/srv/Empty')
        
    def state_callback(self, msg):
        """Monitor delivery state"""
        if msg.data != self.current_state:
            self.current_state = msg.data
            self.get_logger().info(f'Delivery State: {self.current_state}')
            
    def start_delivery_mission(self):
        """Start the delivery mission"""
        self.get_logger().info('Starting delivery mission...')
        
        if not self.start_delivery_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Delivery service not available')
            return False
            
        request = Empty.Request()
        future = self.start_delivery_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            self.get_logger().info('Delivery mission started!')
            return True
        else:
            self.get_logger().error('Failed to start delivery mission')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = MissionPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
