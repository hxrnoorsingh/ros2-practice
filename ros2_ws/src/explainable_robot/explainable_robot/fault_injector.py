#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import random

class FaultInjector(Node):
    def __init__(self):
        super().__init__('fault_injector')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan_raw',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.srv = self.create_service(SetBool, 'toggle_fault', self.toggle_fault_callback)
        self.fault_active = False
        self.get_logger().info('Fault Injector node started')

    def toggle_fault_callback(self, request, response):
        self.fault_active = request.data
        response.success = True
        response.message = f"Fault active: {self.fault_active}"
        return response

    def scan_callback(self, msg):
        if self.fault_active:
            # Inject TOTAL FAILURE for clear testing
            msg.ranges = [float('inf')] * len(msg.ranges)
            self.get_logger().warn('Injecting TOTAL FAILURE')
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
