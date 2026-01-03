#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from explainable_robot.msg import SensorStatus
from sensor_msgs.msg import LaserScan
import numpy as np

class SensorHealthMonitor(Node):
    def __init__(self):
        super().__init__('sensor_health_monitor')
        self.publisher_ = self.create_publisher(SensorStatus, 'sensor_status', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.get_logger().info('Realistic Sensor Health Monitor node started')

    def scan_callback(self, msg):
        status = SensorStatus()
        status.sensor_name = 'lidar'
        
        # Calculate health based on number of valid readings
        ranges = np.array(msg.ranges)
        valid_mask = np.isfinite(ranges)
        valid_count = np.sum(valid_mask)
        total_count = len(ranges)
        
        health_ratio = valid_count / total_count if total_count > 0 else 0.0
        
        if health_ratio < 0.5:
            status.status = 'FAIL'
        elif health_ratio < 0.9:
            status.status = 'DEGRADED'
        else:
            status.status = 'OK'
            
        status.confidence = health_ratio
        self.publisher_.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = SensorHealthMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
