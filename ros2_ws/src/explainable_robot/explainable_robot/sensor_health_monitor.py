#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from explainable_robot.msg import SensorStatus
import random

class SensorHealthMonitor(Node):
    def __init__(self):
        super().__init__('sensor_health_monitor')
        self.publisher_ = self.create_publisher(SensorStatus, 'sensor_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sensors = ['lidar', 'camera', 'imu']
        self.get_logger().info('Sensor Health Monitor node started')

    def timer_callback(self):
        for sensor in self.sensors:
            msg = SensorStatus()
            msg.sensor_name = sensor
            
            # Simulate some degradation based on random noise
            rand_val = random.random()
            if rand_val > 0.9:
                msg.status = 'FAIL'
                msg.confidence = 0.0
            elif rand_val > 0.7:
                msg.status = 'DEGRADED'
                msg.confidence = random.uniform(0.3, 0.7)
            else:
                msg.status = 'OK'
                msg.confidence = random.uniform(0.8, 1.0)
            
            self.publisher_.publish(msg)
            # self.get_logger().info(f'Published health for {sensor}: {msg.status} ({msg.confidence:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = SensorHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
