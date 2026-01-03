#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from explainable_robot.msg import RobotState, SensorStatus

class TaskStateManager(Node):
    def __init__(self):
        super().__init__('task_state_manager')
        self.publisher_ = self.create_publisher(RobotState, 'robot_state', 10)
        self.subscription = self.create_subscription(
            SensorStatus,
            'sensor_status',
            self.sensor_callback,
            10
        )
        self.state = 'IDLE'
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sensor_confidences = {}
        self.get_logger().info('Task State Manager node started')

    def sensor_callback(self, msg):
        self.sensor_confidences[msg.sensor_name] = msg.confidence
        
        # Simple transition logic
        if msg.status == 'FAIL':
            self.state = 'ERROR'
        elif msg.status == 'DEGRADED' and self.state != 'ERROR':
            self.state = 'WAITING'
        elif all(c > 0.8 for c in self.sensor_confidences.values()) and self.state in ['WAITING', 'ERROR']:
             self.state = 'OBSERVING'

    def timer_callback(self):
        # Basic FSM behavior for demo
        if self.state == 'IDLE':
            self.state = 'OBSERVING'
        elif self.state == 'OBSERVING':
            # random chance to process
            import random
            if random.random() > 0.7:
                self.state = 'PROCESSING'
        elif self.state == 'PROCESSING':
            import random
            if random.random() > 0.8:
                self.state = 'IDLE'

        msg = RobotState()
        msg.state = self.state
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Current Robot State: {self.state}')

def main(args=None):
    rclpy.init(args=args)
    node = TaskStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
