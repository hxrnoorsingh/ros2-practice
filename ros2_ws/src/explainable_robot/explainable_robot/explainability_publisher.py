#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from explainable_robot.msg import RobotState, Uncertainty, SensorStatus
from std_msgs.msg import String

class ExplainabilityPublisher(Node):
    def __init__(self):
        super().__init__('explainability_publisher')
        self.publisher_ = self.create_publisher(String, 'explanation', 10)
        
        self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
        self.create_subscription(Uncertainty, 'uncertainty', self.uncertainty_callback, 10)
        self.create_subscription(SensorStatus, 'sensor_status', self.sensor_callback, 10)
        
        self.current_state = "UNKNOWN"
        self.current_uncertainty = 0.0
        self.sensor_issues = []
        
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Explainability Publisher node started')

    def state_callback(self, msg):
        self.current_state = msg.state

    def uncertainty_callback(self, msg):
        self.current_uncertainty = msg.value

    def sensor_callback(self, msg):
        if msg.status != 'OK':
            if msg.sensor_name not in self.sensor_issues:
                self.sensor_issues.append(msg.sensor_name)
        else:
            if msg.sensor_name in self.sensor_issues:
                self.sensor_issues.remove(msg.sensor_name)

    def timer_callback(self):
        explanation = f"I am currently {self.current_state}. "
        
        if self.current_uncertainty > 0.5:
            explanation += f"I am feeling quite uncertain (level: {self.current_uncertainty:.2f}). "
        else:
            explanation += f"I am confident in my actions. "
            
        if self.sensor_issues:
            explanation += f"Issues detected with: {', '.join(self.sensor_issues)}."
            
        msg = String()
        msg.data = explanation
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published explanation: {explanation}')

def main(args=None):
    rclpy.init(args=args)
    node = ExplainabilityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
