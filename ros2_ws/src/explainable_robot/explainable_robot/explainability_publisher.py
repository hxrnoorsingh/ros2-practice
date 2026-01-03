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
        # Store detailed status
        found = False
        for issue in self.sensor_issues:
            if issue['name'] == msg.sensor_name:
                issue['status'] = msg.status
                issue['confidence'] = msg.confidence
                found = True
                break
        if not found:
            self.sensor_issues.append({
                'name': msg.sensor_name,
                'status': msg.status,
                'confidence': msg.confidence
            })

    def timer_callback(self):
        status_map = {
            "IDLE": "I am standing still, waiting for instructions.",
            "OBSERVING": "I am scanning the area for pedestrians and obstacles.",
            "PROCESSING": "I am analyzing the path forward.",
            "MOVING": "I am proceeding to my destination.",
            "WAITING": "I am paused due to temporary uncertainty.",
            "ERROR": "I have stopped because of a system malfunction.",
            "SUCCESS": "I have successfully reached my goal."
        }
        
        base_explanation = status_map.get(self.current_state, f"I am currently {self.current_state}.")
        explanation = f"{base_explanation} "
        
        issues = [i for i in self.sensor_issues if i['status'] != 'OK']
        if issues:
            detail = []
            for i in issues:
                if i['status'] == 'FAIL':
                    detail.append(f"TOTAL FAILURE of {i['name']}")
                else:
                    detail.append(f"DEGRADED performance in {i['name']} (Confidence: {i['confidence']:.2f})")
            explanation += f"CAUTION: I detect {', '.join(detail)}. "
        else:
            if self.current_uncertainty > 0.5:
                explanation += f"I am proceeding with caution because I feel uncertain (level: {self.current_uncertainty:.2f}). "
            else:
                explanation += "Everything looks clear."
            
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
