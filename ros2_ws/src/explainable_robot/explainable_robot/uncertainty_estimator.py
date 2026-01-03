#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from explainable_robot.msg import Uncertainty, SensorStatus

class UncertaintyEstimator(Node):
    def __init__(self):
        super().__init__('uncertainty_estimator')
        self.publisher_ = self.create_publisher(Uncertainty, 'uncertainty', 10)
        self.subscription = self.create_subscription(
            SensorStatus,
            'sensor_status',
            self.sensor_callback,
            10
        )
        self.sensor_confidences = {}
        self.get_logger().info('Uncertainty Estimator node started')

    def sensor_callback(self, msg):
        self.sensor_confidences[msg.sensor_name] = msg.confidence
        self.calculate_uncertainty()

    def calculate_uncertainty(self):
        if not self.sensor_confidences:
            return
        
        # Simple heuristic: average confidence inverted
        avg_confidence = sum(self.sensor_confidences.values()) / len(self.sensor_confidences)
        uncertainty_val = 1.0 - avg_confidence
        
        msg = Uncertainty()
        msg.value = float(uncertainty_val)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UncertaintyEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
