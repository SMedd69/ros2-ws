import random

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import Distance, ComponentStatus

class SensorNode(Node):

    def __init__(self):
        super().__init__(node_name="sensor_node")

        self.obstacle = True
        self.level = 0

        self.sensor_publisher_ = self.create_publisher(
            Distance,
            "/robot/distance",
            10
        )

        self.status_publisher_ = self.create_publisher(
            ComponentStatus,
            "/robot/status/sensor",
            10
        )

        self.distance_timer = self.create_timer(
            0.1,
            self.publish_distance
        )
        self.status_timer = self.create_timer(
            0.1,
            self.publish_status
        )

        self.get_logger().info("Sensor Node started")

    def publish_distance(self):
        distance = random.uniform(0.3, 3.5)
        self.obstacle = distance < 0.7

        distance_msg = Distance()
        distance_msg.distance = distance
        distance_msg.obstacle = self.obstacle
    
        self.sensor_publisher_.publish(distance_msg)

    def publish_status(self):
        status_msg = ComponentStatus()

        status_msg.component = "sensor_node"
        status_msg.level = ComponentStatus.OK
        status_msg.reason = "Sensor status ok"

        self.status_publisher_.publish(status_msg)

def main(args=None):

    rclpy.init(args=args)

    node = SensorNode()
    rclpy.spin(node=node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()