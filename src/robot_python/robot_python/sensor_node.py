import random

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import Distance, ComponentStatus

class SensorNode(Node):

    def __init__(self):
        super().__init__(node_name="sensor_node")

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

        self.timer = self.create_timer(
            1.0,
            self.publish_distance
        )

        self.get_logger().info("Sensor Node started")

    def publish_distance(self):
        
        distance = random.uniform(0.3, 3.5)

        distance_msg = Distance()
        distance_msg.distance = distance
        distance_msg.obstacle = distance < 0.7
    
        self.sensor_publisher_.publish(distance_msg)

        level = random.choices(
            [
                ComponentStatus.OK,
                ComponentStatus.WARNING,
                ComponentStatus.CRITICAL,
                ComponentStatus.DEGRADED,
                ComponentStatus.EMERGENCY,
            ],
            [70, 10, 8, 7, 5]
        )[0]

        status_msg = ComponentStatus()

        status_msg.component = "sensor_node"
        status_msg.level = level

        if level == ComponentStatus.OK:
            status_msg.reason = "Sensor status ok"
        elif level == ComponentStatus.WARNING:
            status_msg.reason = "Sensor status warning"
        elif level == ComponentStatus.CRITICAL:
            status_msg.reason = "Sensor status critical"
        elif level == ComponentStatus.DEGRADED:
            status_msg.reason = "Sensor status degraded"
        elif level == ComponentStatus.EMERGENCY:
            status_msg.reason = "Sensor status emergency"


        self.status_publisher_.publish(status_msg)

        self.get_logger().info( f"Distance: {distance:.2f} m | " f"Obstacle: {distance_msg.obstacle} | " f"Status: {status_msg.level} | " f"{status_msg.reason}" )

def main(args=None):

    rclpy.init(args=args)

    node = SensorNode()
    rclpy.spin(node=node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()