import rclpy
from rclpy.node import Node

from robot_interfaces.msg import Battery, ComponentStatus

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")

        self.publisher_ = self.create_publisher(
            Battery,
            "/robot/battery",
            10
        )

        self.status_publisher_ = self.create_publisher(
            ComponentStatus,
            "/robot/status/battery",
            10
        )

        self.battery_level = 100.0

        self.timer = self.create_timer(
            0.1,
            self.publish_battery
        )

        self.get_logger().info("Battery Node started")

    def publish_battery(self):
        self.battery_level -= 1.0

        if self.battery_level <= 0:
            self.battery_level = 100.0

        msg = Battery()

        msg.percentage = self.battery_level
        msg.charging = False

        self.publisher_.publish(msg=msg)

        status_msg = ComponentStatus()

        status_msg.component = "battery_node"
        status_msg.level = ComponentStatus.OK
        status_msg.reason = "battery status ok"

        self.status_publisher_.publish(status_msg)

        self.get_logger().info(f"Battery: {msg.percentage:1f}%" f"Status: {status_msg.level} | " f"{status_msg.reason}")

def main(args=None):
    rclpy.init(args=args)

    node = BatteryNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()