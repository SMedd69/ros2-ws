import rclpy
from rclpy.node import Node

from robot_interfaces.msg import ComponentStatus, Safety

class SafetyNode(Node):

    def __init__(self):
        super().__init__(node_name="safety_node")

        # --------------------------------
        # Components status
        # --------------------------------

        self.components = {}

        # --------------------------------
        # Sensor status subscription
        # --------------------------------

        self.sensor_subscription = self.create_subscription(
            ComponentStatus,
            "/robot/status/sensor",
            self.status_callback,
            10
        )

        # --------------------------------
        # Battery status subscription
        # --------------------------------

        self.battery_subscription = self.create_subscription(
            ComponentStatus,
            "/robot/status/battery",
            self.status_callback,
            10
        )

        # --------------------------------
        # Safety publisher
        # --------------------------------

        self.safety_publisher = self.create_publisher(
            Safety,
            "/robot/safety",
            10
        )

        self.get_logger().info(
            "Safety Node started"
        )

    # --------------------------------
    # Status callback
    # --------------------------------

    def status_callback(self, msg):

        self.components[msg.component] = msg

        self.publish_safety()

    # --------------------------------
    # Determine global safety
    # --------------------------------

    def determine_global_safety(self):

        if not self.components:
            return (
                ComponentStatus.CRITICAL,
                "No component status available"
            )

        global_level = ComponentStatus.OK
        global_reason = "All systems operating normally"

        for component in self.components.values():

            if component.level > global_level:
                global_level = component.level

                global_reason = (
                    f"{component.component}: "
                    f"{component.reason}"
                )

        return global_level, global_reason

    # --------------------------------
    # Publish global safety
    # --------------------------------

    def publish_safety(self):

        global_level, global_reason = (
            self.determine_global_safety()
        )

        safety_msg = Safety()

        safety_msg.components = list(
            self.components.values()
        )

        safety_msg.global_level = global_level
        safety_msg.global_reason = global_reason

        self.safety_publisher.publish(
            safety_msg
        )

        self.get_logger().info(
            "---------------- SAFETY STATUS ----------------\n"
            + "\n".join(
                f"  {component.component}: "
                f"level={component.level} | "
                f"reason={component.reason}"
                for component in self.components.values()
            )
            + "\n"
            f"  GLOBAL: level={global_level} | "
            f"reason={global_reason}\n"
            "-----------------------------------------------"
        )


def main(args=None):

    rclpy.init(args=args)

    node = SafetyNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
