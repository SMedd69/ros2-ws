import time

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import ComponentStatus, Safety

class SafetyNode(Node):

    def __init__(self):
        super().__init__(node_name="safety_node")

        # --------------------------------
        # Configuration
        # --------------------------------
        self.component_timeout = 0.5  # 500 ms

        # --------------------------------
        # Components status
        # --------------------------------
        self.components = {}

        # Last message received from each component
        self.last_component_message = {}

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
        # Camera status subscription
        # --------------------------------
        self.camera_subscription = self.create_subscription(
            ComponentStatus,
            "/robot/status/camera",
            self.status_callback,
            10
        )

        # --------------------------------
        # Safety publisher
        # --------------------------------
        self.safety_publisher = self.create_publisher(Safety, "/robot/safety", 10)

        # --------------------------------
        # Safety heartbeat timer
        # --------------------------------
        self.safety_timer = self.create_timer(0.1, self.publish_safety)

        # --------------------------------
        # Components watchdog timer
        # --------------------------------
        self.watchdog_timer = self.create_timer(0.1, self.components_watchdog_callback)

        self.get_logger().info("Safety Node started")

    # --------------------------------
    # Status callback
    # --------------------------------
    def status_callback(self, msg):
        component_name = msg.component
        self.components[component_name] = msg
        self.last_component_message[component_name] = time.monotonic()

    # --------------------------------
    # Components watchdog
    # --------------------------------
    def components_watchdog_callback(self):
        now = time.monotonic()

        # --------------------------------
        # No component status received yet
        # --------------------------------
        expected_components = [
            "sensor_node",
            "battery_node",
            "camera_node"
        ]

        for component_name in expected_components:

            # Component has never sent a message
            if component_name not in self.last_component_message:
                self.get_logger().error(f"{component_name}: " f"No status message received.")

                continue

            # --------------------------------
            # Check timeout
            # --------------------------------
            elapsed = (now - self.last_component_message[component_name])

            if elapsed > self.component_timeout:

                # Avoid changing/logging repeatedly
                component = self.components.get(component_name)

                if component is not None:
                    if component.level != ComponentStatus.CRITICAL:
                        component.level = ComponentStatus.CRITICAL

                        component.reason = (
                            f"{component_name}: "
                            f"Status timeout "
                            f"({elapsed:.2f}s)"
                        )

                        self.get_logger().error(
                            f"{component_name}: "
                            f"Status timeout! "
                            f"No message received for "
                            f"{elapsed:.2f}s."
                        )

    # --------------------------------
    # Determine global safety
    # --------------------------------
    def determine_global_safety(self):
        if not self.components:
            return (ComponentStatus.CRITICAL, "No component status available")

        global_level = ComponentStatus.READY
        global_reason = "All systems operating ready"

        for component in self.components.values():
            if component.level > global_level:
                global_level = component.level
                global_reason = (f"{component.component}: " f"{component.reason}")

        return global_level, global_reason

    # --------------------------------
    # Publish global safety
    # --------------------------------
    def publish_safety(self):

        global_level, global_reason = (self.determine_global_safety())

        safety_msg = Safety()

        safety_msg.components = list(self.components.values())

        safety_msg.global_level = global_level
        safety_msg.global_reason = global_reason

        self.safety_publisher.publish(safety_msg)


def main(args=None):

    rclpy.init(args=args)

    node = SafetyNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()