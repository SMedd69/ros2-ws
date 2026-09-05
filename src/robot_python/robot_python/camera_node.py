import rclpy
from rclpy.node import Node

from robot_interfaces.msg import Camera, ComponentStatus 

class CameraNode(Node):

    def __init__(self):
        super().__init__(node_name="camera_node")

        self.camera_publisher_ = self.create_publisher(
            Camera,
            "/robot/camera",
            10
        )

        self.status_publisher_ = self.create_publisher(
            ComponentStatus,
            "/robot/status/camera",
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_camera
        )

        self.get_logger().info("Camera Node started")

    def publish_camera(self):

        msg = Camera()
        msg.thinks = "Quelque chose"

        self.camera_publisher_.publish(msg)

        level = ComponentStatus.OK

        status_msg = ComponentStatus()

        status_msg.component = "camera_node"
        status_msg.level = level

        status_msg.reason = "camera status ok"

        self.status_publisher_.publish(status_msg)

def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()
    rclpy.spin(node=node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()