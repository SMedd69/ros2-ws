import rclpy
from rclpy.node import Node

from std_msgs.msg import String 

class CameraNode(Node):

    def __init__(self):
        super().__init__(node_name="camera_node")

        self.publisher_ = self.create_publisher(
            String,
            "/robot/camera",
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_camera
        )

        self.get_logger().info("Camera Node started")

    def publish_camera(self):

        msg = String()
        msg.data = "Vision"
        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Camera: {msg.data}"
        )

def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()
    rclpy.spin(node=node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()