import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from rclpy.qos import qos_profile_sensor_data

class LidarVisualizer(Node):

    def __init__(self):
        super().__init__("lidar_visualizer")

        # ---------------------------------------------------------
        # Configuration
        # ---------------------------------------------------------

        self.topic = "/robot/lidar/points"

        self.max_points = 10000

        # ---------------------------------------------------------
        # Point cloud data
        # ---------------------------------------------------------

        self.points = []

        # ---------------------------------------------------------
        # ROS2 subscriber
        # ---------------------------------------------------------

        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic,
            self.lidar_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Lidar visualizer started - listening to {self.topic}"
        )

        # ---------------------------------------------------------
        # Matplotlib
        # ---------------------------------------------------------

        self.fig = plt.figure(figsize=(10, 8))

        self.ax = self.fig.add_subplot(
            111,
            projection="3d",
        )

        self.ax.set_title("LiDAR 3D")

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        # Limites initiales
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_zlim(-1, 3)

        # Position du robot
        self.ax.scatter(
            [0],
            [0],
            [0],
            marker="o",
            s=100,
        )

        # Nuage de points
        self.scatter = self.ax.scatter(
            [],
            [],
            [],
            s=2,
        )

        # ---------------------------------------------------------
        # Animation
        # ---------------------------------------------------------

        self.animation = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=100,
            cache_frame_data=False,
        )

    # =============================================================
    # ROS2
    # =============================================================
    def lidar_callback(self, msg: PointCloud2):

        points = []

        for point in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            points.append((x, y, z))

        # Limite de sécurité pour la visualisation
        if len(points) > self.max_points:
            points = points[:self.max_points]

        self.points = points

        self.get_logger().info(f"Visualizer received {len(points)} points")

    # =============================================================
    # Matplotlib
    # =============================================================
    def update_plot(self, _frame):
        if not self.points:
            return self.scatter,

        xs = [point[0] for point in self.points]
        ys = [point[1] for point in self.points]
        zs = [point[2] for point in self.points]

        self.scatter._offsets3d = (
            xs,
            ys,
            zs,
        )

        return self.scatter,

    # =============================================================
    # Shutdown
    # =============================================================
    def destroy_node(self):
        plt.close(self.fig)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LidarVisualizer()

    plt.ion()
    node.fig.show()

    try:
        while rclpy.ok() and plt.fignum_exists(node.fig.number):
            rclpy.spin_once(
                node,
                timeout_sec=0.01,
            )

            node.update_plot(None)

            node.fig.canvas.draw_idle()
            node.fig.canvas.flush_events()

            plt.pause(0.01)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()