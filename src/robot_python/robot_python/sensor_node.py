import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from robot_interfaces.msg import Sensor, ComponentStatus

class SensorNode(Node):

    def __init__(self):
        super().__init__(node_name="sensor_node")

        # ---------------------------------------------------------
        # Configuration
        # ---------------------------------------------------------

        self.obstacle_threshold = 0.7

        # Hauteur approximative considérée comme "sol".
        # Cette valeur sera affinée avec les tests.
        self.ground_z_threshold = 0.08

        # ---------------------------------------------------------
        # Subscriber : LiDAR 3D
        # ---------------------------------------------------------

        self.lidar_subscription_ = self.create_subscription(
            PointCloud2,
            "/simulation/lidar_points/points",
            self.lidar_callback,
            10
        )

        # ---------------------------------------------------------
        # Publisher : perception
        # ---------------------------------------------------------

        self.sensor_publisher_ = self.create_publisher(
            Sensor,
            "/robot/sensor",
            10
        )

        # ---------------------------------------------------------
        # Publisher : état du composant
        # ---------------------------------------------------------

        self.status_publisher_ = self.create_publisher(
            ComponentStatus,
            "/robot/status/sensor",
            10
        )

        # ---------------------------------------------------------
        # Heartbeat
        # ---------------------------------------------------------

        self.status_timer = self.create_timer(
            0.1,
            self.publish_status
        )

        self.get_logger().info("Sensor Node started")

        self.get_logger().info(
            "Waiting for 3D LiDAR data on "
            "/simulation/lidar_points/points"
        )

    # =========================================================
    # LIDAR CALLBACK
    # =========================================================

    def lidar_callback(self, msg: PointCloud2):

        points = self.get_points(msg)

        # -----------------------------------------------------
        # AUCUNE DONNÉE LiDAR
        # -----------------------------------------------------

        if not points:

            self.get_logger().warn(
                "PointCloud2 contains no valid points."
            )

            # Fail-safe :
            # aucune donnée LiDAR = état non sûr.
            #
            # Le DecisionNode verra :
            # ground_detected = false
            #
            # et pourra donc commander STOP.

            sensor_msg = Sensor()

            sensor_msg.ground_detected = False
            sensor_msg.obstacle_detected = False

            sensor_msg.edge_detected = False
            sensor_msg.slope_detected = False

            sensor_msg.ground_distance = 0.0

            sensor_msg.front_distance = 0.0
            sensor_msg.left_distance = 0.0
            sensor_msg.right_distance = 0.0

            sensor_msg.path_left = False
            sensor_msg.path_center = False
            sensor_msg.path_right = False

            sensor_msg.obstacle_start_angle = 0.0
            sensor_msg.obstacle_end_angle = 0.0

            self.sensor_publisher_.publish(
                sensor_msg
            )

            return

        # -----------------------------------------------------
        # Analyse des points
        # -----------------------------------------------------
        left_points = []
        center_points = []
        right_points = []

        ground_points = []
        navigation_points = []

        obstacle_points = []

        # -----------------------------------------------------
        # Classification des points
        # -----------------------------------------------------
        for x, y, z in points:

            # -------------------------------------------------
            # Distance horizontale
            # -------------------------------------------------
            horizontal_distance = math.sqrt(x * x + y * y)

            # -------------------------------------------------
            # Détection du sol
            # -------------------------------------------------
            if z <= self.ground_z_threshold:

                ground_points.append((x, y, z))

                # IMPORTANT :
                #
                # Un point classifié comme sol ne doit PAS
                # participer aux distances de navigation.
                #
                # Sinon le sol pourrait produire :
                #
                # front_distance = 0.33 m
                #
                # alors qu'il n'y a aucun obstacle.
                continue

            # -------------------------------------------------
            # Point utilisable pour la navigation
            # -------------------------------------------------
            navigation_points.append((x, y, z))

            # -------------------------------------------------
            # Détection obstacle
            # -------------------------------------------------
            if (x > 0.0 and horizontal_distance <= self.obstacle_threshold):
                obstacle_points.append((x, y, z))

            # -------------------------------------------------
            # Angle horizontal
            # -------------------------------------------------
            angle = math.atan2(y, x)

            # -------------------------------------------------
            # Zone droite
            # -------------------------------------------------
            if -0.5 <= angle < -0.166:
                right_points.append((x, y, z))

            # -------------------------------------------------
            # Zone centrale
            # -------------------------------------------------

            elif -0.166 <= angle <= 0.166:

                center_points.append(
                    (x, y, z)
                )

            # -------------------------------------------------
            # Zone gauche
            # -------------------------------------------------

            elif 0.166 < angle <= 0.5:

                left_points.append(
                    (x, y, z)
                )

        # -----------------------------------------------------
        # Distances par zone
        # -----------------------------------------------------

        right_distance = self.get_min_horizontal_distance(
            right_points
        )

        center_distance = self.get_min_horizontal_distance(
            center_points
        )

        left_distance = self.get_min_horizontal_distance(
            left_points
        )

        # -----------------------------------------------------
        # Obstacle
        # -----------------------------------------------------

        obstacle_detected = (
            len(obstacle_points) > 0
        )

        # -----------------------------------------------------
        # Chemins libres
        # -----------------------------------------------------

        path_left = (
            left_distance > self.obstacle_threshold
        )

        path_center = (
            center_distance > self.obstacle_threshold
        )

        path_right = (
            right_distance > self.obstacle_threshold
        )

        # -----------------------------------------------------
        # Sol
        # -----------------------------------------------------

        ground_detected = (
            len(ground_points) > 0
        )

        ground_distance = (
            self.get_min_horizontal_distance(
                ground_points
            )
            if ground_points
            else 0.0
        )

        # -----------------------------------------------------
        # Edge detection
        # -----------------------------------------------------

        edge_detected = False

        # -----------------------------------------------------
        # Slope detection
        # -----------------------------------------------------

        slope_detected = False

        # -----------------------------------------------------
        # Construction du message
        # -----------------------------------------------------

        sensor_msg = Sensor()

        sensor_msg.ground_detected = (
            ground_detected
        )

        sensor_msg.obstacle_detected = (
            obstacle_detected
        )

        sensor_msg.edge_detected = (
            edge_detected
        )

        sensor_msg.slope_detected = (
            slope_detected
        )

        sensor_msg.ground_distance = (
            float(ground_distance)
        )

        sensor_msg.front_distance = (
            float(center_distance)
        )

        sensor_msg.left_distance = (
            float(left_distance)
        )

        sensor_msg.right_distance = (
            float(right_distance)
        )

        sensor_msg.path_left = (
            path_left
        )

        sensor_msg.path_center = (
            path_center
        )

        sensor_msg.path_right = (
            path_right
        )

        # -----------------------------------------------------
        # Angles d'obstacle
        # -----------------------------------------------------

        sensor_msg.obstacle_start_angle = 0.0
        sensor_msg.obstacle_end_angle = 0.0

        # -----------------------------------------------------
        # Publication
        # -----------------------------------------------------

        self.sensor_publisher_.publish(
            sensor_msg
        )

    # =========================================================
    # EXTRACTION DES POINTS
    # =========================================================

    def get_points(self, msg: PointCloud2):

        points = []

        try:

            point_generator = point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            )

            for point in point_generator:

                x = float(point[0])
                y = float(point[1])
                z = float(point[2])

                if not math.isfinite(x):
                    continue

                if not math.isfinite(y):
                    continue

                if not math.isfinite(z):
                    continue

                points.append(
                    (x, y, z)
                )

        except Exception as error:

            self.get_logger().error(
                f"Failed to read PointCloud2: {error}"
            )

        return points

    # =========================================================
    # DISTANCE HORIZONTALE MINIMUM
    # =========================================================

    def get_min_horizontal_distance(self, points):

        # Aucun point dans cette direction.
        #
        # On considère alors que la zone est libre
        # jusqu'à la portée maximale du LiDAR.

        if not points:
            return 15.0

        distances = []

        for x, y, z in points:

            # Pour la navigation, on ne tient pas compte
            # de Z.
            #
            # On veut savoir :
            #
            # "À quelle distance horizontale se trouve
            #  l'objet ?"

            distance = math.sqrt(
                x * x +
                y * y
            )

            if math.isfinite(distance):

                distances.append(
                    distance
                )

        if not distances:
            return 15.0

        return min(distances)

    # =========================================================
    # STATUS / HEARTBEAT
    # =========================================================

    def publish_status(self):

        status_msg = ComponentStatus()

        status_msg.component = (
            "sensor_node"
        )

        status_msg.level = (
            ComponentStatus.OK
        )

        status_msg.reason = (
            "Sensor status ok"
        )

        self.status_publisher_.publish(
            status_msg
        )


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = SensorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()