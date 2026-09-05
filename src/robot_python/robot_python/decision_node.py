import rclpy
from rclpy.node import Node

from robot_interfaces.msg import Sensor, Battery, RobotCommand


class DecisionNode(Node):

    def __init__(self):
        super().__init__(node_name="decision_node")

        self.publisher_ = self.create_publisher(
            RobotCommand,
            "/robot/cmd",
            10
        )

        # =========================
        # Sensor state
        # =========================

        self.ground_detected = True
        self.obstacle_detected = False
        self.edge_detected = False
        self.slope_detected = False

        self.ground_distance = 0.0

        self.front_distance = 15.0
        self.left_distance = 15.0
        self.right_distance = 15.0

        self.path_left = True
        self.path_center = True
        self.path_right = True

        self.obstacle_start_angle = 0.0
        self.obstacle_end_angle = 0.0

        # =========================
        # Battery state
        # =========================

        self.battery = 100.0
        self.charging = False

        # =========================
        # Decision state
        # =========================

        self.current_command = None

        # =========================
        # Subscriptions
        # =========================

        self.sensor_subscription = self.create_subscription(
            Sensor,
            "/robot/sensor",
            self.sensor_callback,
            10
        )

        self.battery_subscription = self.create_subscription(
            Battery,
            "/robot/battery",
            self.battery_callback,
            10
        )

        self.get_logger().info("Decision Node started")

    # =========================================================
    # Sensor callback
    # =========================================================

    def sensor_callback(self, msg):

        self.ground_detected = msg.ground_detected
        self.obstacle_detected = msg.obstacle_detected
        self.edge_detected = msg.edge_detected
        self.slope_detected = msg.slope_detected

        self.ground_distance = msg.ground_distance

        self.front_distance = msg.front_distance
        self.left_distance = msg.left_distance
        self.right_distance = msg.right_distance

        self.path_left = msg.path_left
        self.path_center = msg.path_center
        self.path_right = msg.path_right

        self.obstacle_start_angle = msg.obstacle_start_angle
        self.obstacle_end_angle = msg.obstacle_end_angle

        self.make_decision()

    # =========================================================
    # Battery callback
    # =========================================================

    def battery_callback(self, msg):

        self.battery = msg.percentage
        self.charging = msg.charging

        self.make_decision()

    # =========================================================
    # Decision engine
    # =========================================================

    def make_decision(self):

        # -----------------------------------------------------
        # 1. Battery
        # -----------------------------------------------------

        if self.battery < 5.0:
            self.publish_command(
                RobotCommand.GO_CHARGING,
                0.0,
                0.0
            )
            return

        if self.battery < 20.0:
            self.publish_command(
                RobotCommand.LOW_BATTERY,
                0.0,
                0.0
            )
            return

        # -----------------------------------------------------
        # 2. Safety
        # -----------------------------------------------------

        if not self.ground_detected:
            self.publish_command(
                RobotCommand.STOP,
                0.0,
                0.0
            )
            return

        if self.edge_detected:
            self.publish_command(
                RobotCommand.STOP,
                0.0,
                0.0
            )
            return

        if self.slope_detected:
            self.publish_command(
                RobotCommand.STOP,
                0.0,
                0.0
            )
            return

        # -----------------------------------------------------
        # 3. Obstacle detected
        # -----------------------------------------------------

        if self.obstacle_detected:

            self.handle_obstacle()
            return

        # -----------------------------------------------------
        # 4. No obstacle
        # -----------------------------------------------------

        self.follow_open_path()

    # =========================================================
    # Obstacle avoidance
    # =========================================================

    def handle_obstacle(self):

        # Obstacle directly in front.
        # Look for the safest side.

        if self.path_left and self.path_right:

            if self.left_distance > self.right_distance:

                self.publish_command(
                    RobotCommand.TURN_LEFT,
                    0.0,
                    0.8
                )

            else:

                self.publish_command(
                    RobotCommand.TURN_RIGHT,
                    0.0,
                    -0.8
                )

            return

        # Left side is available

        if self.path_left:

            self.publish_command(
                RobotCommand.TURN_LEFT,
                0.0,
                0.8
            )

            return

        # Right side is available

        if self.path_right:

            self.publish_command(
                RobotCommand.TURN_RIGHT,
                0.0,
                -0.8
            )

            return

        # Nothing available

        self.publish_command(
            RobotCommand.TURN_AROUND,
            0.0,
            1.5
        )

    # =========================================================
    # Open path navigation
    # =========================================================

    def follow_open_path(self):

        # If the center is free,
        # continue forward.

        if self.path_center:

            self.publish_command(
                RobotCommand.FORWARD,
                0.4,
                0.0
            )

            return

        # Center is blocked but sensor does not classify
        # it as a complete obstacle yet.
        # Choose the more open side.

        if self.left_distance > self.right_distance:

            self.publish_command(
                RobotCommand.TURN_LEFT,
                0.15,
                0.5
            )

        elif self.right_distance > self.left_distance:

            self.publish_command(
                RobotCommand.TURN_RIGHT,
                0.15,
                -0.5
            )

        else:

            self.publish_command(
                RobotCommand.STOP,
                0.0,
                0.0
            )

    # =========================================================
    # Publish command
    # =========================================================

    def publish_command(
        self,
        command,
        linear_speed,
        angular_speed
    ):

        # Do not republish identical commands.

        if (
            self.current_command is not None
            and self.current_command.command == command
            and self.current_command.linear_speed == linear_speed
            and self.current_command.angular_speed == angular_speed
        ):
            return

        msg = RobotCommand()

        msg.command = command
        msg.linear_speed = linear_speed
        msg.angular_speed = angular_speed

        self.publisher_.publish(msg)

        self.current_command = msg

        self.get_logger().info(
            f"Command: {self.command_to_string(command)} "
            f"| linear={linear_speed:.2f} "
            f"| angular={angular_speed:.2f}"
        )

    # =========================================================
    # Command name
    # =========================================================

    def command_to_string(self, command):

        commands = {
            RobotCommand.STOP: "STOP",
            RobotCommand.FORWARD: "FORWARD",
            RobotCommand.REVERSE: "REVERSE",
            RobotCommand.TURN_LEFT: "TURN_LEFT",
            RobotCommand.TURN_RIGHT: "TURN_RIGHT",
            RobotCommand.TURN_AROUND: "TURN_AROUND",
            RobotCommand.LOW_BATTERY: "LOW_BATTERY",
            RobotCommand.GO_CHARGING: "GO_CHARGING",
        }

        return commands.get(
            command,
            "UNKNOWN"
        )


def main(args=None):

    rclpy.init(args=args)

    node = DecisionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()