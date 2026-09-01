import rclpy
from rclpy.node import Node

from robot_interfaces.msg import Distance, Battery, RobotCommand

class DecisionNode(Node):

    def __init__(self):
        super().__init__(node_name="decision_node")

        self.publisher_ = self.create_publisher(
            RobotCommand,
            "/robot/cmd",
            10
        )

        self.distance = 0.0
        self.obstacle = False

        self.battery = 100.0
        self.charging = False

        self.current_command = None

        self.distance_subscription = self.create_subscription(
            Distance,
            "/robot/distance",
            self.distance_callback,
            10
        )

        self.battery_subscription = self.create_subscription(
            Battery,
            "/robot/battery",
            self.battery_callback,
            10
        )

        self.get_logger().info("Decision Node started")

    def distance_callback(self, msg):
        self.distance = msg.distance
        self.obstacle = msg.obstacle

        self.make_decision()

    def battery_callback(self, msg):
        self.battery = msg.percentage
        self.charging = msg.charging

        self.make_decision()
    
    def make_decision(self):

        if self.battery < 20.0 and self.battery >= 5.0:
            new_command = RobotCommand.LOW_BATTERY

        elif self.battery < 5.0:
            new_command = RobotCommand.GO_CHARGING

        elif self.obstacle:
            new_command = RobotCommand.STOP
        else:
            new_command = RobotCommand.FORWARD

        if new_command == self.current_command:
            return
        
        self.current_command = new_command

        msg = RobotCommand()
        msg.command = new_command

        self.publisher_.publish(msg)

        COMMAND_NAMES = {
            RobotCommand.STOP: "STOP",
            RobotCommand.FORWARD: "FORWARD",
            RobotCommand.LOW_BATTERY: "LOW_BATTERY",
            RobotCommand.GO_CHARGING: "GO_CHARGING",
        }

        self.get_logger().info(
            f"Battery={self.battery:.1f}% | "
            f"Distance={self.distance:.2f}m | "
            f"Obstacle={self.obstacle} -> "
            f"{COMMAND_NAMES[new_command]}"
        )

def main(args=None):

    rclpy.init(args=args)

    node = DecisionNode()
    rclpy.spin(node=node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()