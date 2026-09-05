from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_python",
            executable="sensor_node",
            name='sensor_node',
            output="screen",
        ),
        Node(
            package="robot_python",
            executable="battery_node",
            name='battery_node',
            output="screen",
        ),
        Node(
            package="robot_python",
            executable="camera_node",
            name='camera_node',
            output="screen",
        ),
        Node(
            package="robot_python",
            executable="decision_node",
            name='decision_node',
            output="screen",
        ),
    ])