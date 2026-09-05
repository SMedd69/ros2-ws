import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory(
        "robot_simulation"
    )

    gazebo_package_path = get_package_share_directory(
        "ros_gz_sim"
    )


    # ================================================
    # Fichiers
    # ================================================

    world_file = os.path.join(
        package_path,
        "worlds",
        "test_world.sdf"
    )

    robot_file = os.path.join(
        package_path,
        "urdf",
        "robot.urdf.xacro"
    )


    # ================================================
    # Gazebo
    # ================================================

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_package_path,
                "launch",
                "gz_sim.launch.py"
            )
        ),

        launch_arguments={
            "gz_args": f"-r {world_file}"
        }.items()

    )


    # ================================================
    # Robot State Publisher
    # ================================================

    robot_state_publisher = Node(

        package="robot_state_publisher",

        executable="robot_state_publisher",

        name="robot_state_publisher",

        output="screen",

        parameters=[
            {
                "robot_description": Command([
                    "xacro ",
                    robot_file
                ])
            }
        ]

    )


    # ================================================
    # Spawn du robot
    # ================================================

    spawn_robot = Node(

        package="ros_gz_sim",

        executable="create",

        arguments=[
            "-topic",
            "robot_description",

            "-name",
            "robot",

            "-x",
            "0.0",

            "-y",
            "0.0",

            "-z",
            "0.11"
        ],

        output="screen"

    )


    return LaunchDescription([

        gazebo,

        robot_state_publisher,

        spawn_robot

    ])