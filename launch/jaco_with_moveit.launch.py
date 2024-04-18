import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_description = LaunchDescription()

    # Robot connection
    kinova_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("kinova_bringup"), "launch"),
                "/kinova_robot_launch.py",
            ]
        )
    )
    launch_description.add_action(kinova_robot_launch)

    # Moveit2
    moveit_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("kinova_bringup"), "launch"),
                "/moveit_robot_launch.py",
            ]
        )
    )
    launch_description.add_action(moveit_robot_launch)

    print("Pupil Neon Lunch is Running...")

    return launch_description
