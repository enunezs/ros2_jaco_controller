import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_description = LaunchDescription()

    # Controller
    jaco_controller_node = Node(
        package="ros2_jaco_controller",
        executable="all_purpose_controller.py",
        name="jaco_main_controller",
        parameters=["jacoarm-ros2/config/params.yaml"],
    )
    launch_description.add_action(jaco_controller_node)

    return launch_description
