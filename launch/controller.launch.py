import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("ros2_jaco_controller"), "config", "params.yaml"
    )
    # ! Errors here
    jaco_controller_node = Node(
        package="ros2_jaco_controller",
        executable="all_purpose_controller.py",
        name="jaco_controller",
        arguments=[("__log_level:=debug")],
        output="screen",
        parameters=[config],

    )
    launch_description.add_action(jaco_controller_node)

    print("ROS2 Jaco Controller is Running...")
    print(f"params.yaml: {config}")

    return launch_description
