import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("jacoarm-ros2"), "config", "params.yaml"
    )

    jaco_controller_node = Node(
        package="jacoarm-ros2",
        executable="new_robot_controller.py",
        name="jaco_controller",
        arguments=[("__log_level:=debug")],
        output="screen",
        parameters=[config],
    )
    launch_description.add_action(jaco_controller_node)

    mode_manager_node = Node(
        package="jacoarm-ros2",
        executable="mode_manager.py",
        name="mode_manager",
        arguments=[("__log_level:=debug")],
        output="screen",
        parameters=[config],
    )
    launch_description.add_action(mode_manager_node)
    
    print("ROS2 Jaco Controller is Running...")
    print(f"params.yaml: {config}")

    return launch_description
