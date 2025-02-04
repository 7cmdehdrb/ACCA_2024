from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="localization",
                executable="map_odom_tf_publisher2",
                name="map_odom_tf_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("localization"),
                        "params",
                        "map_odom_tf_publisher2.yaml",
                    )
                ],
            ),
        ]
    )
