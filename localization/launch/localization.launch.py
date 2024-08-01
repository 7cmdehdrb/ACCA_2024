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
    kalman_localization = launch_ros.actions.Node(
        package="localization",
        executable="kalman_localization",
        name="kalman_localization_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("localization"),
                "params",
                "kalman_localization.yaml",
            )
        ],
    )

    tf_localization = launch_ros.actions.Node(
        package="localization",
        executable="map_odom_tf_publisher",
        name="map_odom_tf_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("localization"),
                "params",
                "map_odom_tf_publisher.yaml",
            )
        ],
    )

    map_server = launch_ros.actions.Node(
        package="localization",
        executable="real_time_map_server",
        name="real_time_map_server_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("localization"),
                "params",
                "real_time_map_server.yaml",
            )
        ],
    )

    return LaunchDescription(
        [
            kalman_localization,
            tf_localization,
            # map_server,
        ]
    )
