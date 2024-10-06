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
    ld = launch.LaunchDescription()

    osm = LaunchDescription(
        [
            launch_ros.actions.Node(
                package="localization",
                executable="osm",
                name="osm_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("localization"),
                        "params",
                        "osm.yaml",
                    )
                ],
            ),
        ]
    )

    gps_localizer = LaunchDescription(
        [
            launch_ros.actions.Node(
                package="localization",
                executable="gps_localization",
                name="gps_localization_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("localization"),
                        "params",
                        "gps_localization.yaml",
                    )
                ],
            ),
        ]
    )

    ld.add_action(osm)
    ld.add_action(gps_localizer)

    return ld
