import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="mapviz",
                executable="mapviz",
                name="mapviz",
            ),
            launch_ros.actions.Node(
                package="swri_transform_util",
                executable="initialize_origin.py",
                name="initialize_origin",
                # remappings=[
                #     ("fix", "ublox_gps_node/fix"),
                # ],
                parameters=[
                    {"local_xy_frame": "/map"},
                    {"local_xy_origin": "swri"},
                    # {"local_xy_gpsfix_topic", "/ublox_gps_node/fix"}
                    {"local_xy_navsatfix_topic", "/ulbox_gps_node/fix"},
                    # {"local_xy_origins": [29.45196669, -98.61370577, 233.719, 0.0]},
                    {"local_xy_origins": [37.4961657, 126.9570535, 121.228, 0.0]},
                    # {
                    #     "local_xy_origins": [
                    #         {
                    #             "name": "swri",
                    #             "latitude": 29.45196669,
                    #             "longitude": -98.61370577,
                    #             "altitude": 233.719,
                    #             "heading": 0.0,
                    #         },
                    #         {
                    #             "name": "back_40",
                    #             "latitude": 29.447507,
                    #             "longitude": -98.629367,
                    #             "altitude": 200.0,
                    #             "heading": 0.0,
                    #         },
                    #     ]
                    # },
                ],
            ),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="swri_transform",
                arguments=["0", "0", "0", "0", "0", "0", "map", "origin"],
            ),
            # launch_ros.actions.Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="gps_transform",
            #     arguments=["0", "0", "0", "0", "0", "0", "base_link", "gps"],
            # ),
        ]
    )
