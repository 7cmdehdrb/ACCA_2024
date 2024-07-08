import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__("tf_listener")

        # Declare and acquire `target_frame` parameter
        self.target_frame = (
            self.declare_parameter("target_frame", "odom")
            .get_parameter_value()
            .string_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        print("@")
        from_frame_rel = self.target_frame
        to_frame_rel = "base_link"

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, rclpy.time.Time()
            )
            print(t)
        except Exception as ex:
            self.get_logger().info(
                f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
            )


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


main()
