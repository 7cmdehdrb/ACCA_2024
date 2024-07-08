import rclpy
from rclpy.node import Node
import rclpy.duration
import rclpy.time
import numpy as np

# from tf2_ros import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, TransformStamped
import tf2_geometry_msgs


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return qx, qy, qz, qw


class Test(Node):
    def __init__(self):
        super().__init__("tf_listener_node")

        self.target_frame = "odom"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def listen(self):
        # Create map_wrt_frame transform
        map_wrt_frame = PoseStamped()

        map_wrt_frame.header.stamp = rclpy.time.Time().to_msg()
        map_wrt_frame.header.frame_id = "base_link"

        pose = Pose()

        pose.position.x = 5.0
        pose.position.y = 7.0
        pose.position.z = 0.0

        x, y, z, w = quaternion_from_euler(0.0, 0.0, 1.0)
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        map_wrt_frame.pose = pose

        # Lookup frame_wrt_odom transform
        try:
            frame_wrt_odom = self.tf_buffer.lookup_transform(
                "base_link",
                self.target_frame,
                rclpy.time.Time(),
            )
            # map_wrt_frame(base_link) -> frame_wrt_odom(odom)
        except Exception:
            self.get_logger().warn("Transform lookup failed!")
            frame_wrt_odom = None

        # Convert frame_wrt_odom to Eigen-like Matrix (not directly available in tf2_ros Python)
        # You may need to manually extract translation and rotation and convert them as needed

        # Transform map_wrt_frame to map_wrt_odom
        if frame_wrt_odom:
            try:
                map_wrt_odom = self.tf_buffer.transform(map_wrt_frame, "odom")

            except Exception as ex:
                print(ex)
                self.get_logger().warn("Transformation failed!")
                map_wrt_odom = None

            # Invert map_wrt_odom to get odom_wrt_map
            if map_wrt_odom:
                map_wrt_odom.pose.orientation.x *= -1
                map_wrt_odom.pose.orientation.y *= -1
                map_wrt_odom.pose.orientation.z *= -1
                map_wrt_odom.pose.orientation.w *= -1

                odom_trans = PoseStamped()
                odom_trans

                # odom_trans = TransformStamped()
                # odom_trans.transform = map_wrt_odom.transform
                # odom_trans.header.stamp = stamp
                # odom_trans.header.frame_id = "map"
                # odom_trans.child_frame_id = robot_odom_frame_id

            # Publish odom_trans (not shown in this example)

        # You will need to add the code for broadcasting the transform

        # Spin to keep the node alive


def main():
    rclpy.init()
    node = Test()
    try:
        rclpy.spin_once(node)
        while rclpy.ok():
            node.listen()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


main()
