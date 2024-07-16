import rclpy
import rclpy.duration
from rclpy.node import Node
import numpy as np
import math as m
import rclpy.time
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import threading

from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped as PS
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


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


class Map_Odom_TF_Publisher(Node):
    def __init__(self):
        super().__init__("map_odom_tf_node")

        self.test = False

        self.params = self.declare_parameters(
            namespace="",
            parameters=(
                ("/map_topic", "/pcl_pose"),
                ("/odom_topic", "/odometry/kalman"),
            ),
        )

        self.map_topic = (
            self.get_parameter("/map_topic").get_parameter_value().string_value
        )
        self.odom_topic = (
            self.get_parameter("/odom_topic").get_parameter_value().string_value
        )

        self.tf_publisher = tf2_ros.TransformBroadcaster(self, qos=10)

        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)

        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", qos_profile=10
        )

        self.pcl_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.map_topic,
            callback=self.pcl_callback,
            qos_profile=10,
        )

        self.odom_subscriber = self.create_subscription(
            Odometry, self.odom_topic, callback=self.odom_callback, qos_profile=10
        )

        self.score_subscriber = self.create_subscription(
            Float32, "/pcl_score", callback=self.score_callback, qos_profile=10
        )

        self.pcl = PoseWithCovarianceStamped()
        self.odom = Odometry()
        self.score = 0.0

        self.tf_msg = tf2_ros.TransformStamped()

    def pcl_callback(self, msg):
        self.pcl = msg

        self.publish_tf()

    def odom_callback(self, msg):
        self.odom = msg

    def score_callback(self, msg):
        self.score = msg.data

    def publish_tf(self):
        tf_msg = tf2_ros.TransformStamped()

        tf_msg.header.frame_id = "map"
        tf_msg.header.stamp = rclpy.time.Time().to_msg()
        tf_msg.child_frame_id = "odom"

        p1 = self.odom.pose.pose
        _, _, yaw1 = euler_from_quaternion(
            [
                p1.orientation.x,
                p1.orientation.y,
                p1.orientation.z,
                p1.orientation.w,
            ]
        )

        p2 = self.pcl.pose.pose
        _, _, yaw2 = euler_from_quaternion(
            [
                p2.orientation.x,
                p2.orientation.y,
                p2.orientation.z,
                p2.orientation.w,
            ]
        )

        trans = [
            p2.position.x
            - (
                (p1.position.x * m.cos(yaw2 - yaw1))
                - (p1.position.y * m.sin(yaw2 - yaw1))
            ),
            p2.position.y
            - (
                (p1.position.x * m.sin(yaw2 - yaw1))
                + (p1.position.y * m.cos(yaw2 - yaw1))
            ),
        ]

        rot = quaternion_from_euler(0.0, 0.0, yaw2 - yaw1)

        tf_msg.transform.translation.x = trans[0]
        tf_msg.transform.translation.y = trans[1]

        tf_msg.transform.rotation.x = rot[0]
        tf_msg.transform.rotation.y = rot[1]
        tf_msg.transform.rotation.z = rot[2]
        tf_msg.transform.rotation.w = rot[3]

        self.tf_msg = tf_msg

        self.tf_publisher.sendTransform(self.tf_msg)


def main():
    rclpy.init(args=None)

    node = Map_Odom_TF_Publisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
