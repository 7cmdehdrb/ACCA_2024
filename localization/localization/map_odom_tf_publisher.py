import rclpy
from rclpy.node import Node
import rclpy.duration
import rclpy.time
from rclpy.qos import qos_profile_system_default
import numpy as np
import math as m
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import threading
import time

from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped as PS
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray


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

        class Odom_State(object):
            def __init__(self):
                self.state = [0.0, 0.0]
                self.time = time.time()

            def update_state(self, new_pose):
                vel_x = new_pose.twist.twist.linear.x
                vel_y = new_pose.twist.twist.linear.y

                linear_vel = m.sqrt(vel_x**2 + vel_y**2)

                angular_vel = new_pose.twist.twist.angular.z

                self.state = np.array([linear_vel, angular_vel])
                self.time = time.time()

        class PCL_State(object):
            def __init__(self):
                self.last_pose = PoseWithCovarianceStamped()
                self.last_time = time.time()
                self.state = [0.0, 0.0]
                self.time = time.time()

            def update_state(self, new_pose):
                current_time = time.time()

                dt = current_time - self.last_time

                current_x = new_pose.pose.pose.position.x
                current_y = new_pose.pose.pose.position.y
                current_quat = new_pose.pose.pose.orientation
                _, _, current_yaw = euler_from_quaternion(
                    [current_quat.x, current_quat.y, current_quat.z, current_quat.w]
                )

                last_x = self.last_pose.pose.pose.position.x
                last_y = self.last_pose.pose.pose.position.y
                last_quat = self.last_pose.pose.pose.orientation
                _, _, last_yaw = euler_from_quaternion(
                    [last_quat.x, last_quat.y, last_quat.z, last_quat.w]
                )

                dist_x = current_x - last_x
                dist_y = current_y - last_y
                dist = m.sqrt(dist_x**2 + dist_y**2)

                angular_dist = current_yaw - last_yaw

                linear_vel = dist / dt
                angular_vel = angular_dist / dt

                self.state = [linear_vel, angular_vel]
                self.time = time.time()

                self.last_pose = new_pose
                self.last_time = current_time

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

        # TF

        self.buffer = Buffer(node=self, cache_time=rclpy.duration.Duration(seconds=0.1))
        self.tf_listener = tf2_ros.TransformListener(
            self.buffer, self, qos=qos_profile_system_default
        )
        self.tf_publisher = tf2_ros.TransformBroadcaster(
            self, qos=qos_profile_system_default
        )

        # Topic

        self.err_pub = self.create_publisher(
            Float32MultiArray, "/tf_err", qos_profile=qos_profile_system_default
        )

        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            qos_profile=qos_profile_system_default,
        )

        self.pcl_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.map_topic,
            callback=self.pcl_callback,
            qos_profile=qos_profile_system_default,
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            callback=self.odom_callback,
            qos_profile=qos_profile_system_default,
        )

        self.score_subscriber = self.create_subscription(
            Float32,
            "/pcl_score",
            callback=self.score_callback,
            qos_profile=qos_profile_system_default,
        )

        # Topic Variables

        self.rate = self.create_rate(3.0)

        self.pcl = PoseWithCovarianceStamped()
        self.odom = Odometry()
        self.score = 0.0

        self.stack = 0

        self.pcl_state = PCL_State()
        self.odom_state = Odom_State()

        self.linear_err = 0.0
        self.angular_err = 0.0

        self.tf_msg = tf2_ros.TransformStamped()

        self.last_time = time.time()
        self.i_err = 0.0

        self.err_stack = 0

    def get_data(self):
        return "linear: {}\tangular: {}\ti_linear_err: {}\tscore: {}".format(
            round(self.linear_err, 3),
            round(self.angular_err, 3),
            round(self.i_err, 3),
            round(self.score, 3),
        )

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_time

        # Calculate Errors

        temp_linear_err = self.pcl_state.state[0] - self.odom_state.state[0]
        self.linear_err = abs(temp_linear_err)
        self.angular_err = abs(self.pcl_state.state[1] - self.odom_state.state[1])

        self.i_err += temp_linear_err * dt

        time_difference = abs(self.odom_state.time - self.pcl_state.time)

        self.last_time = current_time

        # Logic

        # Absolutely Fraud Data
        if self.score > 3.0:
            return

        # if time_difference > 2.0:
        #     return

        # reinitialize i_err (To prevent drift i_err)
        if self.linear_err < 0.1:
            self.i_err = 0.0

        # if self.linear_err < 1.5 and self.angular_err < 0.5 and abs(self.i_err) < 1.0:
        if True:
            self.update_tf()
            self.err_stack = 0
            return

        self.get_logger().warn("Cannot trust TF data!")
        self.err_stack += 1

        try:
            if self.err_stack > 10:
                if self.tf_msg.header.frame_id == "map":
                    self.err_stack = 0
                    self.reinitialize()

        except Exception as ex:
            self.get_logger().warn(str(ex))

    def update_tf(self):
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

    def publish_tf(self):
        self.tf_publisher.sendTransform(self.tf_msg)

    def reinitialize(self):
        if self.buffer.can_transform(
            "map",
            "odom",
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=0.1),
        ):
            ps_odom = PS()
            ps_odom.header = self.odom.header
            ps_odom.pose = self.odom.pose.pose

            ps_odom_on_map = self.buffer.transform(
                ps_odom, "map", rclpy.duration.Duration(seconds=0.1)
            )

            pose_stamped_cov_map = PoseWithCovarianceStamped()
            pose_stamped_cov_map.header = ps_odom_on_map.header
            pose_stamped_cov_map.pose.pose = ps_odom_on_map.pose

            self.get_logger().info("Reinitializing...")
            self.initpose_pub.publish(pose_stamped_cov_map)

        else:
            raise Exception("Cannot lookup transform")

    def pcl_callback(self, msg):
        self.pcl = msg

        self.pcl_state.update_state(msg)

        self.update()

        self.publish_tf()

        self.get_logger().info(self.get_data())

    def odom_callback(self, msg):
        self.odom = msg

        self.odom_state.update_state(msg)

    def score_callback(self, msg):
        self.score = msg.data


def main():
    rclpy.init(args=None)

    node = Map_Odom_TF_Publisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
