import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
import math as m
import numpy as np
import time
from std_msgs.msg import Header, Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


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


class MatchingChecker(Node):
    def __init__(self):
        super().__init__("matching_checker_node")

        self.odom = Odometry()
        self.pcl = PoseWithCovarianceStamped()
        self.predicted_pcl = PoseWithCovarianceStamped()

        self.current_time = time.time()
        self.last_time = time.time()

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odometry/kalman",
            callback=self.odom_update_state,
            qos_profile=qos_profile_system_default,
        )
        self.pcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/pcl_pose",
            callback=self.pcl_update_state,
            qos_profile=qos_profile_system_default,
        )
        self.predicted_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/predicted_pcl",
            qos_profile=qos_profile_system_default,
        )
        self.test_pub = self.create_publisher(
            Float32MultiArray, "/test", qos_profile=qos_profile_system_default
        )

    def odom_update_state(self, msg):
        self.odom = msg

    def pcl_update_state(self, msg):
        self.predicted_pcl = self.check_validation(pcl_pose=self.pcl)

        self.pcl = msg

        self.test_pub.publish(
            Float32MultiArray(
                data=[
                    self.predicted_pcl.pose.pose.position.x
                    - self.pcl.pose.pose.position.x,
                    self.predicted_pcl.pose.pose.position.y
                    - self.pcl.pose.pose.position.y,
                ]
            )
        )

        self.get_logger().info(
            "{}\t{}".format(
                self.predicted_pcl.pose.pose.position.x - self.pcl.pose.pose.position.x,
                self.predicted_pcl.pose.pose.position.y - self.pcl.pose.pose.position.y,
            )
        )

    def check_validation(self, pcl_pose):
        self.current_time = time.time()

        dt = self.current_time - self.last_time

        odom_linear_vel = m.sqrt(
            (self.odom.twist.twist.linear.x**2) + (self.odom.twist.twist.linear.y**2)
        )
        odom_angular_vel = self.odom.twist.twist.angular.z

        self.get_logger().info(str(odom_linear_vel))

        predicted_pcl_pose = PoseWithCovarianceStamped()
        predicted_pcl_pose.header = Header(frame_id="map", stamp=Time().to_msg())

        _, _, pcl_yaw = euler_from_quaternion(
            [
                pcl_pose.pose.pose.orientation.x,
                pcl_pose.pose.pose.orientation.y,
                pcl_pose.pose.pose.orientation.z,
                pcl_pose.pose.pose.orientation.w,
            ]
        )

        predicted_pcl_yaw = pcl_yaw + (odom_angular_vel * dt)

        predicted_pcl_quat = quaternion_from_euler(0.0, 0.0, predicted_pcl_yaw)

        predicted_pcl_pose.pose.pose.position.x = pcl_pose.pose.pose.position.x + (
            odom_linear_vel * m.cos(predicted_pcl_yaw) * dt
        )
        predicted_pcl_pose.pose.pose.position.y = pcl_pose.pose.pose.position.y + (
            odom_linear_vel * m.sin(predicted_pcl_yaw) * dt
        )

        # predicted_pcl_pose.pose.pose.position.z = pcl_pose.pose.pose.position.z

        predicted_pcl_pose.pose.pose.orientation.x = predicted_pcl_quat[0]
        predicted_pcl_pose.pose.pose.orientation.y = predicted_pcl_quat[1]
        predicted_pcl_pose.pose.pose.orientation.z = predicted_pcl_quat[2]
        predicted_pcl_pose.pose.pose.orientation.w = predicted_pcl_quat[3]

        # self.predicted_pub.publish(predicted_pcl_pose)

        self.last_time = self.current_time

        return predicted_pcl_pose


def main():
    rclpy.init(args=None)

    node = MatchingChecker()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
