import rclpy
import rclpy.time
import tf2_ros
import numpy as np
import math as m
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


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

    return [qx, qy, qz, qw]


if __name__ == "__main__":
    rclpy.init(args=None)

    node = rclpy.create_node("tf_test_node")

    pub1 = node.create_publisher(PoseStamped, "p1", qos_profile=1)
    pub2 = node.create_publisher(PoseStamped, "p2", qos_profile=1)
    tf_pub = tf2_ros.TransformBroadcaster(node, qos=1)

    t = -10.0

    r = node.create_rate(1)

    try:
        while rclpy.ok():
            rclpy.spin_once(node)

            print("@")

            x1 = np.array([1.0, 3.0, 0.0, 1.0])
            x2 = np.array([5.0, 7.0, 0.0, 1.0])
            deg1 = 10.0
            deg2 = t
            deg = deg2 - deg1

            p1 = PoseStamped()

            quat1 = quaternion_from_euler(0.0, 0.0, m.radians(deg1))
            p1.header.frame_id = "odom"
            p1.header.stamp = rclpy.time.Time().to_msg()
            p1.pose.position.x = x1[0]
            p1.pose.position.y = x1[1]
            p1.pose.position.z = x1[2]
            p1.pose.orientation.x = quat1[0]
            p1.pose.orientation.y = quat1[1]
            p1.pose.orientation.z = quat1[2]
            p1.pose.orientation.w = quat1[3]

            # =======================

            p2 = PoseStamped()

            quat2 = quaternion_from_euler(0.0, 0.0, m.radians(deg2))
            p2.header.frame_id = "map"
            p2.header.stamp = rclpy.time.Time().to_msg()
            p2.pose.position.x = x2[0]
            p2.pose.position.y = x2[1]
            p2.pose.position.z = x2[2]
            p2.pose.orientation.x = quat2[0]
            p2.pose.orientation.y = quat2[1]
            p2.pose.orientation.z = quat2[2]
            p2.pose.orientation.w = quat2[3]

            pub1.publish(p1)
            pub2.publish(p2)

            dx = x2[0] - x1[0]
            dy = x2[1] - x1[1]

            trans = (
                x2[0]
                - ((x1[0] * m.cos(m.radians(deg)) - (x1[1] * m.sin(m.radians(deg))))),
                x2[1]
                - ((x1[0] * m.sin(m.radians(deg)) + (x1[1] * m.cos(m.radians(deg))))),
                0.0,
            )

            rot = quaternion_from_euler(0.0, 0.0, m.radians(deg))

            tf_msg = tf2_ros.TransformStamped()

            tf_msg.header.frame_id = "map"
            tf_msg.header.stamp = rclpy.time.Time().to_msg()
            tf_msg.child_frame_id = "odom"

            tf_msg.transform.translation.x = trans[0]
            tf_msg.transform.translation.y = trans[1]
            tf_msg.transform.translation.z = trans[2]

            tf_msg.transform.rotation.x = rot[0]
            tf_msg.transform.rotation.y = rot[1]
            tf_msg.transform.rotation.z = rot[2]
            tf_msg.transform.rotation.w = rot[3]

            tf_pub.sendTransform(tf_msg)

            print(tf_msg)

            t += 10.0

            r.sleep()

    except Exception as ex:
        print(ex)
    finally:
        node.destroy_node()
        rclpy.shutdown()
