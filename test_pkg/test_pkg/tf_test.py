import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped
import numpy as np
import math as m
import tf2_ros
import random


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


def quaternion_inverse(q):
    """Compute the inverse of a quaternion."""
    q_conjugate = np.array([-q[0], -q[1], -q[2], q[3]])
    q_norm = np.dot(q, q)
    q_inv = q_conjugate / q_norm
    return q_inv


def quaternion_multiply(q1, q2):
    """Multiply two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([x, y, z, w])


def quaternion_to_matrix(q):
    """Convert a quaternion into a 4x4 rotation matrix."""
    q = np.array(q, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < np.finfo(float).eps:
        return np.identity(4)

    q *= np.sqrt(2.0 / n)
    q = np.outer(q, q)

    return np.array(
        [
            [1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0], 0.0],
            [q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0], 0.0],
            [q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2], 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


class TF_test(object):
    def __init__(self, node):
        self.node = node

        self.pose_on_odom = PoseStamped()

        self.pose_on_odom.header.frame_id = "odom"
        self.pose_on_odom.pose.position.x = 8.0
        self.pose_on_odom.pose.position.y = 5.0

        self.odom_yaw = -m.pi / 4.0

        ox, oy, oz, ow = quaternion_from_euler(0.0, 0.0, self.odom_yaw)
        self.pose_on_odom.pose.orientation.x = ox
        self.pose_on_odom.pose.orientation.y = oy
        self.pose_on_odom.pose.orientation.z = oz
        self.pose_on_odom.pose.orientation.w = ow

        self.pose_on_map = PoseStamped()

        self.pose_on_map.header.frame_id = "map"
        self.pose_on_map.pose.position.x = 5.0
        self.pose_on_map.pose.position.y = 7.0

        self.map_yaw = m.pi / 4.0

        mx, my, mz, mw = quaternion_from_euler(0.0, 0.0, self.map_yaw)
        self.pose_on_map.pose.orientation.x = mx
        self.pose_on_map.pose.orientation.y = my
        self.pose_on_map.pose.orientation.z = mz
        self.pose_on_map.pose.orientation.w = mw

        self.map_pub = self.node.create_publisher(
            PoseStamped, "/pose_on_map", qos_profile=1
        )
        self.odom_pub = self.node.create_publisher(
            PoseStamped, "/pose_on_odom", qos_profile=1
        )

        self.tf_pub = tf2_ros.TransformBroadcaster(self.node, qos=10)

    def publish(self):
        self.pose_on_map.header.stamp = rclpy.time.Time().to_msg()
        self.pose_on_odom.header.stamp = rclpy.time.Time().to_msg()

        self.map_pub.publish(self.pose_on_map)
        self.odom_pub.publish(self.pose_on_odom)

    def publishTF(self):

        deg = self.map_yaw - self.odom_yaw

        trans = (
            self.pose_on_map.pose.position.x
            - (
                self.pose_on_odom.pose.position.x
                * m.cos(deg - self.pose_on_odom.pose.position.y * m.sin(deg))
            ),
            self.pose_on_map.pose.position.y
            - (
                self.pose_on_odom.pose.position.x
                * m.sin(deg + self.pose_on_odom.pose.position.y * m.cos(deg))
            ),
            0.0,
        )

        rot = quaternion_from_euler(0.0, 0.0, deg)

        tf_msg = tf2_ros.TransformStamped()

        tf_msg.header.frame_id = "odom"
        tf_msg.header.stamp = rclpy.time.Time().to_msg()
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = trans[0]
        tf_msg.transform.translation.y = trans[1]
        tf_msg.transform.translation.z = trans[2]

        tf_msg.transform.rotation.x = rot[0]
        tf_msg.transform.rotation.y = rot[1]
        tf_msg.transform.rotation.z = rot[2]
        tf_msg.transform.rotation.w = rot[3]

        self.tf_pub.sendTransform(tf_msg)

    def calculate_transform(self):
        t_map = np.array(
            [
                self.pose_on_map.pose.position.x,
                self.pose_on_map.pose.position.y,
                self.pose_on_map.pose.position.z,
            ]
        )
        q_map = np.array(
            [
                self.pose_on_map.pose.orientation.x,
                self.pose_on_map.pose.orientation.y,
                self.pose_on_map.pose.orientation.z,
                self.pose_on_map.pose.orientation.w,
            ]
        )

        t_odom = np.array(
            [
                self.pose_on_odom.pose.position.x,
                self.pose_on_odom.pose.position.y,
                self.pose_on_odom.pose.position.z,
            ]
        )
        q_odom = np.array(
            [
                self.pose_on_odom.pose.orientation.x,
                self.pose_on_odom.pose.orientation.y,
                self.pose_on_odom.pose.orientation.z,
                self.pose_on_odom.pose.orientation.w,
            ]
        )

        # Compute inverse of odom pose
        q_odom_inv = quaternion_inverse(q_odom)
        t_odom_inv = -np.dot(quaternion_to_matrix(q_odom_inv)[:3, :3], t_odom)

        # Compute the transform from map to odom
        t_map_to_odom = t_map + np.dot(quaternion_to_matrix(q_map)[:3, :3], t_odom_inv)
        q_map_to_odom = quaternion_multiply(q_map, q_odom_inv)

        return t_map_to_odom, q_map_to_odom


def main():
    rclpy.init(args=None)

    node = rclpy.create_node("tf_test_node")

    test_node = TF_test(node)

    rate = node.create_rate(30)

    try:
        while rclpy.ok():
            rclpy.spin_once(node)

            test_node.publishTF()

            test_node.publish()

            rate.sleep()

    except Exception as ex:
        print(ex)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
