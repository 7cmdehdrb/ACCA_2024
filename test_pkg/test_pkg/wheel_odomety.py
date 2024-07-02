import rclpy
import rclpy.time
from rclpy.node import Node
import math as m
import numpy as np
import time
from std_msgs.msg import Header
from erp42_msgs.msg import SerialFeedBack
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, Pose, Twist


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


class LowPassFilter:
    def __init__(self, cutoff_freq, ts):
        self.ts = ts
        self.cutoff_freq = cutoff_freq
        self.pre_out = 0.0
        self.tau = self.calc_filter_coef()

    def calc_filter_coef(self):
        w_cut = 2 * np.pi * self.cutoff_freq
        return 1 / w_cut

    def filter(self, data):
        out = (self.tau * self.pre_out + self.ts * data) / (self.tau + self.ts)
        self.pre_out = out
        return out


class WheelOdomety(Node):
    def __init__(self):
        super().__init__("wheel_odomety")
        self.feedback_subscriber = self.create_subscription(
            SerialFeedBack, "/erp42_feedback", self.feedbackCallback, 10
        )
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/wheel", 10)

        self.splpf = LowPassFilter(0.5, 0.01)
        self.stlpf = LowPassFilter(0.5, 0.01)

        self.msg = Odometry()
        self.msg.header.frame_id = "odom"

        self.yaw = 0.0

        self.last_time = time.time()

    def feedbackCallback(self, msg):
        self.msg.header.stamp = rclpy.time.Time().to_msg()
        current_time = time.time()

        speed = msg.speed
        steer = msg.steer
        dt = current_time - self.last_time

        if dt > 0.1:
            dt = 0.0

        dyaw = (speed / 1.040) * m.tan(steer) * dt

        # position
        self.msg.pose.pose.position.x += speed * m.cos(self.yaw) * dt
        self.msg.pose.pose.position.y += speed * m.sin(self.yaw) * dt

        self.msg.pose.covariance[0] = 3.0
        self.msg.pose.covariance[7] = 3.0

        # oroentation
        self.yaw += dyaw
        quat = quaternion_from_euler(roll=0.0, pitch=0.0, yaw=self.yaw)  # [x, y, z, w]

        self.msg.pose.pose.orientation.x = quat[0]
        self.msg.pose.pose.orientation.y = quat[1]
        self.msg.pose.pose.orientation.z = quat[2]
        self.msg.pose.pose.orientation.w = quat[3]

        self.msg.twist.twist.linear.x = speed * m.cos(self.yaw)
        self.msg.twist.twist.linear.y = speed * m.sin(self.yaw)

        self.msg.twist.covariance[0] = 1.0
        self.msg.twist.covariance[7] = 1.0

        self.odom_publisher.publish(self.msg)

        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as ex:
        print(ex)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
