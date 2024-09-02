import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
import numpy as np
import math as m
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


class Normalizaor(object):
    def __init__(self):
        self.last_value = 0.0
        self.value = 0.0

    def filter(self, value):
        self.value = value

        while abs(self.value - self.last_value) > m.pi:
            if self.last_value < 0.0:
                self.value -= 2 * m.pi
            elif self.last_value > 0.0:
                self.value += 2 * m.pi

        self.last_value = self.value

        return self.value


def test(value):
    if value < -m.pi:
        return value + (2 * m.pi)

    elif value > m.pi:
        return value - (2 * m.pi)

    return value


class ImuTest(Node):
    def __init__(self):
        super().__init__("feedback_odom_converter")
        self.nomalizor = Normalizaor()
        self.imu_subscriber = self.create_subscription(
            Imu, "/imu/data", self.callback, 10
        )
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.yaw = 0.0

    def timer_callback(self):
        # random_yaw = random.randint(-314, 314) * 10e-3
        # self.yaw = self.nomalizor.filter(random_yaw)
        test_value = self.yaw - 0.5
        test_value = test(test_value)
        self.yaw = self.nomalizor.filter(test_value)
        print(test_value, self.yaw)

    def callback(self, msg):
        imu_quat = msg.orientation
        _, _, yaw = euler_from_quaternion(
            [imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w]
        )
        self.yaw = self.nomalizor.filter(yaw)
        print(self.yaw)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
