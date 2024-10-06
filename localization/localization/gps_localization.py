import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import tf2_ros
import numpy as np
import math as m
import time
import pyproj
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from erp42_msgs.msg import SerialFeedBack


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


class AverageFilter:
    def __init__(self):
        self.i = 0
        self.average = 0.0

    def filter(self, data):
        # 샘플 수 +1 (+1 the number of sample)
        self.i += 1

        # 평균 필터의 alpha 값 (alpha of average filter)
        alpha = (self.i - 1) / (self.i + 0.0)

        # 평균 필터의 재귀식 (recursive expression of average filter)
        average = alpha * self.average + (1 - alpha) * data

        # 평균 필터의 이전 상태값 업데이트 (update previous state value of average filter)
        self.average = average

        return average


def inv(matrix):
    return np.linalg.inv(matrix)


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


class Ublox:
    class Data(NavSatFix):
        def __init__(self):
            super().__init__()
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.vel = 0.0

        def parse(self, msg):
            self.header = msg.header
            self.status = msg.status
            self.latitude = msg.latitude
            self.longitude = msg.longitude
            self.altitude = msg.altitude
            self.position_covariance = msg.position_covariance
            self.position_covariance_type = msg.position_covariance_type

    def __init__(self, node):
        self.node = node

        self.data = self.Data()
        self.normalizor = Normalizaor()
        self.valid = False

        # Sensing Data
        self.x = np.array(
            [
                0.0,  # x
                0.0,  # y
                0.0,  # yaw
                0.0,  # v
            ]
        )
        # Conv Matrix
        self.cov = np.array(
            [
                [0.1, 0.0, 0.0, 0.0],
                [0.0, 0.1, 0.0, 0.0],
                [0.0, 0.0, 0.1, 0.0],
                [0.0, 0.0, 0.0, 0.1],
            ]
        )
        self.H = np.array(
            [
                [1, 0, 0, 0],  # x
                [0, 1, 0, 0],  # y
                [0, 0, 0, 0],  # yaw
                [0, 0, 0, 0],  # v
            ]
        )

        # Gps transformer
        self.gps = pyproj.CRS("epsg:4326")  # lat, log
        self.tm = pyproj.CRS("epsg:2097")  # m
        self.transformer = pyproj.Transformer.from_crs(self.gps, self.tm)

        gps_origin = (
            self.node.get_parameter("gps_origin")
            .get_parameter_value()
            .double_array_value
        )
        self.base_x, self.base_y = self.transformer.transform(
            gps_origin[0], gps_origin[1]
        )

        self.last_point = Point()

        self.gps_subscriber = self.node.create_subscription(
            NavSatFix,
            "/ublox_gps_node/fix",
            callback=self.gps_callback,
            qos_profile=qos_profile_system_default,
        )

    def gps_callback(self, msg):
        self.data.parse(msg)

        lat = self.data.latitude
        lon = self.data.longitude

        x, y = self.transformer.transform(lat, lon)

        point = Point(x=-(x - self.base_x), y=(y - self.base_y), z=0.0)

        self.data.x = point.x
        self.data.y = point.y
        atan = m.atan2(point.y - self.last_point.y, point.x - self.last_point.x)
        self.data.vel = (
            np.linalg.norm(
                np.array([point.y - self.last_point.y, point.x - self.last_point.x])
            )
            / 0.125
        )

        self.valid = (
            1.0 < self.data.vel
            and self.data.vel < 10.0
            and self.data.position_covariance[0] < 0.01
        )
        if self.valid:
            yaw = self.normalizor.filter(atan)
            self.data.yaw = yaw

        self.x = np.array(
            [
                self.data.x,  # x
                self.data.y,  # y
                0.0,  # yaw
                0.0,  # v
            ]
        )
        self.cov = np.array(
            [
                [
                    self.data.position_covariance[0] * m.sqrt(111319.490793),
                    0.0,
                    0.0,
                    0.0,
                ],  # x
                [
                    0.0,
                    self.data.position_covariance[0] * m.sqrt(111319.490793),
                    0.0,
                    0.0,
                ],  # y
                [0.0, 0.0, 0.0, 0.0],  # yaw
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],  # v
            ]
        )

        self.last_point = point


class Xsens:
    class Data(Imu):
        def __init__(self):
            super().__init__()
            self.yaw = 0.0

        def parse(self, msg):
            self.header = msg.header
            self.orientation = msg.orientation
            self.orientation_covariance = msg.orientation_covariance
            self.angular_velocity = msg.angular_velocity
            self.angular_velocity_covariance = msg.angular_velocity_covariance
            self.linear_acceleration = msg.linear_acceleration
            self.linear_acceleration_covariance = msg.linear_acceleration_covariance

    def __init__(self, node):
        self.node = node

        self.data = self.Data()
        self.normalizor = Normalizaor()
        self.offset = 0.0

        # Sensing Data
        self.x = np.array(
            [
                0.0,  # x
                0.0,  # y
                0.0,  # yaw
                0.0,  # v
            ]
        )
        # Conv Matrix
        self.cov = np.array(
            [
                [0.1, 0.0, 0.0, 0.0],
                [0.0, 0.1, 0.0, 0.0],
                [0.0, 0.0, 0.1, 0.0],
                [0.0, 0.0, 0.0, 0.1],
            ]
        )
        self.H = np.array(
            [
                [0, 0, 0, 0],  # x
                [0, 0, 0, 0],  # y
                [0, 0, 1, 0],  # yaw
                [0, 0, 0, 0],  # v
            ]
        )

        self.imu_subscriber = self.node.create_subscription(
            Imu,
            "/imu/data",
            callback=self.imu_callback,
            qos_profile=qos_profile_system_default,
        )

    def imu_callback(self, msg):
        self.data.parse(msg)

        _, _, yaw = euler_from_quaternion(
            [
                self.data.orientation.x,
                self.data.orientation.y,
                self.data.orientation.z,
                self.data.orientation.w,
            ]
        )
        self.data.yaw = self.normalizor.filter(yaw)

        self.x = np.array(
            [
                0.0,  # x
                0.0,  # y
                self.data.yaw + self.offset,  # yaw
                0.0,  # v
            ]
        )

        self.cov = np.array(
            [
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, self.data.orientation_covariance[0], 0.0],
                [0.0, 0.0, 0.0, 0.0],
            ]
        )


class ERP42:
    class Data(SerialFeedBack):
        def __init__(self):
            super().__init__()
            self.vel = 0.0

        def parse(self, msg):
            self.mora = msg.mora
            self.estop = msg.estop
            self.gear = msg.gear
            self.speed = msg.speed
            self.steer = msg.steer
            self.brake = msg.brake
            self.encoder = msg.encoder
            self.alive = msg.alive

    def __init__(self, node):
        self.node = node

        self.data = self.Data()
        self.drive = True

        # Sensing Data
        self.x = np.array(
            [
                0.0,  # x
                0.0,  # y
                0.0,  # yaw
                0.0,  # v
            ]
        )
        # Conv Matrix
        self.cov = np.array(
            [
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.1],
            ]
        )
        self.H = np.array(
            [
                [0, 0, 0, 0],  # x
                [0, 0, 0, 0],  # y
                [0, 0, 0, 0],  # yaw
                [0, 0, 0, 1],  # v
            ]
        )

        self.serial_subsciber = self.node.create_subscription(
            SerialFeedBack,
            "/erp42_feedback",
            callback=self.serial_callback,
            qos_profile=qos_profile_system_default,
        )

    def serial_callback(self, msg):
        self.data.parse(msg)

        self.drive = self.data.gear == 2
        self.data.vel = self.data.speed * 1.0 if self.drive else -1.0

        self.x = np.array(
            [
                0.0,  # x
                0.0,  # y
                0.0,  # yaw
                self.data.speed,  # v
            ]
        )


class Kalman:
    def __init__(self, node, xsens, ublox, erp42):
        self.node = node

        self.xsens = xsens
        self.ublox = ublox
        self.erp42 = erp42

        self.af = AverageFilter()
        self.xsens_offset = 0.0

        self.last_time = time.time()
        self.dt = 0.0

        # State Matrix
        self.x = np.array(
            [
                0.0,  # x
                0.0,  # y
                0.0,  # yaw
                0.0,  # v
            ]
        )

        # Model
        self.A = np.array(
            [
                [1, 0, 0, m.cos(self.x[2]) * self.dt],  # x
                [0, 1, 0, m.sin(self.x[2]) * self.dt],  # y
                [0, 0, 1, 0],  # yaw
                [0, 0, 0, 1],  # v
            ]
        )
        # Conv Matrix
        self.P = np.array(
            [
                [0.1, 0.0, 0.0, 0.0],
                [0.0, 0.1, 0.0, 0.0],
                [0.0, 0.0, 0.1, 0.0],
                [0.0, 0.0, 0.0, 0.1],
            ]
        )  # Noise Matrix
        self.Q = np.array(
            [
                [0.01, 0.0, 0.0, 0.0],
                [0.0, 0.01, 0.0, 0.0],
                [0.0, 0.0, 0.01, 0.0],
                [0.0, 0.0, 0.0, 0.01],
            ]
        )

        self.odom_publisher = self.node.create_publisher(
            Odometry, "/odometry/kalman", qos_profile=qos_profile_system_default
        )
        self.tf_publisher = tf2_ros.TransformBroadcaster(
            self.node, qos=qos_profile_system_default
        )

        # Loop
        self.node.create_timer(float(1 / 30.0), callback=self.main)

    def main(self):
        current_time = time.time()

        self.dt = current_time - self.last_time

        # Sensor Calibration
        if self.ublox.valid:
            self.xsens.offset = self.af.filter(
                self.ublox.data.yaw - self.xsens.data.yaw
            )

        self.A = np.array(
            [
                [1, 0, 0, m.cos(self.x[2]) * self.dt],  # x
                [0, 1, 0, m.sin(self.x[2]) * self.dt],  # y
                [0, 0, 1, 0],  # yaw
                [0, 0, 0, 1],  # v
            ]
        )

        u_k = np.array(
            [
                0.0,  # x
                0.0,  # y
                0.0,  # yaw
                0.0,  # v
            ]
        )

        R = self.ublox.cov + self.xsens.cov + self.erp42.cov  # Convariances
        x_k = np.dot(self.A, self.x) + u_k  # Predicted State
        P_k = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q  # Predicted Convariance

        # Kalman Gain
        S_gps = inv(np.dot(np.dot(self.ublox.H, P_k), self.ublox.H.T) + R)
        K_gps = np.dot(
            np.dot(P_k, self.ublox.H.T),
            S_gps,
        )

        S_erp42 = inv(np.dot(np.dot(self.erp42.H, P_k), self.erp42.H.T) + R)
        K_erp42 = np.dot(
            np.dot(P_k, self.erp42.H.T),
            S_erp42,
        )

        S_xsens = inv(np.dot(np.dot(self.xsens.H, P_k), self.xsens.H.T) + R)
        K_xsens = np.dot(
            np.dot(P_k, self.xsens.H.T),
            S_xsens,
        )

        K = (
            np.dot(K_gps, inv(S_gps))
            + np.dot(K_xsens, inv(S_xsens))
            + np.dot(K_erp42, inv(S_erp42))
        )

        # Updated X
        X = (
            x_k
            + np.dot(K_gps, (self.ublox.x - np.dot(self.ublox.H, x_k)))
            + np.dot(K_xsens, (self.xsens.x - np.dot(self.xsens.H, x_k)))
            + np.dot(K_erp42, (self.erp42.x - np.dot(self.erp42.H, x_k)))
        )
        self.x = X

        H = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        # Update Covariance
        self.P = P_k - np.dot(np.dot(K, H), P_k)

        self.last_time = current_time

        self.odom_publisher.publish(self.create_odometry())
        self.tf_publisher.sendTransform(self.create_tf())

        return self.x, self.P

    def create_odometry(self):
        msg = Odometry()

        msg.header.frame_id = "utm"
        msg.header.stamp = Time().to_msg()

        msg.pose.pose.position.x = self.x[0]
        msg.pose.pose.position.y = self.x[1]

        x, y, z, w = quaternion_from_euler(0.0, 0.0, self.x[2])
        msg.pose.pose.orientation.x = x
        msg.pose.pose.orientation.y = y
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w

        msg.pose.covariance[0] = self.P[0][0]
        msg.pose.covariance[7] = self.P[1][1]

        msg.twist.twist.linear.x = self.x[3] * m.cos(self.x[2])
        msg.twist.twist.linear.y = self.x[3] * m.sin(self.x[2])

        msg.twist.covariance[0] = self.P[3][3]  # linear x 0-5

        return msg

    def create_tf(self):
        tf_msg = tf2_ros.TransformStamped()

        tf_msg.header.frame_id = "utm"
        tf_msg.header.stamp = Time().to_msg()
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = self.x[0]
        tf_msg.transform.translation.y = self.x[1]
        tf_msg.transform.translation.z = 0.0

        x, y, z, w = quaternion_from_euler(0.0, 0.0, self.x[2])
        tf_msg.transform.rotation.x = x
        tf_msg.transform.rotation.y = y
        tf_msg.transform.rotation.z = z
        tf_msg.transform.rotation.w = w

        return tf_msg


class GPSLocalizer:
    def __init__(self, node, xsens, ublox):

        self.node = node

        self.xsens = xsens
        self.ublox = ublox

        # Ros
        self.odometry_publisher = self.node.create_publisher(
            Odometry, "/odometry/gps", qos_profile=qos_profile_system_default
        )

        self.gps_odometry = Odometry()

        # Main Loop
        self.node.create_timer(0.125, callback=self.create_gps_odometry)

    def create_gps_odometry(self):
        msg = Odometry()
        msg.header.frame_id = "utm"
        msg.header.stamp = Time().to_msg()

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.ublox.data.yaw)

        msg.pose.pose.position = Point(x=self.ublox.data.x, y=self.ublox.data.y, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        position_covariance = np.array(
            [
                [
                    self.ublox.cov[0][0],
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],  # x
                [
                    0.0,
                    self.ublox.cov[0][0],
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],  # y
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # z
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # r
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # p
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # y
            ]
        )
        msg.pose.covariance = position_covariance.reshape(-1)

        self.odometry_publisher.publish(msg)

        return msg


def main():
    rclpy.init(args=None)

    node = Node(node_name="gps_localization_node")

    node.declare_parameters(
        namespace="",
        parameters=[
            ("gps_origin", [37.4961657, 126.9570535]),
        ],
    )

    xsens = Xsens(node)
    ublox = Ublox(node)
    erp42 = ERP42(node)
    gps_localizer = GPSLocalizer(node, xsens, ublox)
    kalman = Kalman(node, xsens, ublox, erp42)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
