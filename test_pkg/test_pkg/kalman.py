import rclpy
import rclpy.logging
from rclpy.node import Node
import numpy as np
import math as m
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Quaternion, Point
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from erp42_msgs.msg import ControlMessage, SerialFeedBack
import pyproj
import time


class Gaussian(object):
    def __init__(self, x, mean, sigma):
        self.x = x
        self.mean = mean
        self.sigma = sigma

        if x is not None:
            self.value = self.calculateGaussian()

    def calculateGaussian(self):
        return (1 / np.sqrt(2.0 * m.pi * self.sigma**2.0)) * np.exp(
            -((self.x - self.mean) ** 2.0) / (2.0 * self.sigma**2)
        )


def gaussianConvolution(g1, g2):
    mean = g1.mean + (g1.sigma**2 * (g2.mean - g1.mean)) / (g1.sigma**2 + g2.sigma**2)
    if g1.sigma == g2.sigma:
        sigma = g1.sigma
    else:
        sigma = 2.0 * (g1.sigma**2 - (g1.sigma**4) / (g1.sigma**2 - g2.sigma**2))
    return Gaussian(g1.x, mean, abs(sigma))


def measure(lat1, lon1, lat2, lon2):
    R = 6378.137
    dLat = lat2 * m.pi / 180 - lat1 * m.pi / 180
    dLon = lon2 * m.pi / 180 - lon1 * m.pi / 180
    a = m.sin(dLat / 2) * m.sin(dLat / 2) + m.cos(lat1 * m.pi / 180) * m.cos(
        lat2 * m.pi / 180
    ) * m.sin(dLon / 2) * m.sin(dLon / 2)
    c = 2 * m.atan2(m.sqrt(a), m.sqrt(1 - a))
    d = R * c
    return d * 1000


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


def kph2mps(value):
    return value * 0.277778


def mps2kph(value):
    return value * 3.6


def getEmpty(shape):
    return np.array([[0.0 for i in range(shape[0])] for j in range(shape[1])])


class Kalman(object):
    def __init__(self, *args, **kwargs):
        super().__init__()

        self.odom_tf = tf2_ros.TransformBroadcaster(node, qos=10)
        self.odom_pub = node.create_publisher(
            topic="/odometry/kalman", msg_type=Odometry, qos_profile=10
        )

        # initial value
        self.x = np.array([0, 0, 0, 0])

        self.P = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # noise
        self.Q = np.array(
            [
                [0.1, 0.0, 0.0, 0.0],
                [0.0, 0.1, 0.0, 0.0],
                [0.0, 0.0, 0.01, 0.0],
                [0.0, 0.0, 0.0, 0.01],
            ]
        )

        self.dt = 0.0

    def inv(self, matrix):
        return np.linalg.inv(matrix)

    def filter(self, gps, erp, imu, dt):
        A = np.array(
            [
                [1, 0, m.cos(self.x[3]) * dt, 0],
                [0, 1, m.sin(self.x[3]) * dt, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        z_position = gps.data
        z_erp = erp.data
        z_imu = imu.data

        cov_position = gps.cov
        cov_erp = erp.cov
        cov_imu = imu.cov

        R = cov_position + cov_erp + cov_imu

        u_k = np.array(
            [
                0.0,
                0.0,
                0.0,
                (kph2mps(cmd.data[0]) / 1.040) * m.tan(-m.degrees(cmd.data[1])) * dt,
            ]
        )
        x_k = np.dot(A, self.x) + u_k
        P_k = np.dot(np.dot(A, self.P), A.T) + self.Q

        H_position = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
        H_erp = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0]])
        H_imu = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]])

        K_position = np.dot(
            np.dot(P_k, H_position.T),
            self.inv(np.dot(np.dot(H_position, P_k), H_position.T) + R),
        )
        K_erp = np.dot(
            np.dot(P_k, H_erp.T), self.inv(np.dot(np.dot(H_erp, P_k), H_erp.T) + R)
        )
        K_imu = np.dot(
            np.dot(P_k, H_imu.T), self.inv(np.dot(np.dot(H_imu, P_k), H_imu.T) + R)
        )

        print(np.dot(np.dot(H_imu, P_k), H_imu.T) + R)

        X = (
            x_k
            + np.dot(K_position, (z_position - np.dot(H_position, x_k)))
            + np.dot(K_erp, (z_erp - np.dot(H_erp, x_k)))
            + np.dot(K_imu, (z_imu - np.dot(H_imu, x_k)))
        )

        self.x = X
        self.P = P_k - np.dot(
            np.dot((K_position + K_erp + K_imu), np.identity(n=4)), P_k
        )

        self.dt = dt

        return self.x, self.P

    def publishOdom(self, x, P):
        msg = Odometry()

        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = x[0]
        msg.pose.pose.position.y = x[1]
        msg.pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0, 0, x[3])

        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        msg.pose.covariance = [
            P[0][0],
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            P[1][1],
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            P[3][3],
        ]

        t = tf2_ros.TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = x[0]
        t.transform.translation.y = x[1]
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Send the transformation
        self.odom_tf.sendTransform(t)
        self.odom_pub.publish(msg)


class Sensor(object):
    def __init__(self, node_name, topic, msg_type):
        super().__init__()

        self.data = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.cov = np.array(
            [
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            ]
        )
        self.once = False

        self.sub = node.create_subscription(
            topic=topic, msg_type=msg_type, callback=self.sensorCallback, qos_profile=10
        )

    def sensorCallback(self, msg):
        self.data, self.cov = self.handleData(msg)
        self.once = True

    def handleData(self, msg):
        return np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64), self.cov


class ERP42(Sensor):
    def __init__(self, node_name, topic, msg_type):
        super().__init__(node_name, topic, msg_type)
        self.cov = np.array(
            [
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    999,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            ]
        )

    def handleData(self, msg):
        speed = msg.speed * (1.0 if msg.gear == 2 else -1.0)
        cov = getEmpty((4, 4))
        cov[2][2] = 0.1

        self.cov = cov

        return np.array([0.0, 0.0, speed, 0.0], dtype=np.float64), self.cov


class Xsens(Sensor):
    def __init__(self, node_name, topic, msg_type):
        super().__init__(node_name, topic, msg_type)

        self.cov = np.array(
            [
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    99,
                ],
            ]
        )

        self.yaw = 0.0
        self.init_yaw = None

    def handleData(self, msg):
        quat = msg.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        if self.init_yaw is None:
            self.init_yaw = yaw

        self.yaw = yaw - self.init_yaw

        cov = getEmpty((4, 4))
        cov[-1][-1] = msg.orientation_covariance[-1]

        self.cov = cov

        return np.array([0, 0, 0, self.yaw]), self.cov


class GPS(Sensor):
    def __init__(self, node_name, topic, msg_type):
        super().__init__(node_name, topic, msg_type)
        self.cov = np.array(
            [
                [
                    999.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    999.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            ]
        )

        self.gps_odom_pub = node.create_publisher(
            topic="odometry/gps", msg_type=Odometry, qos_profile=10
        )

        self.x = 0.0
        self.y = 0.0

        self.gps = pyproj.CRS("epsg:4326")  # lat, log
        self.tm = pyproj.CRS("epsg:2097")  # m

        self.last_position = None

    def publishOdometry(self, x, y):
        msg = Odometry()

        msg.header.frame_id = "odom"
        msg.header.stamp = rclpy.time.Time().to_msg()

        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        quat = quaternion_from_euler(0.0, 0.0, kf.x[3])

        q = Quaternion()
        q.x = quat[0]
        q.y = quat[1]
        q.z = quat[2]
        q.w = quat[3]

        msg.pose.pose.orientation = q

        msg.pose.covariance = [0.0 for i in range(36)]
        msg.pose.covariance[0] = self.cov[0][0]
        msg.pose.covariance[7] = self.cov[1][1]

        self.gps_odom_pub.publish(msg)

    def calculateDistance(self, p1, p2):
        return m.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def calculateDistanceFromGPS(self, log, lat):
        transformer = pyproj.Transformer.from_crs(self.gps, self.tm)
        x, y = transformer.transform(lat, log)

        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0

        current_point = p

        if self.last_position is None:
            self.last_position = current_point

        distance = self.calculateDistance(self.last_position, current_point)

        self.last_position = current_point

        if distance < 0.012:
            distance = 0

        return distance

    def handleData(self, msg):
        distance = self.calculateDistanceFromGPS(log=msg.longitude, lat=msg.latitude)

        if imu.yaw is not None:
            self.x += distance * m.cos(imu.yaw)
            self.y += distance * m.sin(imu.yaw)

        # 37.5

        self.cov = np.array(
            [
                [
                    msg.position_covariance[0] * m.sqrt(111319.490793) + 0.5,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    msg.position_covariance[0] * m.sqrt(111319.490793) + 0.5,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            ]
        )

        self.publishOdometry(self.x, self.y)

        return np.array([self.x, self.y, 0.0, 0.0], dtype=np.float64), self.cov


class Control(Sensor):
    def __init__(self, node_name, topic, msg_type):
        super().__init__(node_name, topic, msg_type)

        self.data = np.array([0.0, 0.0, 0.0])  # sp, st, br
        self.cov = None

    def handleData(self, msg):
        return np.array([msg.Speed, msg.Steer, msg.brake]), None


rclpy.init(args=None)

node = rclpy.create_node("kalman_filter")
gps = GPS("gps", "/ublox_gps_node/fix", NavSatFix)
erp = ERP42("erp42", "/erp42_feedback", SerialFeedBack)
imu = Xsens("imu", "/imu/data", Imu)
cmd = Control("cmd", "/cmd_msg", ControlMessage)

kf = Kalman()

hz = 10
dt = 1.0 / hz

current_time = time.time()
last_time = time.time()

r = node.create_rate(10)

try:
    while rclpy.ok():
        rclpy.spin_once(node)

        current_time = time.time()

        x, P = kf.filter(gps, erp, imu, dt=(current_time - last_time))

        kf.publishOdom(x, P)

        last_time = current_time

        # print(x)

        # r.sleep()

except KeyboardInterrupt as ki:
    pass

except Exception as ex:
    print(ex)


print("LOOP END")
