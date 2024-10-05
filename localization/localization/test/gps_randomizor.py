import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import numpy as np
import math as m
import time
import pyproj
from quaternion import *
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from erp42_msgs.msg import SerialFeedBack
import random


class GPSRandomizor(Node):
    def __init__(self):
        super().__init__("gps_randomizor")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("random", False),
            ],
        )

        self.random = self.get_parameter("random").get_parameter_value().bool_value

        self.msg = NavSatFix()

        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/ublox_gps_node/fix",
            callback=self.gps_callback,
            qos_profile=qos_profile_system_default,
        )
        self.gps_pub = self.create_publisher(
            NavSatFix, "/ublox_gps_node/random", qos_profile=qos_profile_system_default
        )

    def gps_callback(self, msg):
        self.random = self.get_parameter("random").get_parameter_value().bool_value

        if self.random is True:
            msg.latitude += random.uniform(-0.0001, 0.0001)
            msg.longitude += random.uniform(-0.0001, 0.0001)

            msg.position_covariance[0] += 1.0
            msg.position_covariance[4] += 1.0

        self.gps_pub.publish(msg)


def main():
    rclpy.init(args=None)

    node = GPSRandomizor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
