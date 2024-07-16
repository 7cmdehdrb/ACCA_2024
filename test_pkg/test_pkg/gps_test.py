import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
import tf2_ros
from abc import *
import numpy as np
import math as m
import pyproj
import time
import threading

# Msgs
from std_msgs.msg import Empty, Header, Float64MultiArray
from sensor_msgs.msg import Imu, NavSatFix
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry


class GPS(Node):
    def __init__(self):
        super().__init__("gps_test_node")

        self.gps = pyproj.CRS("epsg:4326")  # lat, log
        self.tm = pyproj.CRS("epsg:2097")  # m
        self.transformer = pyproj.Transformer.from_crs(self.gps, self.tm)

        self.sub = self.create_subscription(
            NavSatFix, "/ublox_gps_node/fix", callback=self.callback, qos_profile=10
        )

        self.pub = self.create_publisher(PointStamped, "/gps/pose", qos_profile=10)

        self.init_point = None
        self.point_stamp = PointStamped()

    def callback(self, msg):
        # msg = NavSatFix()
        lat = msg.latitude
        lon = msg.longitude

        x, y = self.transformer.transform(lat, lon)

        if self.init_point is None:
            p = Point()
            p.x = x
            p.y = y

            self.init_point = p

        else:
            p = Point()
            p.x = x - self.init_point.x
            p.y = y - self.init_point.y

            point_stamp = PointStamped()
            point_stamp.header.frame_id = "map"
            point_stamp.header.stamp = rclpy.time.Time().to_msg()
            point_stamp.point = p

            self.point_stamp = point_stamp

        self.publish()

    def publish(self):
        self.pub.publish(self.point_stamp)


def main():
    rclpy.init(args=None)

    node = GPS()

    try:
        rclpy.spin(node)
    except Exception as ex:
        node.get_logger().warn(ex)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
