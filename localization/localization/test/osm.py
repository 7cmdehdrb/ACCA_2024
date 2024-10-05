import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import numpy as np
import math as m
import pyproj
from quaternion import *
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import (
    Point,
    Quaternion,
    PointStamped,
    PoseArray,
    Pose,
    PoseStamped,
)
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from lxml import etree

"""
# XML 파일 읽기
tree = etree.parse(
    "/home/acca/catkin_ws/src/localization/resource/map.osm"
)  # XML 파일 경로
root = tree.getroot()  # 루트 요소 가져오기

# # 특정 태그 값 가져오기
for tag in root.xpath("//node"):
    key = tag.get("lat")
    value = tag.get("lon")
    print(f"Tag Key: {key}, Value: {value}")
"""


class Test(Node):
    def __init__(self):
        super().__init__("test_node")

        # Gps transformer
        self.gps = pyproj.CRS("epsg:4326")  # lat, log
        self.tm = pyproj.CRS("epsg:2097")  # m
        self.transformer = pyproj.Transformer.from_crs(self.gps, self.tm)

        tree = etree.parse(
            "/home/acca/catkin_ws/src/localization/resource/school.osm"
        )  # XML 파일 경로
        # tree = etree.parse(
        #     "/home/acca/catkin_ws/src/localization/resource/kcity.osm"
        # )  # XML 파일 경로
        self.root = tree.getroot()  # 루트 요소 가져오기

        self.tags = self.root.xpath("//node")
        self.nodes_with_name = self.root.xpath("//node[tag]")
        self.ways = self.root.xpath("//way")

        self.node_pub = self.create_publisher(
            MarkerArray, "/marker/node", qos_profile=qos_profile_system_default
        )
        self.path_pub = self.create_publisher(
            MarkerArray, "/marker/path", qos_profile=qos_profile_system_default
        )

        self.base_x, self.base_y = self.transformer.transform(
            37.4961657, 126.9570535
        )  # school
        # self.base_x, self.base_y = self.transformer.transform(
        #     37.23963, 126.77449
        # )  # kcity

        self.create_timer(5.0, self.create_markers)
        self.create_timer(5.0, self.create_path)
        # self.create_markers()
        # self.create_path()

    def create_path(self):
        marker_array = MarkerArray()

        for i, way in enumerate(self.ways):
            marker = Marker()
            marker.header.frame_id = "utm"
            marker.header.stamp = Time().to_msg()

            marker.ns = "line_strip"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.pose.position = Point()
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            marker.color.r = 0.0  # Red
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0  # Blue
            marker.color.a = 1.0  # Alpha (1.0은 완전 불투명)
            marker.scale.x = 0.1  # 포인트 크기

            for nd in way:
                ref = nd.get("ref")
                node = self.root.xpath(f"//node[@id='{ref}']")

                if len(node) == 1:
                    lat = node[0].get("lat")
                    lon = node[0].get("lon")
                else:
                    break

                x, y = self.transformer.transform(lat, lon)

                # 포인트 추가
                point = Point()
                point.x = -(x - self.base_x)
                point.y = y - self.base_y

                # 좌표 추가
                marker.points.append(point)

            marker_array.markers.append(marker)

        self.get_logger().info("Published path")
        self.path_pub.publish(marker_array)

    def create_markers(self):
        marker_array = MarkerArray()

        # Node
        for i, tag in enumerate(self.tags):
            marker = Marker()
            marker.header.frame_id = "utm"  # 적절한 좌표 프레임 설정
            marker.header.stamp = Time().to_msg()
            marker.ns = "points"
            marker.id = i  # 각 마커의 고유 ID
            marker.type = Marker.CUBE  # 포인트 타입
            marker.action = Marker.ADD

            # 포인트 색상 및 크기 설정
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue
            marker.color.a = 1.0  # Alpha (1.0은 완전 불투명)
            marker.scale.x = 1.0  # 포인트 크기
            marker.scale.y = 1.0  # 포인트 크기
            marker.scale.z = 1.0

            lat = tag.get("lat")
            lon = tag.get("lon")
            x, y = self.transformer.transform(lat, lon)

            # 포인트 추가
            point = Point()
            point.x = -(x - self.base_x)
            point.y = y - self.base_y
            point.z = 0.0  # z 좌표 (예: 0.0)

            marker.pose.position = point

            # 마커를 마커 배열에 추가
            marker_array.markers.append(marker)

        # Name Tag
        for i, tag in enumerate(self.nodes_with_name):
            text_marker = Marker()
            text_marker.header.frame_id = "utm"
            text_marker.header.stamp = Time().to_msg()
            text_marker.ns = "named_points_text"
            text_marker.id = len(marker_array.markers)  # 고유 ID 부여
            text_marker.type = Marker.TEXT_VIEW_FACING  # 텍스트 타입
            text_marker.action = Marker.ADD
            text_marker.text = ""

            # 포인트 색상 및 크기 설정
            text_marker.color.r = 1.0  # Red
            text_marker.color.g = 0.0  # Green
            text_marker.color.b = 0.0  # Blue
            text_marker.color.a = 1.0  # Alpha (1.0은 완전 불투명)
            text_marker.scale.x = 1.0  # 포인트 크기
            text_marker.scale.y = 1.0  # 포인트 크기
            text_marker.scale.z = 1.0

            lat = tag.get("lat")
            lon = tag.get("lon")
            x, y = self.transformer.transform(lat, lon)

            # 포인트 추가
            point = Point()
            point.x = -(x - self.base_x)
            point.y = y - self.base_y
            point.z = 0.1  # z 좌표 (예: 0.0)

            text_marker.pose.position = point  # 텍스트 위치 설정

            for t in tag:
                k = t.get("k")
                if k == "name:en":
                    name = t.get("v")
                    text_marker.text = name  # 마커의 이름 설정
                    break

            marker_array.markers.append(text_marker)

        # 마커 배열 퍼블리시
        self.node_pub.publish(marker_array)
        self.get_logger().info("Published node array")


def main():
    rclpy.init(args=None)

    node = Test()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
