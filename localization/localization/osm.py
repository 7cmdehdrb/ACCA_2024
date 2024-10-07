import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import numpy as np
import pyproj
from geometry_msgs.msg import (
    Point,
)
from visualization_msgs.msg import Marker, MarkerArray
from lxml import etree


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


class OSM(Node):
    def __init__(self):
        super().__init__("osm_node")

        # Gps transformer
        self.gps = pyproj.CRS("epsg:4326")  # lat, log
        self.tm = pyproj.CRS("epsg:2097")  # m
        self.transformer = pyproj.Transformer.from_crs(self.gps, self.tm)

        # Param declare
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "osm_file",
                    "/home/acca/catkin_ws/src/localization/resource/school.osm",
                ),
                # ("gps_origin", [37.4961657, 126.9570535]),
                ("gps_origin", [37.496253, 126.957289]),
                ("timer_period_sec", 0.0),
            ],
        )

        tree = etree.parse(
            self.get_parameter("osm_file").get_parameter_value().string_value
        )  # XML 파일 경로
        self.root = tree.getroot()  # 루트 요소 가져오기

        self.tags = self.root.xpath("//node")  # define all nodes
        self.ways = self.root.xpath("//way")  # define all ways

        self.node_pub = self.create_publisher(
            MarkerArray, "/osm/node", qos_profile=qos_profile_system_default
        )
        self.path_pub = self.create_publisher(
            MarkerArray, "/osm/path", qos_profile=qos_profile_system_default
        )

        gps_origin = (
            self.get_parameter("gps_origin").get_parameter_value().double_array_value
        )
        self.base_x, self.base_y = self.transformer.transform(
            gps_origin[0], gps_origin[1]
        )

        timer_period_sec = (
            self.get_parameter("timer_period_sec").get_parameter_value().double_value
        )

        if timer_period_sec == 0.0:
            self.create_nodes()
            self.create_paths()

        else:
            self.create_timer(timer_period_sec, self.create_nodes)
            self.create_timer(timer_period_sec, self.create_paths)

    def create_paths(self):
        marker_array = MarkerArray()

        for i, way in enumerate(self.ways):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = Time().to_msg()

            marker.ns = "path"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.scale.x = 0.1

            nds = way.xpath("./nd")

            for nd in nds:
                ref = nd.get("ref")
                node = self.root.xpath(f"//node[@id='{ref}']")

                lat = node[0].get("lat")
                lon = node[0].get("lon")

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

    def create_nodes(self):
        marker_array = MarkerArray()

        # Node
        for i, tag in enumerate(self.tags):
            marker = Marker()
            marker.header.frame_id = "odom"  # 적절한 좌표 프레임 설정
            marker.header.stamp = Time().to_msg()
            marker.ns = "node"
            marker.id = i  # 각 마커의 고유 ID
            marker.type = Marker.CUBE  # 포인트 타입
            marker.action = Marker.ADD

            lat = tag.get("lat")
            lon = tag.get("lon")
            x, y = self.transformer.transform(lat, lon)

            # 포인트 추가
            point = Point()
            point.x = -(x - self.base_x)
            point.y = y - self.base_y
            point.z = 0.0  # z 좌표 (예: 0.0)

            marker.pose.position = point

            # 포인트 색상 및 크기 설정
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue
            marker.color.a = 1.0  # Alpha (1.0은 완전 불투명)
            marker.scale.x = 1.0  # 포인트 크기
            marker.scale.y = 1.0  # 포인트 크기
            marker.scale.z = 1.0

            # 마커를 마커 배열에 추가
            marker_array.markers.append(marker)

        # 마커 배열 퍼블리시
        self.node_pub.publish(marker_array)
        self.get_logger().info("Published node array")


def main():
    rclpy.init(args=None)

    node = OSM()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
