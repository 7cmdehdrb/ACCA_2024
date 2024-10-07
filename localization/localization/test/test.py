import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_system_default
import pyproj
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import numpy as np
import math as m


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


class Test(Node):
    def __init__(self):
        super().__init__("test_node")

        self.cv2_image = cv2.imread(
            "/home/acca/catkin_ws/geo_37.496253_126.957289_19.png", cv2.IMREAD_GRAYSCALE
        )  # 600, 600, 3 ndarray
        print(self.cv2_image.shape)

        self.gps_positions = [
            [37.49455638193991, 126.95502510313436],
            [37.49455638193991, 126.95908189686564],
            [37.49777501806009, 126.95502510313436],
            [37.49455638193991, 126.95502510313436],
        ]

        # Gps transformer
        self.gps = pyproj.CRS("epsg:4326")  # lat, log
        self.tm = pyproj.CRS("epsg:2097")  # m
        self.transformer = pyproj.Transformer.from_crs(self.gps, self.tm)

        self.gps_origin = [37.496253, 126.957289]
        self.base_x, self.base_y = self.transformer.transform(
            self.gps_origin[0], self.gps_origin[1]
        )

        # self.marker_array = self.parseMakerArray()
        self.occupied_map = self.create_occupied_grid()

        self.marker_array_pub = self.create_publisher(
            MarkerArray, "/map/marker", qos_profile=qos_profile_system_default
        )
        self.occupied_map_pub = self.create_publisher(
            OccupancyGrid, "/occupied_grid_map", qos_profile=qos_profile_system_default
        )

        # self.create_timer(3.0, callback=self.publish_marker_array)
        # self.publish_marker_array()
        self.create_timer(3.0, self.publish_occupied_grid)

    def publish_occupied_grid(self):
        print("HELLO WORLD!")
        self.occupied_map.header.stamp = Time().to_msg()
        self.occupied_map_pub.publish(self.occupied_map)

    def create_occupied_grid(self):
        # _, _, resolution = self.calculate_position()
        # 40 075 016.686
        # 6 378 137
        c = 6378137 * 2 * m.pi
        s_tile = c * m.cos(37.496253) / (2**19)
        s_pixel = s_tile / 256.0

        print(s_pixel)

        occupied_grid = OccupancyGrid()

        occupied_grid.header.frame_id = "odom"
        occupied_grid.header.stamp = Time().to_msg()

        occupied_grid.info.height = self.cv2_image.shape[0]
        occupied_grid.info.width = self.cv2_image.shape[1]

        occupied_grid.info.resolution = s_pixel

        occupied_grid.info.origin.position.x = -(748 / 2) * s_pixel
        occupied_grid.info.origin.position.y = -(1312 / 2) * s_pixel

        # occupied_grid.info.origin.position.x = resolution * -300.0
        # occupied_grid.info.origin.position.y = resolution * -300.0
        occupied_grid.info.origin.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(
            m.radians(0), m.radians(180), m.radians(-90)
        )

        occupied_grid.info.origin.orientation.x = qx
        occupied_grid.info.origin.orientation.y = qy
        occupied_grid.info.origin.orientation.z = qz
        occupied_grid.info.origin.orientation.w = qw

        # OccupancyGrid 데이터 변환 (0~255 값을 0~100으로 변환)
        occupancy_data = []
        for i in self.cv2_image:
            for j in i:
                occupancy_data.append(int(j * (100 / 256)))  # 알 수 없는 공간

        occupied_grid.data = occupancy_data

        # print(occupancy_data)

        return occupied_grid

    def publish_marker_array(self):
        print("HELLO WORLD!")
        # print(self.marker_array)
        self.marker_array_pub.publish(self.marker_array)

    def calculate_position(self):
        result = []

        for gps in self.gps_positions:
            x, y = self.transformer.transform(gps[0], gps[1])
            result.append([-(x - self.base_x), y - self.base_y])

        dx = abs(result[0][0] - result[2][0]) / 600.0
        dy = abs(result[0][1] - result[1][1]) / 600.0

        return result, dx, dy

    def parseMakerArray(self):
        marker_array = MarkerArray()

        i = 0

        positions, dx, dy = self.calculate_position()

        x = positions[0][0]
        y = positions[0][1]

        for row in self.cv2_image:
            for col in row:
                point = Point(x=x, y=y, z=0.0)
                rgb = [col[0] / 255.0, col[1] / 255.0, col[2] / 255.0]
                marker = self.create_marker(point, rgb, i)

                x += dx

                marker_array.markers.append(marker)

                i += 1

                print(i, end=" ")

            y += dy

        return marker_array

    def create_marker(self, point, rgb, id):
        marker = Marker()
        marker.header.frame_id = "utm"
        marker.header.stamp = Time().to_msg()

        marker.ns = "pixel"
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[1]
        marker.color.a = 1.0
        marker.scale.x = 0.1

        marker.points.append(point)

        return marker


def main():
    rclpy.init(args=None)

    test = Test()

    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()


main()
