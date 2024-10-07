import rclpy
from rclpy.node import Node
import rclpy.time
from nav_msgs.msg import MapMetaData, OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class ImageMarkerNode(Node):
    def __init__(self):
        super().__init__("image_marker_node")
        self.publisher = self.create_publisher(OccupancyGrid, "map", 10)
        self.timer = self.create_timer(1.0, self.publish_occupied_grid)

    def publish_occupied_grid(self):
        marker = Marker()
        marker.header.frame_id = "utm"
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = "image"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        # 텍스처로 사용할 이미지 파일 경로
        # marker.mesh_resource = "/home/acca/catkin_ws/static_map1.png"

        # 이미지의 위치 및 크기 설정 (XY 평면에 놓기)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # 이미지의 크기를 정하기 위해 사각형의 4개 점 설정
        marker.points = [
            Point(x=-1.0, y=-1.0, z=0.0),  # 왼쪽 아래
            Point(x=1.0, y=-1.0, z=0.0),  # 오른쪽 아래
            Point(x=1.0, y=1.0, z=0.0),  # 오른쪽 위
            Point(x=-1.0, y=1.0, z=0.0),  # 왼쪽 위
        ]

        # 텍스처를 적용할 이미지 색상
        marker.color.a = 1.0  # 이미지의 알파 값 (투명도)

        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ImageMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
