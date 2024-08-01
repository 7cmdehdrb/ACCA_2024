import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import PointCloud2
from erp42_msgs.msg import SerialFeedBack
from rclpy.qos import qos_profile_system_default


class VelodynePoints2(object):
    def __init__(self, node):
        self.node = node
        self.publisher = self.node.create_publisher(
            PointCloud2, "/velodyne_points2", qos_profile=qos_profile_system_default
        )
        self.subscriber = self.node.create_subscription(
            PointCloud2,
            "/velodyne_points",
            callback=self.callback,
            qos_profile=qos_profile_system_default,
        )

    def callback(self, msg):
        msg.header.stamp = rclpy.time.Time().to_msg()
        self.publisher.publish(msg)


def main():
    rclpy.init(args=None)

    node = rclpy.create_node("velodyne_points2_node")

    velodyne = VelodynePoints2(node)

    try:
        rclpy.spin(node)
    except Exception as ex:
        print(ex)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
