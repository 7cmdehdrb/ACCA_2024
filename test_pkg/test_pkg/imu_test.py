import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class ImuTest(Node):
    def __init__(self):
        super().__init__("feedback_odom_converter")
        self.imu_subscriber = self.create_subscription(
            Imu, "/imu/data", self.feedbackCallback, 10
        )
        self.imu_publisher = self.create_publisher(Imu, "/imu/test", 10)

    def feedbackCallback(self, msg):
        imu_msg = msg
        imu_msg.header.stamp = rclpy.time.Time().to_msg()
        self.imu_publisher.publish(imu_msg)


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
