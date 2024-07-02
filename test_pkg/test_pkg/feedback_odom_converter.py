import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Header
from erp42_msgs.msg import SerialFeedBack
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped


class FeedbackOdomConverter(Node):
    def __init__(self):
        super().__init__("feedback_odom_converter")
        self.feedback_subscriber = self.create_subscription(
            SerialFeedBack, "/erp42_feedback", self.feedbackCallback, 10
        )
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/erp42", 10)

    def feedbackCallback(self, msg):
        odom_msg = Odometry()
        # odom_msg = TwistWithCovarianceStamped()

        odom_msg.header.frame_id = "base_link"
        odom_msg.header.stamp = rclpy.time.Time().to_msg()
        odom_msg.twist.twist.linear.x = msg.speed
        odom_msg.twist.covariance[0] = 0.1

        self.odom_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FeedbackOdomConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as ex:
        print(ex)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
