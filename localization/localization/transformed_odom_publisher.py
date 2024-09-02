import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped as PoseStamped2
from geometry_msgs.msg import PoseWithCovarianceStamped



class TransformedOdomPublisher(Node):
    def __init__(self):
        super().__init__("transformed_tf_odom_node")

        self.odom_sub = self.create_subscription(Odometry, "/odometry/kalman", callback=self.odom_callback, qos_profile=qos_profile_system_default)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/kalman/transformed", qos_profile=qos_profile_system_default)

        self.buffer = Buffer(node=self, cache_time=Duration(seconds=0.1))
        self.tf_listener = TransformListener(self.buffer, self, qos=qos_profile_system_default)   


    def odom_callback(self, msg):
        if self.buffer.can_transform(
            "map",
            "odom",
            Time(),
            Duration(seconds=0.1),
        ):
            ps_odom = PoseStamped2(header=msg.header, pose=msg.pose.pose)

            ps_odom_on_map = self.buffer.transform(
                ps_odom, "map", Duration(seconds=0.1)
            )

            map_odom = Odometry()

            map_odom.header.frame_id = "map"
            map_odom.header.stamp = Time().to_msg()
            map_odom.child_frame_id = "odom"

            map_odom.pose.pose = ps_odom_on_map.pose

            self.odom_pub.publish(map_odom)


        else:
            self.get_logger().warn("Cannot lookup transform")




def main():
    rclpy.init(args=None)

    node = TransformedOdomPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()