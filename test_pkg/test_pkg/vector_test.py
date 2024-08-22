import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_system_default
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped
import numpy as np
import math as m
import time
from std_msgs.msg import Float32, Float32MultiArray, Header
from geometry_msgs.msg import Transform, Vector3, Quaternion, PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped as TransformStamped2
from nav_msgs.msg import Odometry
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped as PoseStamped2


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


class Map_Odom_TF_Publisher(Node):
    def __init__(self):
        super().__init__("map_odom_tf_node")

        class LowPassFilter:
            def __init__(self, cutoff_freq, ts):
                self.ts = ts
                self.cutoff_freq = cutoff_freq
                self.pre_out = 0.0
                self.tau = self.calc_filter_coef()

            def calc_filter_coef(self):
                w_cut = 2 * np.pi * self.cutoff_freq
                return 1 / w_cut

            def filter(self, data):
                out = (self.tau * self.pre_out + self.ts * data) / (self.tau + self.ts)
                self.pre_out = out
                return out

        # Odomety State Instace. Odometry data => state(linear vel, angular vel)
        class Odom_State(object):
            def __init__(self):
                self.current_time = time.time()
                self.last_time = time.time()
                self.last_pose = Odometry()
                self.state = [0.0, 0.0, 0.0]

            def get_state(self):
                return "{}\t{}\t{}".format(
                    round(self.state[0], 3),
                    round(self.state[1], 3),
                    round(self.state[2], 3),
                )

            def update_state(self, new_pose):
                self.current_time = time.time()

                dt = self.current_time - self.last_time

                new_position = new_pose.pose.pose.position
                last_position = self.last_pose.pose.pose.position

                new_quat = new_pose.pose.pose.orientation
                _, _, new_yaw = euler_from_quaternion(
                    [new_quat.x, new_quat.y, new_quat.z, new_quat.w]
                )

                last_quat = self.last_pose.pose.pose.orientation
                _, _, last_yaw = euler_from_quaternion(
                    [last_quat.x, last_quat.y, last_quat.z, last_quat.w]
                )

                pose_difference_vector = np.array(
                    [new_position.x - last_position.x, new_position.y - last_position.y]
                )
                heading_angle_vector = np.array([m.cos(new_yaw), m.sin(new_yaw)])
                pose_difference_norm = np.linalg.norm(pose_difference_vector)

                vel_x = new_pose.twist.twist.linear.x
                vel_y = new_pose.twist.twist.linear.y

                linear_vel = m.sqrt(vel_x**2 + vel_y**2)

                angular_vel = new_pose.twist.twist.angular.z

                angle_difference = m.acos(
                    np.dot(pose_difference_vector, heading_angle_vector)
                    / pose_difference_norm
                    if pose_difference_norm != 0.0
                    else 0.0
                )

                self.state = [linear_vel, angular_vel, angle_difference]

                self.last_pose = new_pose
                self.last_time = self.current_time

        # PCL State Instance. Same with OdomState
        class PCL_State(object):
            def __init__(self):
                self.current_time = time.time()
                self.last_time = time.time()
                self.last_pose = PoseWithCovarianceStamped()
                self.state = [0.0, 0.0, 0.0]
                self.test_vector = PoseWithCovarianceStamped()

            def get_state(self):
                return "{}\t{}\t{}".format(
                    round(self.state[0], 3),
                    round(self.state[1], 3),
                    round(self.state[2], 3),
                )

            def update_state(self, new_pose):
                self.current_time = time.time()

                dt = self.current_time - self.last_time

                new_position = new_pose.pose.pose.position
                last_position = self.last_pose.pose.pose.position

                new_quat = new_pose.pose.pose.orientation
                _, _, new_yaw = euler_from_quaternion(
                    [new_quat.x, new_quat.y, new_quat.z, new_quat.w]
                )

                last_quat = self.last_pose.pose.pose.orientation
                _, _, last_yaw = euler_from_quaternion(
                    [last_quat.x, last_quat.y, last_quat.z, last_quat.w]
                )

                pose_difference_vector = np.array(
                    [new_position.x - last_position.x, new_position.y - last_position.y]
                )

                heading_angle_vector = np.array([m.cos(new_yaw), m.sin(new_yaw)])
                pose_difference_norm = np.linalg.norm(pose_difference_vector)

                linear_vel = pose_difference_norm / dt if dt != 0.0 else 0.0
                angular_vel = (new_yaw - last_yaw) / dt if dt != 0.0 else 0.0

                angle_difference = m.acos(
                    np.dot(heading_angle_vector, pose_difference_vector)
                    / pose_difference_norm
                    if pose_difference_norm != 0.0
                    else 0.0
                )

                self.state = [linear_vel, angular_vel, angle_difference]

                self.last_pose = new_pose
                self.last_time = self.current_time

        self.map_topic = "/pcl_pose"
        self.odom_topic = "/odometry/kalman"

        # TF
        self.buffer = Buffer(node=self, cache_time=Duration(seconds=0.1))
        self.tf_listener = TransformListener(
            self.buffer, self, qos=qos_profile_system_default
        )
        self.tf_publisher = TransformBroadcaster(self, qos=qos_profile_system_default)
        self.tf_msg = tf2_ros.TransformStamped()

        self.pcl_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.map_topic,
            callback=self.pcl_callback,
            qos_profile=qos_profile_system_default,
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            callback=self.odom_callback,
            qos_profile=qos_profile_system_default,
        )

        # Topic Variables. Update autumatically
        self.pcl = PoseWithCovarianceStamped()
        self.odom = Odometry()
        self.score = 0.0

        # Local Variables
        self.err_stack = 0  # Err stack flag
        self.linear_err = 0.0
        self.angular_err = 0.0
        self.angle_difference_err = 0.0

        # State instance
        self.pcl_state = PCL_State()
        self.odom_state = Odom_State()

        self.lpf = LowPassFilter(1.0, 0.1)
        self.test = self.create_publisher(
            Float32MultiArray, "/test", qos_profile=qos_profile_system_default
        )

    # Callback Functions
    def pcl_callback(self, msg):
        self.pcl = msg

        self.pcl_state.update_state(msg)

        # print(self.pcl_state.get_state())

        self.update()

    def odom_callback(self, msg):
        self.odom = msg

        self.odom_state.update_state(msg)

        # print(self.odom_state.get_state())

    # Loop function : update tf_msg with conditions
    def update(self):
        # Calculate Errors
        temp_linear_err = self.pcl_state.state[0] - self.odom_state.state[0]
        self.linear_err = abs(temp_linear_err)
        self.angular_err = abs(self.pcl_state.state[1] - self.odom_state.state[1])
        self.angle_difference_err = abs(
            self.pcl_state.state[2] - self.odom_state.state[2]
        )

        # print(self.pcl_state.state)
        # print(self.odom_state.state)

        lpf_return = self.lpf.filter(self.pcl_state.state[0])

        self.test.publish(
            Float32MultiArray(
                data=[
                    self.pcl_state.state[0],
                    lpf_return,
                    self.odom_state.state[0],
                    self.odom_state.state[1],
                    self.pcl_state.state[2],
                ]
            )
        )


def main():
    rclpy.init(args=None)

    node = Map_Odom_TF_Publisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
