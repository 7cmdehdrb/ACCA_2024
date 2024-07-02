#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf_transformations import *
from std_msgs.msg import Float32
from erp42_msgs.msg import ControlMessage
from sensor_msgs.msg import Imu

class SteerImu(Node):
    def __init__(self):
        super().__init__("steer_imu")
        qos_profile = QoSProfile(depth=10)
        
        self.yaw = None
        self.data = None
        self.steer = None
        
        self.publisher = self.create_publisher(Float32, "/steer/imu", qos_profile)
        self.timer = self.create_timer(1.0 / 30.0, self.pub)

        self.subscriber_imu = self.create_subscription(
            Imu,
            "/imu/data", #토픽명 정확히 확인 후 수정 필요!
            self.callback_imu,
            qos_profile
        )

        self.subscriber_cmd = self.create_subscription(
            ControlMessage,
            "/cmd_msg",
            self.callback_cmd,
            qos_profile
        )

    def callback_imu(self, msg):
        self.data = msg.orientation
        data = self.data
        if self.yaw is None:
            _,_,self.yaw = euler_from_quaternion([data.x,data.y,data.z,data.w])

    def callback_cmd(self, msg):
        if self.yaw is not None:
            _,_,yaw = euler_from_quaternion([self.data.x,self.data.y,self.data.z,self.data.w])
            self.steer = yaw - self.yaw


    def pub(self):
        msg = Float32()
        if self.steer is not None:
            msg.data = self.steer
            self.publisher.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = SteerImu()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
