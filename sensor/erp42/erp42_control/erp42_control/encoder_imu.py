#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from erp42_msgs.msg import SerialFeedBack
import numpy as np
from tf_transformations import *
import math as m
from pyproj import *
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix



class EnconderImu(Node):
    def __init__(self):
        super().__init__("Encoder_imu")
        qos_profile = QoSProfile(depth = 1)
        self.create_subscription(Imu, "imu/rotated", self.callback_imu, qos_profile)
        self.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp, qos_profile)

        self.yaw = 0.
        self.init_yaw = None
        self.v = 0.
        self.x = 0.
        self.y = 0.
        self.current = self.get_clock().now().seconds_nanoseconds()[0] + (self.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.last = self.get_clock().now().seconds_nanoseconds()[0] + (self.get_clock().now().seconds_nanoseconds()[1] / 1e9)
    
    def callback_imu(self, msg):
        _,_,yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        if self.init_yaw is None:
            self.init_yaw = yaw
        self.yaw = yaw - self.init_yaw
    
    def callback_erp(self, msg):
        direction = 1.0 if msg.gear == 2 else -1.0
        self.v = msg.speed * direction
        self.handleData()


    def handleData(self):
        self.current = self.get_clock().now().seconds_nanoseconds()[0] + (self.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        dt = self.current - self.last
        self.last = self.current
        self.x += dt*self.v*m.cos(self.yaw)
        self.y += dt*self.v*m.sin(self.yaw)
        print(self.x, self.y, self.yaw)
        self.make_txt()
    
    def make_txt(self):
        f = open("/home/gjs/imu_encoder_pose/position_data", 'a')
        data = "{}, {}\n".format(self.x, self.y)
        f.write(data)




def main(args = None):
    rclpy.init(args=args)
    node = EnconderImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
        