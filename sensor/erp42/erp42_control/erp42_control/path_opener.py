#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, Float32, String
from tf_transformations import *
from pyproj import *
import math as m
from rclpy.qos import QoSProfile



class PathViewer(Node):
    def __init__(self, frame_id, period):
        super().__init__("erp42_control")
        qos_profile = QoSProfile(depth = 10)
        
        self.frame_id = frame_id
        self.file_path = self.declare_parameter("file_path", "/home/gjs/global_path/school_speed.txt").value

        self.create_subscription(NavSatFix, "ublox_gps_node/fix", self.callback, qos_profile)
        self.create_subscription(Int32, "target_idx", self.callback_idx, qos_profile)
        
        self.pub_path = self.create_publisher(PoseArray, "path/global", qos_profile)
        self.pub_speed = self.create_publisher(Float32, "target_speed", qos_profile)
        self.pub_tag = self.create_publisher(String, "mission_tag", qos_profile)
        self.pub_road_type = self.create_publisher(String, "road_type", qos_profile)

        self.timer = self.create_timer(1 / period, self.timer_callback)

        self.poses = []
        self.speeds = []
        self.tags = []
        self.road_type = []

        self.gps = Proj(init = "epsg:4326")  # lat, lon
        self.tm = Proj(init="epsg:2097")    # m

        self.gps_datum = None
        self.index = 0

    def callback(self,msg):
        if self.gps_datum is None:
            self.gps_datum = [msg.latitude, msg.longitude]
            print(self.gps_datum)
    
    def file_opener(self,file):

        f = open(file, "r")

        lines = f.readlines()

        poses = []
        speeds = []
        tags = []
        road_type = []
        num = 0
        x_o, y_o = transform(p1=self.gps, p2=self.tm, x=self.gps_datum[1], y=self.gps_datum[0])

        for line in lines:
            pose = Pose()
            
            l = line.rstrip("\n")
            l_split = l.split(", ")
            x,y,yaw = float(l_split[0]),float(l_split[1]),float(l_split[2]) 

            pose.position.x = x-x_o
            pose.position.y = y-y_o
            pose.position.z = 0.

            qx,qy,qz,qw = quaternion_from_euler(0, 0, m.radians(yaw))

            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            poses.append(pose)
            speeds.append(float(l_split[3]))
            tags.append(l_split[4])
            road_type.append(l_split[5])
            num +=1
        f.close()

        return poses, speeds, tags, road_type
    
    def callback_idx(self,msg):
        self.index = msg.data
    
    def publish_path(self,frame_id,file):
        if not self.gps_datum is None:
            if len(self.poses) == 0:
                self.poses, self.speeds, self.tags, self.road_type = self.file_opener(file)
            data = PoseArray()
            data.header.stamp = self.get_clock().now().to_msg()
            data.header.frame_id = frame_id
            # data.header.seq += 1
            data.poses = self.poses
            self.pub_path.publish(data)

    def publish_target_speed(self,target_idx):
        data = Float32()
        data.data = self.speeds[target_idx]
        self.pub_speed.publish(data)

    def publish_tag(self,target_idx):
        data = String()
        data.data = self.tags[target_idx]
        self.pub_tag.publish(data)

    def publish_road_type(self,target_idx):
        data = String()
        data.data = self.road_type[target_idx]
        self.pub_road_type.publish(data)

    def timer_callback(self):
        self.publish_path(self.frame_id, self.file_path)
        self.publish_target_speed(self.index)
        self.publish_tag(self.index)
        self.publish_road_type(self.index)

def main(args=None):
    rclpy.init(args=args)
    node = PathViewer("odom", 8.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
