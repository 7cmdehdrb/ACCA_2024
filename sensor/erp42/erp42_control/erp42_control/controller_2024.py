#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Int32, String
from erp42_msgs.msg import SerialFeedBack
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray
from stanley import Stanley
from erp42_msgs.msg import ControlMessage
from tf_transformations import *
import numpy as np
import math as m
import threading
from pyproj import *



class PathHandler():
    def __init__(self, node, path_topic):
        qos_profile = QoSProfile(depth = 10)
        node.create_subscription(PoseArray, path_topic, self.callback_path, qos_profile)

        self.cx = []
        self.cy = []
        self.cyaw = []

    def callback_path(self,msg):
        self.cx, self.cy, self.cyaw = self.update_path(msg)

    def update_path(self, data):
        cx = []
        cy = []
        cyaw = []
        for p in data.poses:
            cx.append(p.position.x)
            cy.append(p.position.y)
            _,_,yaw = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]) 
            cyaw.append(yaw)    
        return cx, cy, cyaw
    
class State():
    def __init__(self, node, odom_topic):
        qos_profile = QoSProfile(depth = 10)
        node.create_subscription(Odometry, odom_topic, self.callback, qos_profile)

        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s

    def callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _,_,self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        # self.v = m.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
    
    
        
class PID():
    def __init__(self, node):
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.85).value
        # self.d_gain = node.declare_parameter("/stanley_controller/d_gain", 1.0).value
        self.node = node
        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0
        self.speed_ = 0.0
        # self.d_err = 0.0
        
        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
    def PIDControl(self, speed, desired_value):

        node = self.node
        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        dt = self.current - self.last
        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt 
        self.p_err = err
        self.i_err += self.p_err * dt  * (0.0 if speed == 0 else 1.0)

        self.last = self.current
        # speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err) + (self.d_gain * self.d_err)
        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        self.speed_ = int(np.clip(self.speed, 0, 20))
        return self.speed_
    
class SpeedSupporter():
    def __init__(self, node):
        self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.04).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.08).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value


    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self,value,hdr,ctr,min_value,max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        # err = ctr
        res = np.clip(value + err, min_value, max_value)
        # res = value

        return res
    
class Drive():
    def __init__(self, node, state, path, target_speed):
        qos_profile = QoSProfile(depth = 10)
        node.create_subscription(Float32, target_speed, self.callback_speed, qos_profile)
        node.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp, qos_profile)
        # node.create_subscription(String, "path_type", self.callback_path_type, qos_profile)
        node.create_subscription(String, "road_type", self.callback_shape, qos_profile)
        node.create_subscription(NavSatFix, "ublox_gps_node/fix", self.callback_gps, qos_profile)

        self.pub = node.create_publisher(ControlMessage, "cmd_msg", qos_profile)
        self.pub_idx = node.create_publisher(Int32, "target_idx", qos_profile)

        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)
        self.path = path
        self.state = state
        self.last_target_idx_global = 0
        self.path_speed = 6
        self.current_speed = 0.
        self.path_type = "global"
        self.road_type = "straight"
        self.max_steer = 28

        self.hdr = 0.
        self.ctr = 0.
        self.h = 0.
        self.c = 0.
        self.adapted_speed = 0.
        self.steer = 0.
        self.mora = 0
        self.gear = 0
        self.input_brake = 0
        self.output_brake = 0
        self.gps_datum = None
        self.x_o = 0.
        self.y_o = 0.
        self.x = 0.
        self.y = 0.
        self.gps = Proj(init = "epsg:4326")  # lat, lon
        self.tm = Proj(init="epsg:2097")    # m
        self.target_idx = 0


    def publish_cmd(self):
        path_type = self.path_type
        if path_type == "global":
            last_target_idx = self.last_target_idx_global
        else:
            last_target_idx = 0
        steer, self.target_idx, self.hdr, self.ctr, self.h, self.c= self.st.stanley_control(self.state, self.path.cx, self.path.cy, self.path.cyaw, last_target_idx)
        #self.hdr는 항 / self.h는 순수 헤딩 에러

        if path_type == "global":
            self.last_target_idx_global = self.target_idx
            path_speed = self.path_speed
        else:
            path_speed = 4

        self.adapted_speed = self.ss.adaptSpeed(path_speed,self.hdr,self.ctr,min_value=4,max_value=20)
        self.steer = np.clip(steer,m.radians((-1)*self.max_steer),m.radians(self.max_steer))
        speed = self.pid.PIDControl(self.current_speed*3.6, self.adapted_speed)

        msg = ControlMessage()
        msg.speed = speed*10
        msg.steer = int(m.degrees((-1)*self.steer))
        msg.gear = 2
        
        if self.current_speed * 3.6 >= self.adapted_speed:
            self.input_brake = (abs(self.current_speed * 3.6 - self.adapted_speed) / 10.0) * 200
        else:
            self.input_brake = 0

        msg.brake = int(self.input_brake)

        data = Int32()
        data.data = self.target_idx

        self.pub.publish(msg)
        self.pub_idx.publish(data)
    
    
    
    def make_txt(self):
        f = open("/home/gjs/stanley_test/he_gain:{}, ce_gain:{}, he_thr:{}, ce_thr:{}_5"
                    .format(self.ss.he_gain, self.ss.ce_gain, self.ss.he_thr, self.ss.ce_thr), 'a')
            
        # data = "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(self.x, self.y, self.path.cx[self.target_idx], self.path.cy[self.target_idx], self.h, self.c, self.hdr, self.ctr, self.current_speed * 3.6, self.adapted_speed, self.steer, self.road_type, self.input_brake, self.output_brake, self.pid.speed_)
        data = "{}, {}, {}, {}\n".format(self.current_speed * 3.6, self.adapted_speed, self.steer, self.pid.speed_)


        if self.mora == 1:
            f.write(data)
        else:
            f.close()

    def callback_gps(self, msg):
        if self.gps_datum is None:
            self.gps_datum = [msg.latitude, msg.longitude]
            self.x_o, self.y_o = transform(p1=self.gps, p2=self.tm, x=self.gps_datum[1], y=self.gps_datum[0])
        else:
            x, y = transform(p1=self.gps, p2=self.tm, x=msg.longitude, y=msg.latitude)
            self.x = x - self.x_o
            self.y = y - self.y_o
    
    def callback_speed(self,msg):
        self.path_speed = msg.data

    def callback_erp(self,msg):
        direction = 1.0 if msg.gear == 2 else -1.0
        self.current_speed = msg.speed * direction
        self.gear = msg.gear
        self.output_brake = msg.brake

        if msg.mora == 0:
            self.mora = 0
        else:
            self.mora = 1

        

    def callback_path_type(self,msg):
        self.path_type = msg.data

    def callback_shape(self, msg):
        self.road_type = msg.data

def main(args = None):
    rclpy.init(args = args)
    node = rclpy.create_node("driving_node")
    state = State(node, "odometry/navsat")
    path_tracking = PathHandler(node, "path/global")
    d = Drive(node, state, path_tracking, "target_speed")


    thread = threading.Thread(target=rclpy.spin, args= (node, ), daemon = True)
    thread.start()

    rate = node.create_rate(8)

    while rclpy.ok():
        try:
            d.publish_cmd()
            d.make_txt()
        except Exception as ex:
            print(ex)
        rate.sleep()
    
if __name__=="__main__":
    main()