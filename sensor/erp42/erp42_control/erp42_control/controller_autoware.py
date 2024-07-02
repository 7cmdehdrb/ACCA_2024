#!/usr/bin/env python3

import rclpy
import math as m
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32, String
from erp42_msgs.msg import SerialFeedBack
from erp42_msgs.msg import ControlMessage
from tf_transformations import *
from autoware_auto_planning_msgs.msg import Trajectory
import threading

class TrajectoryLoader():
    def __init__(self, node, trajectory_topic):
        qos_profile = QoSProfile(depth = 10)
        node.create_subscription(Trajectory, "planning/scenario_planning/trajectory", self.callback_trajectory, qos_profile) #10hz

        self.cx = 0.
        self.cy = 0.
        self.cyaw = 0.
        self.cv = 0.

    def callback_trajectory(self, msg):
        self.cx, self.cy, self.cyaw, self.cv = self.update_path(msg)

    def update_path(self, data):
        cx = []
        cy = []
        cyaw = []
        cv = []

        for t in data.points:
            cx.append(t.pose.position.x)
            cy.append(t.pose.position.y)
            _,_,yaw = euler_from_quaternion([t.pose.orientation.x, t.pose.orientation.y, t.pose.orientation.z, t.pose.orientation.w]) 
            cyaw.append(yaw)
            cv.append(t.longitudinal_velocity_mps)

        return cx, cy, cyaw, cv
    
class State():
    def __init__(self, node, odom_topic):
        
        qos_profile = QoSProfile(depth = 10)
        node.create_subscription(Odometry, odom_topic, self.callback, qos_profile)
        node.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp, qos_profile)

        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s

    def callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _,_,self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        # self.v = m.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

    def callback_erp(self, msg):
        direction = 1.0 if msg.gear == 2 else -1.0
        self.v = msg.speed * direction

class PID():
    def __init__(self, node):
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.85).value
        # self.d_gain = node.declare_parameter("/stanley_controller/d_gain", 1.0).value
        self.node = node
        self.p_err = 0.0
        self.i_err = 0.0
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
        speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(speed, 0, 20))
    
class Stanley():
    def __init__(self, node):
        self.__L = 1.040  # [m] Wheel base of vehicle
        self.k_v = 1.1
        self.max_steer = 28 # [deg]
        self.target_idx = 0
        self.__k = node.declare_parameter("/stanley_controller/c_gain", 0.8).value
        self.__hdr_ratio = node.declare_parameter("/stanley_controller/hdr_ratio", 0.5).value
        


    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def stanley_control(self, state, cx, cy, cyaw):
        self.target_idx, ctr = self.calc_index_ctr(state, cx, cy)
        hdr = self.calc_hdr(state, cyaw, self.target_idx)

        print(self.target_idx)

        theta_c = np.arctan2(self.__k * ctr, (self.k_v + state.v))
        theta_h = hdr * self.__hdr_ratio

        delta = np.clip(theta_c + theta_h, m.radians((-1)*self.max_steer), m.radians(self.max_steer))

        return delta, theta_h, theta_c
    
    def calc_index_ctr(self, state, cx ,cy):
        fx = state.x + self.__L * np.cos(state.yaw) / 2.0 
        fy = state.y + self.__L * np.sin(state.yaw) / 2.0

        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]

        d = np.hypot(dx, dy)
        target_idx = int(np.argmin(d))

        front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]

        cross_track_error = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, cross_track_error
    
   
    def calc_hdr(self, state, cyaw, target_idx):
        heading_error = self.normalize_angle(cyaw[target_idx] - state.yaw)
        
        return heading_error


class SpeedSupporter():
    def __init__(self, node):
        self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value


    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self,value,hdr,ctr,min_value,max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)

        return res
    
    
    
class Drive():
    def __init__(self, node, state, trajectory):
        qos_profile = QoSProfile(depth = 10)
        self.pub = node.create_publisher(ControlMessage, "cmd_msg", qos_profile)
        # self.pub_idx = node.create_publisher(Int32, "target_idx", qos_profile)

        self.st = Stanley(node)
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)
        self.trajectory = trajectory
        self.state = state
        self.trajectory_speed = 6 # [Km/h] # speed setting
        # self.trajectory_speed = trajectory.cv[self.st.target_idx] * 3.6  # [Km/h] #trajectory speed setting
        self.adapted_speed = 0.
        
    
    def publish_cmd(self):
        steer, hdr, ctr = self.st.stanley_control(self.state, self.trajectory.cx, self.trajectory.cy, self.trajectory.cyaw)

        self.adapted_speed = self.ss.adaptSpeed(self.trajectory_speed, hdr, ctr, min_value=4, max_value=20)
        
        speed = self.pid.PIDControl(self.state.v*3.6, self.adapted_speed)

        brake = self.calc_brake()

        msg = ControlMessage()
        msg.speed = speed*10
        msg.steer = int(m.degrees((-1)*steer))
        msg.gear = 2
        msg.brake = int(brake)

        self.pub.publish(msg)

    def calc_brake(self):
        if self.state.v * 3.6 >= self.adapted_speed:
            brake = (abs(self.state.v * 3.6 - self.adapted_speed) / 10.0) * 200
        else:
            brake = 0

        return brake
   
def main(args = None):
    rclpy.init(args = args)
    node = rclpy.create_node("driving_autoware_node")
    state = State(node, "/localization/kinematic_state") 
    trajectory_tracking = TrajectoryLoader(node, "/planning/scenario_planning/trajectory")
    d = Drive(node, state, trajectory_tracking)

    thread = threading.Thread(target=rclpy.spin, args= (node, ), daemon = True)
    thread.start()

    rate = node.create_rate(8)

    while rclpy.ok():
        try:
            d.publish_cmd()
        except Exception as ex:
            print(ex)
        rate.sleep()
        
if __name__=="__main__":
    main()