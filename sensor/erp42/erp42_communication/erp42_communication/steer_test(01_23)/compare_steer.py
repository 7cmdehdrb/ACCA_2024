#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math as m
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
from erp42_msgs.msg import SerialFeedBack
from erp42_msgs.msg import ControlMessage


class Compare(Node):
    def __init__(self):
        super().__init__("compare_steer")
        qos_profile =  QoSProfile(depth=10)
        self.timer = self.create_timer(1, self.print_steer)

        self.feedback_steer = None
        self.imu_steer = None
        self.cmd_steer = None


        self.subscriber_feedback = self.create_subscription(
            SerialFeedBack,
            "/erp42_feedback",
            self.callback_feedback,
            qos_profile
        )

        self.subscriber_imu = self.create_subscription(
            Float32,
            "/steer/imu",
            self.callback_imu,
            qos_profile
        )

        self.subscriber_cmd = self.create_subscription(
            ControlMessage,
            "/cmd_msg",
            self.callback_cmd,
            qos_profile
        )

    def print_steer(self):
        print("feedback_steer: {}, imu_steer: {}, cmd_steer: {}\n".format(self.feedback_steer, self.imu_steer, self.cmd_steer))


    def callback_feedback(self, msg):
        self.feedback_steer = m.degrees(msg.steer)
    
    def callback_imu(self, msg):
        self.imu_steer = msg.data

    def callback_cmd(self, msg):
        self.cmd_steer = msg.steer

def main(args = None):
    rclpy.init(args = args)
    node = Compare()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()