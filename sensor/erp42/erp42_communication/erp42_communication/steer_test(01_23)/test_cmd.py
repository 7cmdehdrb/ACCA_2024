#!/usr/bin/env python3


import sys
import rclpy
import math as m
from rclpy.node import Node
from rclpy.qos import QoSProfile
from erp42_msgs.msg import ControlMessage

class Test(Node):
    def __init__(self, steer):
        super().__init__("test_cmd")
        qos_profile =  QoSProfile(depth=10)
        self.steer = steer

        self.publisher = self.create_publisher(ControlMessage, "/cmd_msg", qos_profile)
        self.timer = self.create_timer(1.0 / 30.0, self.pub)


    def pub(self):
        msg = ControlMessage()

        msg.speed = 3*10
        msg.steer = int((-1)*self.steer)
        msg.gear = 2
        msg.brake = 0

        self.publisher.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = Test(steer = 0) #steer(deg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()