import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default
import asyncio
import websockets
import json
import threading
import numpy as np
import math as m
from uuid import uuid4
from rclpy_message_converter import message_converter, json_message_converter

# Messages
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class WebsocketBridge(Node):
    class Publisher:
        def __init__(self, node, topic, instance, dtype):
            self.node = node
            self.topic = topic
            self.instance = instance
            self.type = dtype

            self.msg = None
            self.trigger = False

            self.subscriber = self.node.create_subscription(
                self.instance,
                self.topic,
                callback=self.callback,
                qos_profile=qos_profile_system_default,
            )

            self.thread = self.create_socket_publisher()
            self.thread.start()

        def callback(self, msg):
            self.msg = msg
            self.trigger = True

        def parse_dictionary(self, msg):
            fields = [field[1:] for field in msg.__slots__]
            for f in fields:
                print(f, type(getattr(msg, f)))
            return {field: getattr(msg, field) for field in fields}

        def parse_message(self, msg):
            js = json_message_converter.convert_ros_message_to_json(msg)
            return json.loads(js.replace("'", '"'))

        async def publish_ros_topic(self):
            uri = "ws://localhost:9090"  # URI to the rosbridge WebSocket server
            async with websockets.connect(uri) as websocket:
                # Define the publish message for my_msgs/CustomMsg

                advertise_msg = {
                    "op": "advertise",
                    "topic": self.topic,
                    "type": self.type,
                }

                # Send the advertise message as a JSON string
                await websocket.send(json.dumps(advertise_msg))

                print("Published to {} topic.".format(self.topic))

                while rclpy.ok():
                    try:
                        if self.trigger is True:
                            msg = self.parse_message(self.msg)
                            publish_message = {
                                "op": "publish",
                                "topic": self.topic,
                                "msg": msg,
                            }

                            # Send the publish message to the rosbridge server

                            await websocket.send(json.dumps(publish_message))

                            self.trigger = False
                    except Exception as ex:
                        print("Exception: publish_ros_topic")
                        print(ex)

        def publish_async(self):
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.publish_ros_topic())

        def create_socket_publisher(self):
            return threading.Thread(target=self.publish_async)

    class Subscriber:
        def __init__(self, node, topic, instance):
            self.node = node
            self.topic = topic
            self.instance = instance

            self.publisher = self.node.create_publisher(
                self.instance,
                self.topic,
                qos_profile=qos_profile_system_default,
            )

            self.thread = self.create_socket_subscriber()
            self.thread.start()

        def parse_message(self, instance, data_dict):
            if "header" in data_dict:
                del data_dict["header"]["stamp"]
                del data_dict["header"]["seq"]

            msg = message_converter.convert_dictionary_to_ros_message(
                instance, data_dict
            )
            msg.header.stamp = Time().to_msg()

            return msg

        async def subscribe_ros_topic(self):
            uri = "ws://localhost:9090"
            async with websockets.connect(uri) as websocket:
                # Define the subscription message
                subscribe_message = {"op": "subscribe", "topic": self.topic}

                # Send the subscription message to the rosbridge server
                await websocket.send(json.dumps(subscribe_message))

                print("Subscribed to {} topic.".format(self.topic))

                while rclpy.ok():
                    # Wait for a message from the server
                    try:
                        response = await websocket.recv()
                        data = json.loads(response)

                        msg = self.parse_message(self.instance, data["msg"])

                        self.publisher.publish(msg)
                    except Exception as ex:
                        print("Exception: subscribe_ros_topic")
                        print(ex)

        def subscribe_async(self):
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.subscribe_ros_topic())

        def create_socket_subscriber(self):
            return threading.Thread(target=self.subscribe_async)

    def __init__(self):
        super().__init__("ros2_websocket_bridge_node")

        self.pub1 = self.Publisher(
            self, "/velodyne_points", PointCloud2, "sensor_msgs/PointCloud2"
        )
        self.pub2 = self.Publisher(self, "/imu/data", Imu, "sensor_msgs/Imu")
        self.pub3 = self.Publisher(
            self,
            "/initialpose",
            PoseWithCovarianceStamped,
            "geometry_msgs/PoseWithCovarianceStamped",
        )
        self.sub1 = self.Subscriber(self, "/odom", Odometry)


def main():
    rclpy.init()

    node = WebsocketBridge()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
