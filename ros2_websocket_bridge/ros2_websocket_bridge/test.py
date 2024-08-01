from rclpy_message_converter import message_converter, json_message_converter
from nav_msgs.msg import Odometry
import json


def parse_message(msg):
    js = json_message_converter.convert_ros_message_to_json(msg)
    return json.loads(js.replace("'", '"'))


def parse_odometry(dic):
    dic["TwistWithCovariance"] = dic["twist"]
    dic["PoseWithCovariance"] = dic["pose"]
    del dic["twist"]
    del dic["pose"]
    return dic


def parse_message(instance, data_dict):
    instance = instance()
    for key, value in data_dict.items():
        if hasattr(instance, key):
            setattr(instance, key, value)

    return instance


msg = {
    "topic": "/odom",
    "msg": {
        "twist": {
            "twist": {
                "linear": {"y": 0.0, "x": 0.0, "z": 0.0},
                "angular": {"y": 0.0, "x": 0.0, "z": 0.0},
            },
            "covariance": [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        },
        "header": {
            # "stamp": {"secs": 1722428091, "nsecs": 145342111},
            "frame_id": "map",
            # "seq": 170,
        },
        "pose": {
            "pose": {
                "position": {
                    "y": 1611.609375,
                    "x": -5169.197265625,
                    "z": -2291.408447265625,
                },
                "orientation": {
                    "y": -0.27018759106822815,
                    "x": -0.27589386856660403,
                    "z": -0.7944275070923245,
                    "w": 0.4687923060848089,
                },
            },
            "covariance": [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        },
        "child_frame_id": "base_link",
    },
    "op": "publish",
}


t = message_converter.convert_dictionary_to_ros_message(Odometry, msg["msg"])

print(t)
