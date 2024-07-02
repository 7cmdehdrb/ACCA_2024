import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class SimpleSubscriber(Node):
    def __init__(self, f):
        super().__init__("test_sub")
        self.subscription = self.create_subscription(
            NavSatFix, "/ublox_gps_node/fix", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        self.f = f

    def listener_callback(self, msg):
        print(msg.latitude, msg.longitude)
        self.f.write("{}, {}\n".format(msg.latitude, msg.longitude))


def main(args=None):
    rclpy.init(args=args)

    f = open("gps_log.txt", "w")

    node = SimpleSubscriber(f=f)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        f.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
