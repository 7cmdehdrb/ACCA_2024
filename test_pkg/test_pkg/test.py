import rclpy
from rclpy.node import Node
import time


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Node has been started")

    def do_something(self):
        self.get_logger().info("Doing something")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    # Set the rate to 2 Hz (0.5 seconds)
    rate = node.create_rate(2)

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.do_something()
            rate.sleep()  # Sleep to enforce the loop rate
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
