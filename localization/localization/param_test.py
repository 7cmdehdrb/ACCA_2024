import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("param_test_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("frequency", 0.0),
            ],
        )
        self.get_logger().info("HELLO")

        freq = self.get_parameter("frequency").get_parameter_value().double_value
        self.get_logger().info("FREQ: {}".format(freq))
        # 파라미터 읽기


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
