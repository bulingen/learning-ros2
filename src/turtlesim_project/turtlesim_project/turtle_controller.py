#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info("turtle_controller started")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
