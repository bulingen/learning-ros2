#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class Battery(Node):
    def __init__(self):
        super().__init__("battery")


def main(args=None):
    rclpy.init(args=args)
    node = Battery()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
