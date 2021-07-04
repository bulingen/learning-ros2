#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class LedPanel(Node):
    def __init__(self):
        super().__init__("led_panel")


def main(args=None):
    rclpy.init(args=args)
    node = LedPanel()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
