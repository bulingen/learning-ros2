#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
