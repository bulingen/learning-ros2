#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")

        self.get_logger().info("turtle_spawner started")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
