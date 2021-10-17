#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import Servo


class ServoNode2(Node):
    def __init__(self):
        super().__init__("servo_2")


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
