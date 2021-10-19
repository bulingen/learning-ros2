#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from example_interfaces.msg import String


class GnssNode(Node):
    def __init__(self):
        super().__init__("gnss")
        self.publisher = self.create_publisher(String, "gnss_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish)
        self.serial_source = serial.Serial("/dev/ttyS0")
        self.get_logger().info("GNSS status publisher has been started.")

    def publish(self):
        # msg = String()
        # stuff = ""
        # val = "gnss stuff: {}".format(stuff)
        # msg.data = "" + val
        # self.publisher.publish(msg)

        print(self.serial_source.readline())


def main(args=None):
    rclpy.init(args=args)
    node = GnssNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
