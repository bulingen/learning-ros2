#!/usr/bin/env python3
import rclpy
import serial
import pynmea2
from rclpy.node import Node
from example_interfaces.msg import String

import time
import datetime


class GnssNode(Node):
    def __init__(self):
        super().__init__("gnss")
        self.publisher = self.create_publisher(String, "gnss_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish)
        self.serial_source = serial.Serial("/dev/ttyS0", 9600, timeout=2)
        self.get_logger().info("GNSS status publisher has been started.")

    def publish(self):
        # msg = String()
        # stuff = ""
        # val = "gnss stuff: {}".format(stuff)
        # msg.data = "" + val
        # self.publisher.publish(msg)

        line = self.serial_source.readline()
        decoded = line.decode('utf-8')

        if "GGA" in decoded:
            parsed = pynmea2.parse(decoded)
            # print(repr(parsed))
            # time.localtime(
            print(parsed.latitude, parsed.longitude, parsed.timestamp, type(parsed.timestamp), dir(parsed.timestamp), parsed.timestamp.tzinfo, parsed.timestamp.tzname(), parsed.timestamp.utcoffset())


        # print(decoded)


def main(args=None):
    rclpy.init(args=args)
    node = GnssNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
