#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import adafruit_bno055
import board


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        self.imu_publisher = self.create_publisher(String, "imu_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish_imu)
        self.get_logger().info("IMU status publisher has been started.")

        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

    def publish_imu(self):
        msg = String()
        val = "temp: {}  euler: {}  grav: {}".format(self.sensor.temperature, self.sensor.euler, self.sensor.gravity)
        msg.data = "" + val
        self.imu_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
