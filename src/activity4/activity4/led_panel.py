#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from activity4_interfaces.msg import LedState


class LedPanel(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.led_1_active = False
        self.led_2_active = False
        self.led_3_active = False
        self.led_states = [False, False, False]

        self.led_states_publisher_ = self.create_publisher(LedState, "led_states", 10)
        self.led_states_timer_ = self.create_timer(4, self.publish_led_states)
        self.get_logger().info("LED panel node has been started")

    def publish_led_states(self):
        msg = LedState()
        msg.led_1_active = self.led_1_active
        msg.led_2_active = self.led_2_active
        msg.led_3_active = self.led_3_active
        msg.led_states = self.led_states
        self.led_states_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LedPanel()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
