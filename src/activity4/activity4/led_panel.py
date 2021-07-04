#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from activity4_interfaces.msg import LedState
from activity4_interfaces.srv import SetLed


class LedPanel(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.led_1_active = False
        self.led_2_active = False
        self.led_3_active = False
        self.led_states = [False, False, False]

        self.led_states_publisher_ = self.create_publisher(LedState, "led_states", 10)
        self.led_states_timer_ = self.create_timer(4, self.publish_led_states)
        self.set_led_service_ = self.create_service(SetLed, "set_led", self.on_set_led)
        self.get_logger().info("LED panel node has been started")

    def publish_led_states(self):
        msg = LedState()
        msg.led_1_active = self.led_1_active
        msg.led_2_active = self.led_2_active
        msg.led_3_active = self.led_3_active
        msg.led_states = self.led_states
        self.led_states_publisher_.publish(msg)

    def on_set_led(self, request, response):
        led_number = request.led_number
        state = request.state
        
        if led_number > len(self.led_states) or led_number <= 0:
            response.success = False
            return response

        self.led_states[led_number - 1] = state

        if led_number == 1:
            self.led_1_active = state
        elif led_number == 2:
            self.led_2_active = state
        elif led_number == 3:
            self.led_3_active = state

        response.success = True
        self.publish_led_states()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LedPanel()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
