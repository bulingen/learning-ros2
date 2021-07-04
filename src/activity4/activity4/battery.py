#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from activity4_interfaces.srv import SetLed


class Battery(Node):
    def __init__(self):
        super().__init__("battery")
        self.battery_state = "full"
        self.last_time_battery_state_changed = self.get_current_time_in_seconds()
        self.batter_timer = self.create_timer(0.1, self.check_battery_state)
        self.get_logger().info("battery node started")

    def get_current_time_in_seconds(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0

    def check_battery_state(self):
        time_now = self.get_current_time_in_seconds()
        if self.battery_state == "full":
            if time_now - self.last_time_battery_state_changed > 4.0:
                self.battery_state = "empty"
                self.get_logger().info("battery is empty")
                self.last_time_battery_state_changed = time_now
                self.call_set_led_server(3, True)
        else:
            if time_now - self.last_time_battery_state_changed > 6.0:
                self.battery_state = "full"
                self.get_logger().info("battery is full")
                self.last_time_battery_state_changed = time_now
                self.call_set_led_server(3, False)

    def call_set_led_server(self, led_number, state):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server Add Two Ints...")

        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        future = client.call_async(request)
        # use func_tools.partial to be able to pass additional parameters
        future.add_done_callback(
            partial(self.handle_server_response, led_number=led_number, state=state)
        )

    # passing in request values a and be as well, to be able to match request
    # with correct response
    def handle_server_response(self, future, led_number, state):
        try:
            response = future.result()
            self.get_logger().info(str(response.success))

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = Battery()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
