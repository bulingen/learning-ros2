#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from pynput import keyboard


COMMAND_TOPIC = '/commands'
TURTLE_CMD_TOPIC = '/turtle1/cmd_vel'  # Twist


class CommandState:
    def __init__(self) -> None:
        self.forward = 0
        self.backward = 0
        self.left = 0
        self.right = 0
    
    def __str__(self) -> str:
        return 'forward: {}, backward: {}, left: {}, right: {}'.format(self.forward, self.backward, self.left, self.right)

class CommandInput(Node):
    def __init__(self):
        super().__init__("command_input")
        self.command_publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.turtle_publisher = self.create_publisher(Twist, TURTLE_CMD_TOPIC, 10)

        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_keyboard_press,
            on_release=self.on_keyboard_release)
        self.keyboard_listener.start()

        self.command_state = CommandState()
        self.get_logger().info("command input started. listening for arrow keys")


    def on_keyboard_press(self, key):
        should_publish = False

        if key == keyboard.Key.up:
            self.command_state.forward = 1
            should_publish = True
        if key == keyboard.Key.down:
            self.command_state.backward = 1
            should_publish = True
        if key == keyboard.Key.left:
            self.command_state.left = 1
            should_publish = True
        if key == keyboard.Key.right:
            self.command_state.right = 1
            should_publish = True

        if should_publish:
            self.publish_command_state()

    def on_keyboard_release(self, key):
        should_publish = False
        if key == keyboard.Key.up:
            self.command_state.forward = 0
            should_publish = True
        if key == keyboard.Key.down:
            self.command_state.backward = 0
            should_publish = True
        if key == keyboard.Key.left:
            self.command_state.left = 0
            should_publish = True
        if key == keyboard.Key.right:
            self.command_state.right = 0
            should_publish = True

        if should_publish:
            self.publish_command_state()

    def create_turtle_message(self):
        msg = Twist()
        linear = Vector3()
        angular = Vector3()

        if self.command_state.backward or self.command_state.forward:
            if self.command_state.backward:
                linear.x = -1.0
            if self.command_state.forward:
                linear.x = 2.0

            if self.command_state.left:
                angular.z = 4.0
            
            if self.command_state.right:
                angular.z = -4.0

        msg.linear = linear
        msg.angular = angular
        return msg


    def publish_command_state(self):
        msg = self.create_turtle_message()
        self.turtle_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = CommandInput()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
