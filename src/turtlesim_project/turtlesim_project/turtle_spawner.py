#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from functools import partial
from turtlesim.srv import Spawn
from turtlesim_project_interfaces.msg import Turtle, TurtleArray


X_BOUNDS = {
    'min': 0.0,
    'max': 11.0,
}

THETA_BOUNDS = {
    'min': 0.0,
    'max': 2 * math.pi,
}


def get_random_float_between(min_val, max_val, count_decimals):
    return round(random.uniform(min_val, max_val), count_decimals)


def get_random_coordinate():
    return get_random_float_between(X_BOUNDS["min"], X_BOUNDS["max"], 1)


def get_random_angle():
    return get_random_float_between(THETA_BOUNDS["min"], THETA_BOUNDS["max"], 2)


def get_random_position():
    x = get_random_coordinate()
    y = get_random_coordinate()
    theta = get_random_angle()
    return {
        'x': x,
        'y': y,
        'theta': theta,
    }


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.spawn_timer = self.create_timer(2.0, self.spawn)
        self.alive_turtles = []
        self.publisher = self.create_publisher(TurtleArray, 'alive_turtles', 10)
        self.get_logger().info("turtle_spawner started")

    def spawn(self):
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('waiting for Spawn server')

        request = Spawn.Request()
        pos = get_random_position()
        request.x = pos['x']
        request.y = pos['y']
        request.theta = pos['theta']

        future = client.call_async(request)

        # use func_tools.partial to be able to pass additional parameters
        future.add_done_callback(
            partial(self.handle_server_response, x=pos['x'], y=pos['y'], theta=pos['theta'])
        )

    def handle_server_response(self, future, x, y, theta):
        try:
            response = future.result()
            self.add_turtle(x, y, theta, response.name)

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def add_turtle(self, x, y, theta, name):
        self.alive_turtles.append({
            'x': x,
            'y': y,
            'theta': theta,
            'name': name,
        })
        self.publish_turtles()

    def publish_turtles(self):
        msg = TurtleArray()
        turtles = []
        for turtle in self.alive_turtles:
            t = Turtle()
            t.x = turtle['x']
            t.y = turtle['y']
            t.theta = turtle['theta']
            t.name = turtle['name']
            turtles.append(t)

        msg.turtles = turtles
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
