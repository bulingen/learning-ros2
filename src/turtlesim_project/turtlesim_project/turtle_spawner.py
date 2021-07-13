#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from functools import partial
from turtlesim.srv import Spawn, Kill
from turtlesim_project_interfaces.msg import Turtle, TurtleArray
from turtlesim_project_interfaces.srv import CatchTurtle


ALIVE_TURTLES_TOPIC = 'alive_turtles'
CATCH_TURTLE_SERVICE = 'catch_turtle'
KILL_TURTLE_SERVICE = 'kill'

SPAWN_PERIOD_IN_SEC = 5.0

X_BOUNDS = {
    'min': 0.0,
    'max': 11.0,
}

# 0.0 is pointing to the right
# negative is turning right
# positive is turning left
THETA_BOUNDS = {
    'min': -math.pi,
    'max': math.pi,
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
        self.spawn_timer = self.create_timer(SPAWN_PERIOD_IN_SEC, self.spawn)
        self.alive_turtles = []
        self.publisher = self.create_publisher(TurtleArray, ALIVE_TURTLES_TOPIC, 10)
        self.server = self.create_service(
            CatchTurtle,
            CATCH_TURTLE_SERVICE,
            self.handle_catch_turtle,
        )
        self.kill_client = None
        self.get_logger().info("turtle_spawner started")

    def handle_catch_turtle(self, request, response):
        if not self.kill_client:
            self.kill_client = self.create_client(Kill, KILL_TURTLE_SERVICE)

        while not self.kill_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server {}".format(KILL_TURTLE_SERVICE))

        kill_request = Kill.Request()
        kill_request.name = request.turtle_name

        future = self.kill_client.call_async(kill_request)

        future.add_done_callback(
            partial(self.handle_kill_turtle_response, name=request.turtle_name)
        )
        response.success = True

        return response

    def handle_kill_turtle_response(self, future, name):
        try:
            future.result()
            self.alive_turtles = list(filter(lambda t: t['name'] != name, self.alive_turtles))
            self.publish_turtles()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

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
