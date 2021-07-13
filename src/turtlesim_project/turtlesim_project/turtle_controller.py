#!/usr/bin/env python3
import rclpy
import math
from functools import partial
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3
from turtlesim_project_interfaces.msg import Turtle, TurtleArray
from turtlesim_project_interfaces.srv import CatchTurtle


COMMAND_TOPIC = '/turtle1/cmd_vel'  # Twist
STATUS_TOPIC = '/turtle1/pose'  # Pose
ALIVE_TURTLES_TOPIC = 'alive_turtles'  # TurtleArray
CATCH_TURTLE_SERVICE = 'catch_turtle'

# top left (ish)
GOAL_TOP_LEFT = {'x': 1.0, 'y': 10.0}

# bottom right (ish)
GOAL_BOTTOM_RIGHT = {'x': 10.0, 'y': 1.0}
GOAL_TOP_RIGHT = {'x': 10.0, 'y': 10.0}
GOAL_BOTTOM_LEFT = {'x': 1.0, 'y': 1.0}

GOAL = GOAL_BOTTOM_RIGHT

# maybe send commands at some interval and calculate stuff at another?
# 1 sec / 2 = 0.5
COMMAND_PERIOD = 0.1

# TODO: maybe increase this, to handle problems with not reaching target
# when target gets spawned very close
# angle_vel * 2
ANGLE_VELOCITY_COEFFICIENT = 2

# a full lap is 2 * PI
ANGLE_TOLERANCE = 0.1

# the canvas side is 11.0
DISTANCE_TOLERANCE = 0.3


def calculate_distance(pos1_x, pos1_y, pos2_x, pos2_y):
    x_distance = pos2_x - pos1_x
    y_distance = pos2_y - pos1_y
    return math.sqrt(math.pow(x_distance, 2) + math.pow(y_distance, 2))


def calculate_angle(orig_x, orig_y, goal_x, goal_y):
    return math.atan2(goal_y - orig_y, goal_x - orig_x)


def get_turn_angle(target_angle, source_angle, use_radians=True):
    half_lap = math.pi if use_radians else 180
    full_lap = 2 * half_lap

    angle_diff = target_angle - source_angle
    if angle_diff > half_lap:
        angle_diff -= full_lap
    elif angle_diff < -half_lap:
        angle_diff += full_lap

    return angle_diff


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.turtle1_subscription = self.create_subscription(
                Pose, STATUS_TOPIC, self.handle_status_update, 10)
        self.alive_turtles_subscription = self.create_subscription(
                TurtleArray, ALIVE_TURTLES_TOPIC, self.handle_turtles_update, 10)
        self.publisher = self.create_publisher(Twist, COMMAND_TOPIC, 10)
        self.catch_turtle_client = None
        self.last_time_command_sent = self.get_current_time_in_seconds()
        self.current_position = None
        self.target_turtle = None

        self.get_logger().info("turtle_controller started")

    def catch_turtle(self, name):
        if not self.catch_turtle_client:
            self.catch_turtle_client = self.create_client(CatchTurtle, CATCH_TURTLE_SERVICE)
        
        while not self.catch_turtle_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server {}".format(CATCH_TURTLE_SERVICE))

        request = CatchTurtle.Request()
        request.turtle_name = name

        self.catch_turtle_client.call_async(request)

    def get_current_time_in_seconds(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0

    def handle_turtles_update(self, msg):
        if len(msg.turtles) > 0 and not self.target_turtle:
            self.target_turtle = msg.turtles[0]
            
    def get_driving_instructions(self):
        linear_velocity = 0.0
        angle_velocity = 0.0

        if not self.target_turtle:
            return linear_velocity, angle_velocity

        distance_to_target = calculate_distance(
            self.target_turtle.x,
            self.target_turtle.y,
            self.current_position.x,
            self.current_position.y
        )
        reached_target = distance_to_target < DISTANCE_TOLERANCE

        if reached_target:
            if self.target_turtle:
                # TODO: maybe refactor how commands are published in the node.
                # A publish_command here works, but is a bit misplaced.
                # Maybe have helper functions for stop(), go_to_target() etc.
                self.publish_command(0.0, 0.0)
                self.catch_turtle(self.target_turtle.name)
                self.target_turtle = None
            return linear_velocity, angle_velocity

        linear_velocity = 2.0
        angle_to_target = calculate_angle(
            self.current_position.x,
            self.current_position.y,
            self.target_turtle.x, self.target_turtle.y
        )
        turn_angle = get_turn_angle(angle_to_target, self.current_position.theta)
        angle_velocity = ANGLE_VELOCITY_COEFFICIENT * turn_angle

        if abs(turn_angle) <= ANGLE_TOLERANCE:
            angle_velocity = 0.0

        return linear_velocity, angle_velocity

    def handle_status_update(self, msg):
        time_now = self.get_current_time_in_seconds()
        if time_now - self.last_time_command_sent > COMMAND_PERIOD:

            self.current_position = msg
            linear_velocity, angle_velocity = self.get_driving_instructions()

            self.publish_command(linear_velocity, angle_velocity)
            self.last_time_command_sent = time_now

    def publish_command(self, linear_velocity, angle_velocity):
        msg = Twist()
        linear = Vector3()
        linear.x = linear_velocity
        angular = Vector3()
        angular.z = angle_velocity
        msg.linear = linear
        msg.angular = angular
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
