#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3

COMMAND_TOPIC = '/turtle1/cmd_vel'  # Twist
STATUS_TOPIC = '/turtle1/pose'  # Pose

# top left (ish)
GOAL_TOP_LEFT = {'x': 1.0, 'y': 10.0}

# bottom right (ish)
GOAL_BOTTOM_RIGHT = {'x': 10.0, 'y': 1.0}
GOAL_TOP_RIGHT = {'x': 10.0, 'y': 10.0}
GOAL_BOTTOM_LEFT = {'x': 1.0, 'y': 1.0}

GOAL = GOAL_BOTTOM_RIGHT

# maybe send commands at some interval and calculate stuff at another?
# 1 sec / 2 = 0.5
COMMAND_PERIOD = 0.2


# angle_vel * 2
ANGLE_VELOCITY_COEFFICIENT = 2

# a full lap is 2 * PI
ANGLE_TOLERANCE = 0.1

# the canvas side is 11.0
DISTANCE_TOLERANCE = 0.2


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
        self.subscriber_ = self.create_subscription(
                Pose, STATUS_TOPIC, self.handle_status_update, 10)
        self.publisher = self.create_publisher(Twist, COMMAND_TOPIC, 10)
        self.last_time_command_sent = self.get_current_time_in_seconds()

        self.get_logger().info("turtle_controller started")

    def get_current_time_in_seconds(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0

    def handle_status_update(self, msg):
        time_now = self.get_current_time_in_seconds()
        if time_now - self.last_time_command_sent > COMMAND_PERIOD:
            distance_to_target = calculate_distance(GOAL['x'], GOAL['y'], msg.x, msg.y)

            linear_velocity = 0.0
            angle_velocity = 0.0

            has_somewhere_to_go = distance_to_target > DISTANCE_TOLERANCE

            if has_somewhere_to_go:
                linear_velocity = 2.0

                angle_to_target = calculate_angle(msg.x, msg.y, GOAL['x'], GOAL['y'])

                turn_angle = get_turn_angle(angle_to_target, msg.theta)
                angle_velocity = ANGLE_VELOCITY_COEFFICIENT * turn_angle

                if abs(turn_angle) <= ANGLE_TOLERANCE:
                    angle_velocity = 0.0

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
