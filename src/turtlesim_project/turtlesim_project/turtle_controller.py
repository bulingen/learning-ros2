#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3

COMMAND_TOPIC = '/turtle1/cmd_vel'  # Twist
STATUS_TOPIC = '/turtle1/pose'  # Pose

# top left (ish)
GOAL = {'x': 1.0, 'y': 10.0}

# 1 sec / 2 = 0.5
COMMAND_PERIOD = 0.5

# angle_vel * 2
ANGLE_VELOCITY_COEFFICIENT = 2

ANGLE_TOLERANCE = 0.2

# Pose example
# x: 5.3709917068481445
# y: 5.558200359344482
# theta: -0.1742475926876068
# linear_velocity: 0.0
# angular_velocity: 0.0


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
        angle_to_target = calculate_angle(msg.x, msg.y, GOAL['x'], GOAL['y'])
        distance_to_target = calculate_distance(GOAL['x'], GOAL['y'], msg.x, msg.y)
        time_now = self.get_current_time_in_seconds()
        if time_now - self.last_time_command_sent > COMMAND_PERIOD:
            turn_angle = get_turn_angle(angle_to_target, msg.theta)
            angle_velocity = ANGLE_VELOCITY_COEFFICIENT * turn_angle

            if abs(turn_angle) <= ANGLE_TOLERANCE:
                print('we are on track')
                angle_velocity = 0.0

            else:
                if turn_angle > 0:
                    # we're to the right of target angle, so need to turn left
                    print('turn left', turn_angle, 'velocity', angle_velocity)
                    # angle_velocity = 0.4
                    # angle_velocity = -angle_error
                else:
                    print('turn right', turn_angle, 'velocity', angle_velocity)
                    # angle_velocity = -0.4
                    # angle_velocity = -angle_error

            self.publish_command(angle_velocity)
            self.last_time_command_sent = time_now

    def publish_command(self, angle_velocity):
        msg = Twist()
        linear = Vector3()
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
