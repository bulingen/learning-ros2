from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    number_publisher_node = Node(
        package="activity2",
        executable="number_publisher"
    )

    number_counter_node = Node(
        package="cpp_activity2",
        executable="number_counter"
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)

    return ld
