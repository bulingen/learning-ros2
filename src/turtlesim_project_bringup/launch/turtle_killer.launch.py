from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    turtle_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtle_simulator",
    )

    controller_node = Node(
        package="turtlesim_project",
        executable="turtle_controller",
        name="controller",
    )
    
    spawner_node = Node(
        package="turtlesim_project",
        executable="turtle_spawner",
        name="spawner",
        parameters=[
            {"spawn_frequency": 0.2}
        ]
    )

    ld.add_action(turtle_node)
    ld.add_action(controller_node)
    ld.add_action(spawner_node)

    return ld
