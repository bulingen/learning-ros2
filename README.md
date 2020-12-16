# What is this?

I'm trying to learn ROS2, which is why I've created this Docker environment for running exercises in. I've followed this course: https://www.udemy.com/course/ros2-for-beginners

# Get started

```
git clone [this repo]
cd path/to/repo
docker-compose up -d
```

This will bring up an environment. For instance, you can now step into the running container and create your packages. For example:

```
docker-compose exec dev bash
cd src
ros2 pkg create my_py_pkg --build-type ament_python
cd ..
colcon build
```

Or you could try out default demo nodes like this:

```
docker-compose exec dev bash
ros2 run demo_nodes_cpp talker

// open another terminal and run

docker-compose exec dev bash
ros2 run demo_nodes_cpp listener
```

## Useful commands

- `ros2 run demo_nodes_cpp talker`
- `ros2 run demo_nodes_cpp listener`
- `ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy`
- `ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp`
- `colcon build --packages-select my_py_pkg`

## Useful VS Code extensions

- C/C++
- CMake
- CMake Tools
- Jupyter
- Python

## Notes

- A node is a unit that does something
- A package can contain many nodes
- Nodes communicate with each other through topics
- A main control loop as a node is probably a good thing
