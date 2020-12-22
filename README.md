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

# Get started with using VS Code Remote-Container extension

1. Install the extension "Remote - Containers"
1. Press `Ctrl`+`Shift`+`P` and select "Remote-Containers: Rebuild and reopen in container"
1. You should now be inside a working environment for ROS2. Open a terminal and do what you want.
1. After your first `colcon build`, you should add `source ~/ros2_workspace/install/setup.bash` to your `.bashrc` file in the container.

If you're using Windows, you might have to do a couple of things to get Git working inside your container. Try `git fetch` and see if you get an error. If that's the case, and you're using SSH keys as credentials, open a Powershell as Administrator and run

```
ssh-add $HOME/.ssh/id_rsa // <-- or whatever keys you are using
```

And if that doesn't work, then you might have to start the SSH Agent:

```
Set-Service ssh-agent -StartupType Automatic
Start-Service ssh-agent
Get-Service ssh-agent
```


## Useful commands

- `ros2 run demo_nodes_cpp talker`
- `ros2 run demo_nodes_cpp listener`
- `ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy`
- `ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp`
- `colcon build --packages-select my_py_pkg`
- `colcon build --packages-select my_py_pkg --symlink-install` (Changes will be reflected without the need for recompilation. The Python file has to be executable. Does not work for CPP.)
- `rqt`
- `rqt_graph`
- `ros2 run turtlesim turtlesim_node`
- `ros2 run turtlesim turtle_teleop_key`
- `ros2 run my_cpp_pkg cpp_node --ros-args -r __node:=new_name`
- `ros2 topic list`
- `ros2 topic echo /name_of_topic`
- `ros2 topic info /name_of_topic`
- `ros2 topic hz /name_of_topic`
- `ros2 topic bw /name_of_topic`
- `ros2 topic pub -r 10 /name_of_topic example_interfaces/msg/String "{data: 'test'}"`
- `ros2 interface show example_interfaces/msg/String`
- `ros2 node list`
- `ros2 node info /name_of_node`

## Useful VS Code extensions

- C/C++
- CMake
- CMake Tools - (not sure about this one)
- Jupyter
- Python

## Notes

- A node is a unit that is responsible for one thing
- A package can contain many nodes
- Nodes communicate with each other through topics, services and parameters
- A main control loop as a node is probably a good thing
- Two different nodes cannot have the same name
- Publishers and receivers are nodes
- A publisher will publish on a certain topic. A subscriber can subscribe to that topic.
- There can be many publishers and many receivers on the same topic.
- A node can contain many publishers/subscribers for many different topics.

## How to create a simple Python node

1. `docker-compose exec dev bash`
1. `cd src`
1. `ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy`
1. Create a Python file: `touch my_py_pkg/my_py_pkg/my_first_node.py`
1. Edit the file and add:

   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node


   def main(args=None):
       rclpy.init(args=args)
       node = Node('py_test')
       node.get_logger().info('Hello ROS2')
       rclpy.spin(node)
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

1. Make the file executable: `chmod +x the_python_file.py`. Then you can execute by running `./the_python_file.py`
1. Add your node to `src/my_py_pkg/setup.py`
   ```python
   entry_points={
       'console_scripts': [
           'py_node = my_py_pkg.my_first_node:main' # <-- this line
       ],
   },
   ```
1. Go to the root of your workspace: `cd /ros2_ws`
1. Build your package: `colcon build --packages-select my_py_pkg`
1. Source again: `source ~/.bashrc`
1. Run your node manually: `./install/my_py_pkg/lib/my_py_pkg/py_node`
1. Run your node through ROS2 commands (source `~/.bashrc` again if necessary): `ros2 run my_py_pkg py_node`

## How to create a simple CPP node

1. `docker-compose exec dev bash`
1. `cd src`
1. `ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp`
1. Create a CPP file: `touch my_cpp_pkg/src/my_first_node.cpp`
1. Edit the file and add:

   ```cpp
   #include "rclcpp/rclcpp.hpp"

   int main(int argc, char **argv)
   {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<rclcpp::Node>("cpp_test");
     RCLCPP_INFO(node->get_logger(), "Hello Cpp Node");
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
   }
   ```

1. Add to `CMakeLists.txt`:

   ```txt
   // find_package...

   add_executable(cpp_node src/my_first_node.cpp)
   ament_target_dependencies(cpp_node rclcpp)

   install(TARGETS
     cpp_node
     DESTINATION lib/${PROJECT_NAME}
   )

   // ament_package...
   ```

1. Run the node with: `/ros2_ws/install/my_cpp_pkg/lib/my_cpp_pkg/cpp_node`
1. Source: `source ~/.bashrc`
1. Run with ROS2 command: `ros2 run my_cpp_pkg cpp_node`
