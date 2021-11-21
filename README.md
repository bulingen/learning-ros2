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
- `ros2 run my_cpp_pkg cpp_node --ros-args -r __node:=new_name -r topic_name:=new_name_of_topic`
- `ros2 interface show example_interfaces/srv/AddTwoInts` (Will show service definition: request and response types separated by three dashes.)
- `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"` (Call a service from CLI).
- `ros2 service list`
- `rqt` --> select in menu "Plugins" --> "Services" --> "Service Caller". Great for testing service calls when the interface is complex.
- `ros2 run my_py_pkg add_two_ints_server --ros-args -r add_two_ints:=new_name` (Remap a service with a different name.)
- `ros2 run my_py_pkg add_two_ints_client --ros-args -r add_two_ints:=new_name` (Remap the client as well, to make use of the server.)

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
- While topics are a way for unidirectional communication between nodes, services are a client/server setup, where clients request something from the server.
- In that case, the service is the "interface" between the client and the server.
- The client will do a request and receive a response. 
- A service can be either synchronous or asynchronous.
- A service is defined by its name and the message types of the requests and responses.
- A service server can exist only once, but can have many clients.
- "Am I just sending some data, or do I expect a response after I send the message?”. This will tell you if you need to use a Topic or a Service

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

## How to create an interface package

Create a separate package for message and service definitions, for easier dependency management later on. This package does not need any dependencies, since it won't have any program code. So the first `pkg create` command does not have any dependencies. However, some changes need to be done to the `package.xml` and `CMakeLists.txt` files.

- `ros2 pkg create my_interfaces`
- `cd my_interfaces`
- `rm -rf include/`
- `rm -rf src/`
- `mkdir msg`

Open `package.xml` and add between `<buildtool_depend>` and `<test_depend>`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Edit `CMakeLists.txt` like this:
- Remove `# Default to C99` section.
- Remove `if(BUILD_TESTING)` block.
- Add to `# find dependencies` section:
  - `find_package(rosidl_default_generators REQUIRED)`

Add file to `msg` folder, e. g. `SomeStatus.msg`:
```
int64 some_int_value
bool is_ready
string debug_message
```

Add to `CMakeLists.txt`, after `# find dependencies` section:

```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SomeStatus.msg"
)
```

Build the package:
```sh
cd /root/of/project
colcon build --packages-select my_interfaces
```

Verify that it's there:
```sh
source ~/.bashrc
ros2 interface show my_interfaces/msg/SomeStatus
```

## How to use parameters

Declare and use parameters in the constructor of the node like this:

```python
self.declare_parameter("some_name", "default_value")
self.my_value = self.get_parameter("some_name").value
```

```cpp
this->declare_parameter("some_name", 100);
int my_value = this->get_parameter("some_name").as_int();
```

Set the parameters during runtime:

```bash
ros2 run my_pkg my_node --ros-args -p some_name=the_value
```

The type of the value will be automatically interpreted by ROS2 - int, string, double and so on.

List and get the parameters during runtime:

```bash
ros2 param list
```

```bash
ros2 param get /the_node the_param
```

## How to use launch files

1. Create a new package for the launch files. Nice to have them separated as in the case of interfaces. And also, the launch files will probably use nodes from different packages, so they don't really fit inside any of the other ones.
   ```bash
   ros2 pkg create my_robot_bringup
   ```
1. Remove `/include` and `/src` directories inside the new package.
1. Create a `/launch` directory inside the new package.
1. In the `CMakeLists.txt` file, remove the `Default to C99` section. Also remove the `if(BUILD_TESTING)` block.
1. Add instructions on how to install the launch files (right after `find_package(...)` section):
   ```
   install(DIRECTORY
     launch
     DESTINATION share/${PROJECT_NAME}
   )
   ```
1. Create a launch file inside the `/launch` folder, e g `my_robot_app.launch.py`.
1. Make it executable: `chmod +x launch/my_robot_app.launch.py`.
1. Add the minimal piece of code to the launch file:
   ```python
   from launch import LaunchDescription

   def generate_launch_description():
     ld = LaunchDescription()
     return ld
   ```
1. Build the package and source your terminal. Then run:
   ```bash
   ros2 launch my_robot_bringup my_robot_app.launch.py
   ```
1. Add configuration on how to run your application in the launch file. See `src/my_robot_bringup/launch/number_app.launch.py` for reference.
1. Add exec dependencies to the `package.xml`. See `src/my_robot_bringup/package.xml` for reference.

## How to make VS Code accept your local imports

1. Open User settings
1. Search for "python path"
1. Click "Edit in settings.json" under "Auto complete: extra paths"
1. Add to the array `python.autoComplete.extraPaths` for example:
  ```
  "~/learning-ros2/install/my_robot_interfaces/lib/python3.8/site-packages/my_robot_interfaces"
  ```

## How to use your custom message in a python node

1. Create a python node
1. Import message: `from my_robot_interfaces.msg import HardwareStatus`
1. (Fix your VS Code python imports as described above)
1. Add to `package.xml`:
  ```
  <depend>my_robot_interfaces</depend>
  ```
1. You can now use the message as any other message.

## Turtlesim project notes

### Get started

Get source code and build:

```
git clone <this repo>
cd path/to/repo
colcon build --packages-select turtlesim_project turtlesim_project_interfaces
source source_me.sh
```

Run nodes:

(terminal 1)
```
ros2 run turtlesim turtlesim_node
```

(terminal 2)
```
ros2 run turtlesim_project turtle_controller
```

(terminal 3)
```
ros2 run turtlesim_project turtle_spawner
```

### More turtlesim project notes

`colcon build --packages-select turtlesim_project`
`ros2 run turtlesim_project turtle_spawner`
`ros2 service type /spawn`
`ros2 interface show turtlesim/srv/Spawn`
`ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 1.0, theta: 45.0, name: 'a_name'}"`

bounds
x: 0.0 - 11.0
y: 0.0 - 11.0
theta: 0.0 - 2*PI

`colcon build --packages-select turtlesim_project_interfaces`
`ros2 run turtlesim_project turtle_controller`

`ros2 topic echo /turtle1/cmd_vel`
`ros2 topic info /turtle1/cmd_vel`
Type: geometry_msgs/msg/Twist

`ros2 topic echo /turtle1/pose`
`ros2 topic info /turtle1/pose`
Type: turtlesim/msg/Pose

`ros2 interface show geometry_msgs/msg/Twist`
This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular

`ros2 interface show geometry_msgs/msg/Vector3`
This represents a vector in free space.

This is semantically different than a point.
A vector is always anchored at the origin.
When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z

`ros2 interface show turtlesim/msg/Pose`
float32 x
float32 y
float32 theta

float32 linear_velocity
float32 angular_velocity


`ros2 service call /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 4.0, y: 4.0}}"`

`ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0}, angular: {z: 1.0}}" --once`

`ros2 run rqt_plot rqt_plot`
Topic: /turtle1/pose/theta

## Troubleshoot

- `ModuleNotFoundError` - Have you sourced your shell? Remember to run `source source_me.sh` everytime you open a new shell.


## Some notes on working with hardware on a Raspberry Pi

### Notes on running `servo_node`

- Haven't figured out a nice way of working with dependencies so far. There are three ways of installing stuff:
  - using `rosdep`
  - using `apt-get` or similar
  - using `pip install`
- Check out these links on installing stuff:
  - http://docs.ros.org/en/galactic/How-To-Guides/Using-Python-Packages.html?highlight=rosdep
  - https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml
  - https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml
- For now, install GPIO Python lib like this: `sudo apt-get -y install python3-rpi.gpio`
- Add your user to the `gpio` group using either `user.py` or `user2.py`, can't remember.
- Restart the Raspberry Pi.
- Try out the servo like this: `ros2 run rpi_test servo_node`.
- Is this a better option? `sudo pip3 install gpiozero`

### Notes on running `imu`

Plug in the BNO055 module to some pins (not sure which ones..? Possibly like this: https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/hardware)


```
cd /some/path/outside/this/folder
git clone https://github.com/adafruit/Adafruit_CircuitPython_BNO055.git
cd /path/to/cloned-repo
sudo python3 setup.py install
```

Check that you have permission to run I2C stuff.

Open a Python shell: `python3`

```python
import adafruit_bno055
import board
i2c = board.I2C()
```

This will probably give you a permission error. Do this to resolve:

```
sudo apt update 
sudo apt upgrade -y
sudo apt install -y i2c-tools
sudo usermod -a -G i2c ubuntu // if ubuntu is your user
```

Should `adafruit_bno055` be listed in package dependencies? No idea on how to handle dependencies that are resolved locally like this.

However, you should be able to run the `imu` node now:

```
cd /path/to/this/project
colcon build --packages-select rpi_test
source source_me.sh
ros2 run rpi_test imu
```

### Notes on running `gnss` node

Plug in NEO 6M module. Possibly like this: https://medium.com/@kekreaditya/interfacing-u-blox-neo-6m-gps-module-with-raspberry-pi-1df39f9f2eba


See if it works:

```python
import serial
ser = serial.Serial("/dev/ttsy0")
print(ser.readline())
```

This will probably not work, since another service is using the same port. Follow these steps to solve (stolen from https://askubuntu.com/a/1338744). Or maybe do as the notes say in `config.txt` and edit the `usercfg.txt` file instead, but I haven't tried that.

1. Backup `config.txt` and `cmdline.txt` files:
    ```bash
    sudo cp -pr /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt-orig
    sudo cp -pr /boot/firmware/config.txt /boot/firmware/config.txt-orig
    ```
1. Edit `/boot/firmware/config.txt` to comment out the `enable_uart=1`
    ```
    #enable_uart=1

    cmdline=cmdline.txt
    ```
1. Remove the console setting `console=serial0,115200` from `/boot/firmware/cmdline.txt`
1. Disable the Serial Service which used the miniUART
    ```
    sudo systemctl stop serial-getty@ttyS0.service
    sudo systemctl disable serial-getty@ttyS0.service
    sudo systemctl mask serial-getty@ttyS0.service
    ```
1. Add the user which will use the miniUART to tty and dialout group
    ```
    sudo adduser ${USER} tty
    sudo adduser ${USER} dialout
    ```
1. Reboot


Install this lib: `pip install pynmea2`

Build and run:

```
colcon build --packages-select rpi_test
source source_me.sh
ros2 run rpi_test gnss
```

### More servo notes

`sudo apt install pigpio` not working

https://abyz.me.uk/rpi/pigpio/download.html
Follow the "download and install" instructions.

sudo pigpiod
sudo killall pigpiod

sudo apt install python3-gpiozero python3-pigpio