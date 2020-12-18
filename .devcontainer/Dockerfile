FROM osrf/ros:foxy-desktop

RUN apt-get update -y && apt-get upgrade -y

RUN apt-get -y install --no-install-recommends python3-pip

# to be able to run any ros2 commands at all
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# to get ros2 auto completion in bash 
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
