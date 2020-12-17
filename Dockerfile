FROM osrf/ros:foxy-desktop

# to get all packages needed to run ros2
RUN apt-get update -y && apt-get upgrade -y

# to be able to run any ros2 commands at all
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# to get ros2 auto completion in bash 
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

RUN mkdir /ros2_ws
RUN mkdir /ros2_ws/src

WORKDIR /ros2_ws

COPY . /ros2_ws/

RUN /bin/bash -c 'source /opt/ros/foxy/setup.bash; colcon build'

RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /ros2_ws/src

# to keep container running
CMD tail -f /dev/null