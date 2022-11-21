FROM ros:foxy-ros-base
LABEL maintainer="Emanuel Nunez S gmail dot com"
ENV HOME /root
WORKDIR $HOME
SHELL ["/bin/bash", "-c"]

# As per instructions on
# https://support.franka.de/docs/franka_ros2.html

# basic utilities
RUN apt-get update && apt-get install -y \
    	apt-utils \
    	dialog

# general utilities
RUN apt-get update && apt-get install -y \
    	wget \
    	curl \
    	git \
    	gdb \
    	vim \
    	nano \
    	unzip \
    	iputils-ping

# install ros2 packages
RUN apt-get update && apt-get install -y \ 
	ros-foxy-control-msgs \
	ros-foxy-xacro \
	ros-foxy-angles \
	ros-foxy-ros2-control \
	ros-foxy-realtime-tools \
	ros-foxy-control-toolbox \
	ros-foxy-moveit \
	ros-foxy-ros2-controllers \
	ros-foxy-joint-state-publisher \
	ros-foxy-joint-state-publisher-gui \
	ros-foxy-ament-cmake-clang-format \
	python3-colcon-common-extensions

RUN apt-get install iputils-ping

# Magic?
# RUN apt-get update &&  apt-get dist-upgrade -y
#RUN apt-get update &&  apt-get upgrade

# SET ENVIRONMENT
WORKDIR $HOME/ws/franka_emika_panda/
 
#### Step 0: Prerequisites: building and setting up libfranka
# Done
# Known bug, no real time system

RUN apt-get update && apt-get install -y \ 
	libpoco-dev \
	libeigen3-dev &&\
	git clone https://github.com/frankaemika/libfranka.git --recursive &&\
	cd libfranka &&\
	mkdir build && cd build &&\
	cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF  .. &&\
	cmake --build . -j$(nproc) &&\
	cpack -G DEB &&\
	sudo dpkg -i libfranka-*.deb

RUN echo 'echo "Updating bash.rc" &&\
	 export RCUTILS_COLORIZED_OUTPUT=1 &&\
	 export LC_NUMERIC=en_US.UTF-8' >> $HOME/.bashrc

#### Step 1: Setup: building franka_ros2
# TODO: change /home to ~ or $HOME

WORKDIR $HOME/ws/
RUN mkdir -p franka_ros2_ws/src   && \ 
	cd $HOME/ws/franka_ros2_ws && \
	git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2 && \
	source /opt/ros/foxy/setup.bash && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
	source install/setup.sh

#WORKDIR $HOME/ws/franka_ros2_ws/src/franka_ros2

# Source ROS2
#RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && source /home/user/ws/franka_ros2_ws/install/setup.bash"

RUN echo 'source /opt/ros/foxy/setup.bash &&\
	source $HOME/ws/franka_ros2_ws/install/setup.sh' >> $HOME/.bashrc

#CMD ["ros2", "launch", "franka_moveit_config", "moveit.launch.py", "robot_ip:=172.16.10.1"]

RUN echo 'ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.10.1' >> $HOME/.bashrc

# source /opt/ros/foxy/setup.bash && source /home/user/ws/franka_ros2_ws/install/setup.bash

# ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.10.1 use_rviz:=true
# ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.10.1
# ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.10.1

# Move to start sample
# ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=172.16.10.1

## franka_ros2/franka_example_controllers/src/move_to_start_example_controller.cpp 

WORKDIR $HOME/ws/
COPY move_to_start_example_controller.cpp  $HOME/ws/franka_ros2_ws/src/franka_ros2/franka_example_controllers/src/
RUN source /opt/ros/foxy/setup.bash && \ 
	source franka_ros2_ws/install/setup.sh && \
	colcon build --packages-select franka_example_controllers
	

