ARG ROS_DISTRO=humble
<<<<<<< .merge_file_QcC6N4
ARG DISPLAY
=======
<<<<<<< HEAD
FROM ros:$ROS_DISTRO-ros-base
=======
>>>>>>> .merge_file_wWOwY9
FROM ros:$ROS_DISTRO-ros-base as ros-kinova
>>>>>>> 32b90d2f783f4f6db2785f3dcba0953584cc9a6d
LABEL maintainer="Emanuel Nunez S gmail dot com"
ENV HOME /root
WORKDIR $HOME
SHELL ["/bin/bash", "-c"]

<<<<<<< HEAD
# As per instructions on
# https://support.franka.de/docs/franka_ros2.html
=======
# As per instructions on https://github.com/RRL-ALeRT/kinova-ros2
>>>>>>> 32b90d2f783f4f6db2785f3dcba0953584cc9a6d

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
# TODO: Pending to remove
RUN apt-get update && apt-get install -y \ 
	ros-$ROS_DISTRO-control-msgs \
	ros-$ROS_DISTRO-xacro \
	ros-$ROS_DISTRO-angles \
	ros-$ROS_DISTRO-ros2-control \
	ros-$ROS_DISTRO-realtime-tools \
	ros-$ROS_DISTRO-control-toolbox \
	ros-$ROS_DISTRO-moveit \
	ros-$ROS_DISTRO-ros2-controllers \
	ros-$ROS_DISTRO-joint-state-publisher \
	ros-$ROS_DISTRO-joint-state-publisher-gui \
	ros-$ROS_DISTRO-ament-cmake-clang-format \
	ros-$ROS_DISTRO-backward-ros \
	python3-colcon-common-extensions

RUN apt-get update && apt-get install -y \
	iputils-ping \
	libusb-1.0-0

# RUN apt-get update &&  apt-get dist-upgrade -y

# SET ENVIRONMENT
<<<<<<< HEAD
WORKDIR $HOME/ws/franka_emika_panda/
 
#### Step 0: Prerequisites: building and setting up libfranka (ROS version independent)
# Done
# Known bug, no real time system
=======
#WORKDIR $HOME/ws/kinova-ros2/
>>>>>>> 32b90d2f783f4f6db2785f3dcba0953584cc9a6d

#### Step 1: Setup: building franka_ros2

WORKDIR $HOME/ws/

RUN mkdir -p kinova-ros2/src   && \ 
	cd $HOME/ws/kinova-ros2/src/ && \
	git clone https://github.com/RRL-ALeRT/kinova-ros2.git -b $ROS_DISTRO &&\
	mkdir -p /etc/udev/rules.d/ && \ 
	cp kinova-ros2/kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/ && \ 
	source /opt/ros/$ROS_DISTRO/setup.bash && \
	colcon build --symlink-install --packages-select kinova_msgs kinova_bringup kinova_driver kinova_description kinova_demo

#### SET ENVIRONMENT

#RUN echo 'alias python="python3"' >> $HOME/.bashrc
#RUN echo 'source /opt/ros/$ROS_DISTRO/setup.sh && colcon build' >> $HOME/.bashrc
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.sh' >> $HOME/.bashrc
RUN echo 'source kinova-ros2/src/install/setup.bash' >> $HOME/.bashrc

RUN echo 'echo "Updating bash.rc" &&\
	 export RCUTILS_COLORIZED_OUTPUT=1 &&\
	 export LC_NUMERIC=en_US.UTF-8' >> $HOME/.bashrc
	 

<<<<<<< HEAD
#### Step 1: Setup: building franka_ros2

WORKDIR $HOME/ws/
RUN mkdir -p franka_ros2_ws/src   && \ 
	cd $HOME/ws/franka_ros2_ws && \
	git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2 && \
	source /opt/ros/$ROS_DISTRO/setup.bash && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
	source install/setup.sh
=======
### Split here # Seems we dont need this!
#FROM ros-kinova as ros-kinova-SDK
#COPY --from=kinova-ros2 /bin/hello /bin/hello

## Install Jaco SDK

COPY Ubuntu $HOME/ws/kinova-ros2/sdk/
WORKDIR $HOME/ws/kinova-ros2/sdk/64bits
RUN sudo dpkg -i $HOME/ws/kinova-ros2/sdk/64bits/KinovaAPI-5.2.0-amd64.deb 

WORKDIR $HOME/ws/
### TEMP ONLY
### Split here # Seems we dont need this!
#FROM ros-kinova as ros-kinova-SDK
#./jaco2Install64_1.0.0 # sh /root/ws/kinova-ros2/sdk/64bits/jaco2Install64_1.0.0

WORKDIR $HOME/ws/

#FROM x11docker/fvwm
ARG DISPLAY
RUN apt-get update && apt-get install -y \ 
	xterm
RUN sudo xterm ./kinova-ros2/sdk/64bits/jaco2Install64_1.0.0

>>>>>>> 32b90d2f783f4f6db2785f3dcba0953584cc9a6d

#RUN sh /opt/kinova/GUI/DevelopmentCenter.sh 

<<<<<<< HEAD
# Source ROS2
# TODO: change /home to ~ or $HOME
#RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /home/user/ws/franka_ros2_ws/install/setup.bash"

RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash &&\
	source $HOME/ws/franka_ros2_ws/install/setup.sh' >> $HOME/.bashrc
=======

#RUN echo 'ros2 launch kinova_bringup kinova_robot_launch.py' >> $HOME/.bashrc
>>>>>>> 32b90d2f783f4f6db2785f3dcba0953584cc9a6d

#COPY servo_teleop.launch.py $HOME/ws_moveit2_tut/src/moveit2_tutorials/doc/examples/realtime_servo/launch/


<<<<<<< HEAD
# source /opt/ros/$ROS_DISTRO/setup.bash && source /home/user/ws/franka_ros2_ws/install/setup.bash
=======
>>>>>>> 32b90d2f783f4f6db2785f3dcba0953584cc9a6d

### Run stuff 
# "Normal"
#RUN echo 'ros2 launch kinova_bringup kinova_robot_launch.py' >> $HOME/.bashrc

# Interactive mode
#RUN echo 'ros2 run kinova_driver kinova_interactive_control j2n6s300' >> $HOME/.bashrc
# Moveit
#RUN echo 'ros2 run kinova_driver joint_trajectory_action_server j2n6s300' >> $HOME/.bashrc
#RUN echo 'ros2 run kinova_driver gripper_command_action_server j2n6s300' >> $HOME/.bashrc
#RUN echo 'ros2 launch kinova_bringup moveit_robot_launch.py' >> $HOME/.bashrc

## franka_ros2/franka_example_controllers/src/move_to_start_example_controller.cpp 

WORKDIR $HOME/ws/
COPY move_to_start_example_controller.cpp  $HOME/ws/franka_ros2_ws/src/franka_ros2/franka_example_controllers/src/
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \ 
	source franka_ros2_ws/install/setup.sh && \
	colcon build --packages-select franka_example_controllers
	


