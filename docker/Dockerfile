ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO-ros-base as ros-kinova
LABEL maintainer="Emanuel Nunez S gmail dot com"
ENV HOME /root
WORKDIR $HOME
SHELL ["/bin/bash", "-c"]

# As per instructions on https://github.com/RRL-ALeRT/kinova-ros2

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
	libusb-1.0-0

#### Step 1: Setup: building kinova_ros2

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
	 

### Split here
#FROM ros-kinova as ros-kinova-SDK
#COPY --from=kinova-ros2 /bin/hello /bin/hello

## Install Jaco SDK

COPY Ubuntu $HOME/ws/kinova-ros2/sdk/
WORKDIR $HOME/ws/kinova-ros2/sdk/64bits
RUN dpkg -i $HOME/ws/kinova-ros2/sdk/64bits/KinovaAPI-5.2.0-amd64.deb 

WORKDIR $HOME/ws/


#FROM ros-kinova as ros-kinova-SDK
WORKDIR $HOME/ws/

ARG DISPLAY
RUN apt-get update && apt-get install -y \ 
	xterm
RUN xterm ./kinova-ros2/sdk/64bits/jaco2Install64_1.0.0

# Clean apt cache
RUN rm -rf /var/lib/apt/lists/*.


# For pip3 
# general utilities
RUN apt-get update && apt-get install -y \
    python3-pip \
    unzip

RUN pip3 install --upgrade pip
RUN pip3 install -U \
	  scipy

# For ROS2 joy
RUN apt-get update && apt-get install -y \ 
	ros-$ROS_DISTRO-joy 

#### SET ENVIRONMENT

RUN echo 'alias python="python3"' >> $HOME/.bashrc
#RUN echo 'source /opt/ros/humble/setup.sh && colcon build' >> $HOME/.bashrc
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-select ros2_jaco_controller' >> $HOME/.bashrc
RUN echo 'source install/setup.bash' >> $HOME/.bashrc





