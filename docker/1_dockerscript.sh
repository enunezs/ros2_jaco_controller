#!/bin/bash
xhost +local:root

docker pull enunezs/kinova-ros2:1.2

docker run -it \
	--device=/dev/video0:/dev/video0 \
	--env DISPLAY=$DISPLAY \
	--env="QT_X11_NO_MITSHM=1" \
	--env "ROS_DOMAIN_ID=7" \
	--privileged \
	--net=host \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume $(pwd)/:/root/ws/jacoarm-ros2 \
	--volume /dev/shm:/dev/shm \
	-u 0 \
	enunezs/kinova-ros2:1.2

export containerId=$(docker ps -l -q)


