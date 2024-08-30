xhost +local:root



docker run -it \
	--env="DISPLAY" \
	--device=/dev/video0:/dev/video0 \
	-e DISPLAY=$DISPLAY \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--privileged \
	--net=host \
	-v /dev/shm:/dev/shm \
	osrf/ros:humble-desktop \
	rqt_graph
#	-e "ROS_DOMAIN_ID=7" \

export containerId=$(docker ps -l -q)



