xhost +local:root

#docker image build -t ros2-panda .     
docker pull enunezs/ros2_franka:foxy

docker run -it \
	--env="DISPLAY" \
	--device=/dev/video0:/dev/video0 \
	-e DISPLAY=$DISPLAY \
	--env="QT_X11_NO_MITSHM=1" \
	--net=host \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--privileged \
	-e "ROS_DOMAIN_ID=7" \
	-v /dev/shm:/dev/shm \
	-u 0 enunezs/ros2_franka:foxy


# sudo apt update &&  sudo apt install ros-foxy-rmw-fastrtps-cpp ros-foxy-rmw-fastrtps-shared-cpp
# ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.10.1

export containerId=$(docker ps -l -q)


