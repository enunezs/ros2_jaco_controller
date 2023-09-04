xhost +local:root

<<<<<<< .merge_file_u7NzRR
docker image build -t kinova-ros2 --build-arg DISPLAY=:0 --network=host .    
=======
<<<<<<< HEAD
docker image build -t ros2-panda .    
 

=======
docker image build -t kinova-ros2 .    
>>>>>>> .merge_file_dovT0W
 
>>>>>>> 32b90d2f783f4f6db2785f3dcba0953584cc9a6d
docker run -it \
	--env="DISPLAY" --device=/dev/video0:/dev/video0 \
	-e DISPLAY=$DISPLAY --env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--privileged \
	-e "ROS_DOMAIN_ID=7" \
	--net=host -v /dev/shm:/dev/shm \
	-u 0 \
	kinova-ros2 

export containerId=$(docker ps -l -q)


