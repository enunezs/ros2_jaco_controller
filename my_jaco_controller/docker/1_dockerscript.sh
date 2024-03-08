#!/bin/bash

# This command allows the root user on the local machine to connect to the X server. 
# This will allow GUI applications running in Docker to display on the host.
xhost +local:root

# This command builds a Docker image named kinova-ros2 using the Dockerfile in the current directory. 
# It sets the build argument DISPLAY to :0 (the default display on the host), 
# and uses the host's network stack.

docker image build -t kinova-ros2 \
	--build-arg DISPLAY=:0 \
	--network=host .     

# This command runs a container from the kinova-ros2 image. 
# It sets several environment variables and options to allow the container to display GUI applications on the host and access the host's video device. It also mounts the X11 Unix socket from the host into the container to allow communication with the X server. 
# The --privileged option gives the container full access to the host's devices, and the -u 0 option runs the container as the root user.

docker run -it \
	--env="DISPLAY" --device=/dev/video0:/dev/video0 \
	-e DISPLAY=$DISPLAY --env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--privileged \
	-e "ROS_DOMAIN_ID=7" \
	--net=host -v /dev/shm:/dev/shm \
	-v $(pwd)/:/root/ws/my_controllers \
	-u 0 \
	kinova-ros2 

# This command gets the ID of the last created container and stores it in the containerId variable. 
# The -l option to docker ps means "last" and the -q option means "quiet", which causes the command to only output the container ID.

export containerId=$(docker ps -l -q)


