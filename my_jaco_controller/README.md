# ros2_franka_docker

A simple Docker container for using the Franka Panda Emika arm with ROS2. Environment has been prepared accoridng to the installation setup described in [https://support.franka.de/docs/franka_ros2.html](https://support.franka.de/docs/franka_ros2.html)

The included `dockerscript.sh` includes the recommended settings for running the docker script

Also available in the docker store [https://hub.docker.com/repository/docker/enunezs/ros2_franka](https://hub.docker.com/repository/docker/enunezs/ros2_franka)

## Running the Dockerfile with the provided script

1. First, ensure that Docker is installed on your machine. If not, you can download and install Docker from [https://www.docker.com/get-started](https://www.docker.com/get-started).
   pose_goal.pose.orientation.x = (joy_msg.axes[2] \* MAX_POSITION_CHANGE)

2. Clone this repository to your local machine using the following command:

   ```bash
   git clone https://github.com/your-github-username/ros2_franka_docker.git
   ```

3. Navigate to the directory containing the Dockerfile and the `dockerscript.sh`:

   ```bash
   cd ros2_franka_docker
   ```

4. Make the `dockerscript.sh` executable:

   ```bash
   chmod +x dockerscript.sh
   ```

5. Run the `dockerscript.sh` script:

   ```bash
   ./dockerscript.sh
   ```

This script will build the Docker image and then run a Docker container from that image. The container will have the necessary settings for using the Franka Panda Emika arm with ROS2.

# Testing on robot

To simply connect to the robot, run the following command:

```bash
ros2 launch kinova_bringup kinova_robot_launch.py
```

Launch interactive control

```bash
ros2 run kinova_driver kinova_interactive_control j2n6s300
```

To launch with moveit and rviz, run the following command:

```bash
ros2 launch kinova_bringup moveit_robot_launch.py
```

Launch action servers

```bash
ros2 run kinova_driver joint_trajectory_action_server j2n6s300

ros2 run kinova_driver gripper_command_action_server j2n6s300
```
