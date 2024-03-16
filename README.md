# ros2_jaco_controller

Environment and scripts to connect to a jaco arm and run controllers on ROS2.
Allows for easy setup and running of the jaco arm in a docker container.
Ideal for real-time control of the jaco arm.

The included `01_dockerscript.sh` includes the recommended settings for running the docker script

The associated Docker Image can be found [at the following link](https://hub.docker.com/repository/docker/enunezs/kinova-ros2/general)

---

# Installation

0. Install Docker

1. Clone this repository to your local machine workspace with:

```bash
git clone https://github.com/enunezs/ros2_jaco_docker.git
```

2. Navigate to the directory containing the Dockerfile and the `dockerscript.sh`:

   ```bash
   cd ros2_franka_docker
   ```

3. If needed, make the `dockerscript.sh` executable:

   ```bash
   chmod +x docker/01_dockerscript.sh
   ```

---

# Running

To launch the container with the environment, run the following ready script:

```bash
./docker/1_dockerscript.sh
```

This will pull the latest image from the docker store and run the container with the recommended settings.

## Main container

To launch the controller for the Jaco arm, simply run the following command:

```bash
ros2 launch ros2_jaco_controller jaco_and_controller.launch.py
```

## Running a controller

To run a controller, simply run the following command:

```bash
ros2 run ros2_jaco_controller all_purpose_controller.py
```

---

## Running nodes independently

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

---
