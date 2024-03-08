# Messages for experimentation

ros2 topic pub --once /j2n6s300_driver/in/cartesian_velocity kinova_msgs/msg/PoseVelocity "{twist_linear_x: 0.1, twist_linear_y: 0, twist_linear_z: 0, twist_angular_x: 0, twist_angular_y: 0.1, twist_angular_z: 0}"

# Actions

After running

```
ros2 launch kinova_bringup kinova_robot_launch.py
```

We get three actions:

- /j2n6s300_driver/joint_angles # Direct joint control. Impractical
- /j2n6s300_driver/tool_pose # Seems like it is the way to go
- /j2n6s300_driver/finger_positions # Control fingers

## joint_angles -> joint action

ros2 action send_goal /j2n6s300_driver/joint_angles kinova_msgs/action/ArmJointAngles "{
angles: {joint1: 0.1,
joint2: 0,
joint3: 0,
joint4: 0,
joint5: 0,
joint6: 0,
joint7: 0
}}"

## tool_pose -> pose action client

ros2 action send_goal /j2n6s300_driver/tool_pose kinova_msgs/action/ArmPose "{ pose: ... }" Complex

geometry_msgs/PoseStamped pose

http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html

ros2 action send_goal /j2n6s300_driver/tool_pose kinova_msgs/action/ArmPose "{
pose: {
header: { stamp: { sec: 1694019287, nanosec: 9744447 } , frame_id: "j2n6s300_link_base" } ,
pose: {
position: { x: 0.10, y: -0.50, z: 0.35 },
orientation: { x: -1.0, y: 0.0, z: 0.0, w: 0.0 }
} } }"

## finger_positions -> finger action

ros2 action send_goal /j2n6s300_driver/finger_positions kinova_msgs/action/SetFingersPosition "{
fingers: {
finger1: 4000.0 ,
finger2: 4000.0 ,
finger3: 4000.0 }
}"

Where 6000 is closed and 0 is open

```
ros2 run kinova_driver joint_trajectory_action_server j2n6s300
```

New action

- /j2n6s300/follow_joint_trajectory # Difference?

## follow_joint_trajectory

ros2 action send_goal /j2n6s300/follow_joint_trajectory control_msgs/action/FollowJointTrajectory

http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html

```
ros2 run kinova_driver gripper_command_action_server j2n6s300
```

# Servers

## Send robot home

ros2 service call /j2n6s300_driver/in/home_arm kinova_msgs/srv/HomeArm {}\

## Add to cartesian trajectory

ros2 service call /j2n6s300_driver/in/add_pose_to_Cartesian_trajectory kinova_msgs/srv/AddPoseToCartesianTrajectory "{
x: 0.0,
y: 0.0,
z: 0.0,
theta_x: 0.0,
theta_y: 0.0,
theta_z: 0.0 }"

# Tests with in

## joint_velocity works

ros2 topic pub -r 100 /j2n6s300_driver/in/joint_velocity kinova_msgs/JointVelocity "{joint1: 0.0, joint2: 0.0, joint3: 0.0, joint4: 0.0, joint5: 0.0, joint6: 10.0}"

## cartesian_velocity works

ros2 topic pub -r 100 /j2n6s300_driver/in/cartesian_velocity kinova_msgs/msg/PoseVelocity "{
twist_linear_x: 0.1, twist_linear_y: 0.0, twist_linear_z: 0.0, twist_angular_x: 0.0, twist_angular_y: 0.0, twist_angular_z: 0.0
}"

---

## joint_torque - Does not work?

ros2 service call /j2n6s300_driver/in/set_torque_control_mode kinova_msgs/srv/SetTorqueControlMode "{ state: 1 }" &&
ros2 topic pub -r 100 /j2n6s300_driver/in/joint_torque kinova_msgs/msg/JointTorque "{
joint1: 1.0,
joint2: 1.0,
joint3: 1.0,
joint4: 1.0,
joint5: 1.0,
joint6: 1.0 }"

## cartesian_force - Does not work?

### Set torque control parameters

ros2 service call /j2n6s300_driver/in/set_torque_control_parameters kinova_msgs/srv/SetTorqueControlParameters {}

ros2 param set /j2n6s300_driver torque_parameters/publish_torque_with_gravity_compensation true
ros2 param set /j2n6s300_driver torque_parameters/torque_min "[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
ros2 param set /j2n6s300_driver torque_parameters/torque_max "[50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
ros2 param set /j2n6s300_driver torque_parameters/safety_factor 1.0
ros2 param set /j2n6s300_driver torque_parameters/com_parameters "[0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0]"

publish_torque_with_gravity_compensation

ros2 service call /j2n6s300_driver/in/set_torque_control_parameters kinova_msgs/srv/SetTorqueControlParameters "{}"

### Set torque control mode

ros2 service call /j2n6s300_driver/in/set_torque_control_mode kinova_msgs/srv/SetTorqueControlMode "{ state: 1 }"

### Call service

ros2 topic pub -r 100 /j2n6s300_driver/in/cartesian_force kinova_msgs/msg/CartesianForce "{
force_x: 1.0,
force_y: 1.0,
force_z: 1.0,
torque_x: 0.0,
torque_y: 0.0,
torque_z: 0.0
}"

## admitance

@warning You can only use this function if your robotic device has torque sensors on it.
Also, the robotic device must be in a standard vertical position.

ros2 service call /j2n6s300_driver/in/start_force_control kinova_msgs/srv/Start "{}"

ros2 service call /j2n6s300_driver/in/set_force_control_params kinova_msgs/srv/SetForceControlParams "{inertia_linear:
x: 1.25094 ,
y: 0.0438545 ,
z: -0.0735931 ,
inertia_angular:
x: -0.689186 ,
y: -0.0259276 ,
z: 0.197213 ,
damping_linear:
x: -0.0359891 ,
y: 0.107899 ,
z: -0.0670875 ,
damping_angular:
x: -0.0322568 ,
y: 1.65547 ,
z: -0.42493 ,
force_min_linear:
x: 0.853827 ,
y: -0.0336616 ,
z: 0.128325 ,
force_min_angular:
x: -0.124854 ,
y: 0.0 ,
z: 0.0 }"

\
\ \ x:\ 0.0\
\ \ y:\ 0.0\
\ \ z:\ 0.0\
inertia_angular:\
\ \ x:\ 0.0\
\ \ y:\ 0.0\
\ \ z:\ 0.0\
damping_linear:\
\ \ x:\ 0.0\
\ \ y:\ 0.0\
\ \ z:\ 0.0\
damping_angular:\
\ \ x:\ 0.0\
\ \ y:\ 0.0\
\ \ z:\ 0.0\
force_min_linear:\
\ \ x:\ 0.0\
\ \ y:\ 0.0\
\ \ z:\ 0.0\
force_min_angular:\
\ \ x:\ 0.0\
\ \ y:\ 0.0\
\ \ z:\ 0.0\
\ \ z:\ 0.0\ ^Car:\

[kinova_arm_driver-1] The parameters are:
% Inertia Linear
[kinova_arm_driver-1] Param[0] = 1.25094
[kinova_arm_driver-1] Param[1] = 0.0438545
[kinova_arm_driver-1] Param[2] = -0.0735931
% Inertia Angular
[kinova_arm_driver-1] Param[3] = -0.689186
[kinova_arm_driver-1] Param[4] = -0.0259276
[kinova_arm_driver-1] Param[5] = 0.197213
% Damping Linear
[kinova_arm_driver-1] Param[6] = -0.0359891
[kinova_arm_driver-1] Param[7] = 0.107899
[kinova_arm_driver-1] Param[8] = -0.0670875
% Damping Angular
[kinova_arm_driver-1] Param[9] = -0.0322568
[kinova_arm_driver-1] Param[10] = 1.65547
[kinova_arm_driver-1] Param[11] = -0.42493
% Force min linear
[kinova_arm_driver-1] Param[12] = 0.853827
[kinova_arm_driver-1] Param[13] = -0.0336616
[kinova_arm_driver-1] Param[14] = 0.128325
% Force min angular
[kinova_arm_driver-1] Param[15] = -0.124854

## cartesian_velocity_with_finger_velocity

## cartesian_velocity_with_fingers

# Calibrate

ros2 action send_goal /j2n6s300_driver/joint_angles kinova_msgs/action/ArmJointAngles "{ angles: {
joint1: 180.0,
joint2: 180.0,
joint3: 180.0,
joint4: 180.0,
joint5: 180.0,
joint6: 180.0,
joint7: 180.0}}"

ros2 service call /j2n6s300_driver/in/set_zero_torques kinova_msgs/srv/ZeroTorques {}

---

# robot params

ros\_\_parameters:
connection_type: USB
convert_joint_velocities: true
jointSpeedLimitParameter1: 10
jointSpeedLimitParameter2: 10
kinova_robotName: left
kinova_robotType: j2n6s300
local_broadcast_port: 25025
local_cmd_port: 25000
local_machine_IP: 192.168.100.100
payload: []
qos_overrides:
/parameter_events:
publisher:
depth: 1000
durability: volatile
history: keep_last
reliability: reliable
robot_name: j2n6s300
robot_type: j2n6s300
serial_number: ''
status_interval_seconds: 0.1
subnet_mask: 255.255.255.0
torque_parameters/com_parameters: []
torque_parameters/publish_torque_with_gravity_compensation: false
torque_parameters/safety_factor: 1.0
torque_parameters/torque_max: []
torque_parameters/torque_min: []
torque_parameters/use_estimated_COM_parameters: true
use_jaco_v1_fingers: true
use_sim_time: false
