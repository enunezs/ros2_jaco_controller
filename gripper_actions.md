

# Set to open
ros2 action send_goal /panda_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.027, max_effort: 0.1}}"

# Set to closed
ros2 action send_goal /panda_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.012, max_effort: 0.1}}"

# Get info 
ros2 action info /panda_gripper/gripper_action
