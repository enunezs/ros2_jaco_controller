#!/usr/bin/env python3
"""
Refactored Jaco Robot Controller

Receives button commands and manages:
- Continuous velocity control (Twist messages)
- Discrete rotation commands
- Frame-independent position/rotation
- PID-controlled motion
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import yaml
import numpy as np
from dataclasses import dataclass, field
from enum import Enum
import traceback

from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import String, Header
from kinova_msgs.msg import PoseVelocityWithFingerVelocity, FingerPosition
from sensor_msgs.msg import Joy

from tf2_ros import Buffer, TransformListener, TransformBroadcaster, LookupException
from scipy.spatial.transform import Rotation
import rclpy.time


class PIDController:
    """Simple PID controller"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, refresh_rate=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.refresh_rate = refresh_rate
        self.prev_error = 0.0
        self.integral_error = 0.0
    
    def update(self, target, current, dt=None):
        """Returns control output"""
        if dt is None:
            dt = 1.0 / self.refresh_rate
        
        error = target - current
        self.integral_error += error * dt
        self.integral_error = np.clip(self.integral_error, -1.0, 1.0)
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        
        return self.kp * error + self.ki * self.integral_error + self.kd * derivative


@dataclass
class ControlState:
    """Central robot control state"""
    position_frame: str = "j2n6s300_link_base"
    rotation_frame: str = "j2n6s300_link_base"
    
    # Velocity targets
    vel_linear: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel_angular: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Rotation accumulator (for discrete steps)
    accumulated_rotation_deg: np.ndarray = field(default_factory=lambda: np.zeros(3))
    pending_rotation_update: bool = False
    
    # Current state
    current_pose: PoseStamped = None
    current_vel: np.ndarray = field(default_factory=lambda: np.zeros(6))
    
    # Gripper
    finger_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))


class JacoGazeController(Node):
    def __init__(self):
        super().__init__('jaco_gaze_controller')
        
        # Load configuration
        config_file = self.declare_parameter('config_file', 
                                            '/path/to/button_config.yaml').value
        self.config = self._load_config(config_file)
        
        # State
        self.state = ControlState()
        
        # Callback groups (prevent blocking between input and output)
        # TODO: Learn about ReentrantCallbackGroup 
        input_group = MutuallyExclusiveCallbackGroup()
        output_group = MutuallyExclusiveCallbackGroup()
        
        # Subscribers
        self.button_sub = self.create_subscription(
            String,
            '/robot/button_command',
            self.button_callback,
            10,
            callback_group=input_group
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/j2n6s300_driver/out/tool_pose',
            self.pose_callback,
            10,
            callback_group=input_group
        )
        
        # Publishers
        self.twist_pub = self.create_publisher(
            PoseVelocityWithFingerVelocity,
            '/j2n6s300_driver/in/cartesian_velocity_with_finger_velocity',
            1,
            callback_group=output_group
        )
        
        self.pose_goal_pub = self.create_publisher(
            PoseStamped,
            '/robot/goal_pose',
            1,
            callback_group=output_group
        )
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # PID controllers
        pid_cfg = self.config.get('pid', {})
        linear_cfg = pid_cfg.get('linear', {})
        angular_cfg = pid_cfg.get('angular', {})
        
        self.pid_linear = PIDController(
            kp=linear_cfg.get('kp', 0.7),
            ki=linear_cfg.get('ki', 0.1),
            kd=linear_cfg.get('kd', 0.0),
            refresh_rate=100.0
        )
        
        self.pid_angular = PIDController(
            kp=angular_cfg.get('kp', 2.0),
            ki=angular_cfg.get('ki', 2.0),
            kd=angular_cfg.get('kd', 5.0),
            refresh_rate=100.0
        )
        
        # Load config values
        vel_cfg = self.config.get('velocity', {})
        self.max_linear_vel = np.array(vel_cfg.get('max_linear', [0.1, 0.06, 0.08]))
        self.max_angular_vel = vel_cfg.get('max_angular', 2.0)
        self.max_finger_vel = vel_cfg.get('max_finger', 2000.0)
        
        rot_cfg = self.config.get('rotation', {})
        self.discretise_rotation = rot_cfg.get('discretise', True)
        self.quantisation_deg = rot_cfg.get('quantisation_degrees', 30.0)
        self.max_angle_diff = rot_cfg.get('max_angle_difference', 0.785)
        
        # Previous button tracking
        self.prev_button_state = {}
        
        # Publish timer
        self.publish_timer = self.create_timer(
            1.0 / 100.0,
            self.publish_commands,
            callback_group=output_group
        )
        
        self.get_logger().info("Jaco Gaze Controller initialized")
    
    def _load_config(self, path: str) -> dict:
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            return {}
    
    def button_callback(self, msg: String):
        """Handle button commands from parser"""
        try:
            parts = msg.data.split(':')
            if len(parts) != 2:
                return
            
            button_name, action = parts
            button_cfg = self.config.get('button_mappings', {}).get(button_name)
            
            if button_cfg is None:
                self.get_logger().warn(f"Unknown button: {button_name}")
                return
            
            if action == "press":
                self._execute_button_press(button_name, button_cfg)
            elif action == "release":
                self._execute_button_release(button_name, button_cfg)
        
        except Exception as e:
            self.get_logger().error(f"Error in button_callback: {e}")
            traceback.print_exc()
    
    def _execute_button_press(self, button_name: str, button_cfg: dict):
        """Execute on button press"""
        mode = button_cfg.get('mode')
        action = button_cfg.get('action')
        
        if mode == 'velocity_xyz':
            self._handle_velocity_press(button_cfg)
        elif mode == 'velocity_rot':
            self._handle_rotation_press(button_cfg)
        elif mode == 'gripper':
            self._handle_gripper_press(button_cfg)
        elif mode == 'system':
            self._handle_system_command(button_cfg)
    
    def _execute_button_release(self, button_name: str, button_cfg: dict):
        """Execute on button release"""
        mode = button_cfg.get('mode')
        
        # Stop velocity on release
        if mode == 'velocity_xyz':
            axis = button_cfg.get('axis')
            axis_idx = {'x': 0, 'y': 1, 'z': 2}.get(axis)
            if axis_idx is not None:
                self.state.vel_linear[axis_idx] = 0.0
    
    def _handle_velocity_press(self, cfg: dict):
        """Handle continuous velocity command"""
        axis = cfg.get('axis')
        direction = 1 if cfg.get('direction') == 'positive' else -1
        magnitude = cfg.get('magnitude', 0.1)
        
        axis_idx = {'x': 0, 'y': 1, 'z': 2}.get(axis)
        if axis_idx is not None:
            self.state.vel_linear[axis_idx] = direction * magnitude
    
    def _handle_rotation_press(self, cfg: dict):
        """Handle discrete rotation command"""
        axis = cfg.get('axis')
        delta_deg = cfg.get('delta_degrees', 30)
        
        axis_idx = {'x': 0, 'y': 1, 'z': 2}.get(axis)
        if axis_idx is not None:
            self.state.accumulated_rotation_deg[axis_idx] += delta_deg
            self.state.pending_rotation_update = True
            self.get_logger().info(
                f"Queued rotation: {self.state.accumulated_rotation_deg}"
            )
    
    def _handle_gripper_press(self, cfg: dict):
        """Handle gripper command"""
        direction = cfg.get('direction')
        velocity = cfg.get('velocity', self.max_finger_vel)
        
        if direction == 'open':
            self.state.finger_velocity = np.array([1, 1, 1]) * velocity
        elif direction == 'close':
            self.state.finger_velocity = np.array([-1, -1, -1]) * velocity
    
    def _handle_system_command(self, cfg: dict):
        """Handle system commands"""
        action = cfg.get('action')
        
        if action == 'set_position_frame':
            self.state.position_frame = cfg.get('frame')
            self.get_logger().info(f"Position frame: {self.state.position_frame}")
        
        elif action == 'set_rotation_frame':
            self.state.rotation_frame = cfg.get('frame')
            self.get_logger().info(f"Rotation frame: {self.state.rotation_frame}")
        
        elif action == 'home_robot':
            self.get_logger().info("Homing robot...")
            # TODO: Implement home
        
        elif action == 'reset_state':
            self.state = ControlState()
            self.get_logger().info("State reset")
        
        elif action == 'recalibrate_gaze':
            self.get_logger().info("Recalibrating gaze...")
            # TODO: Implement recalibration
    
    def publish_commands(self):
        """Continuously publish velocity commands"""
        try:
            # Transform velocities if needed
            vel_lin_transformed = self._transform_velocity(
                self.state.vel_linear,
                source=self.state.position_frame,
                target='j2n6s300_link_base'
            )
            
            vel_ang_transformed = self._transform_velocity(
                self.state.vel_angular,
                source=self.state.rotation_frame,
                target='j2n6s300_link_base'
            )
            
            # Create message
            msg = PoseVelocityWithFingerVelocity()
            
            msg.twist_linear_x = vel_lin_transformed[0]
            msg.twist_linear_y = vel_lin_transformed[1]
            msg.twist_linear_z = vel_lin_transformed[2]
            
            msg.twist_angular_x = vel_ang_transformed[0]
            msg.twist_angular_y = vel_ang_transformed[1]
            msg.twist_angular_z = vel_ang_transformed[2]
            
            msg.finger1 = self.state.finger_velocity[0]
            msg.finger2 = self.state.finger_velocity[1]
            msg.finger3 = self.state.finger_velocity[2]
            
            self.twist_pub.publish(msg)
            
            # Handle discrete rotations (send once when updated)
            if self.state.pending_rotation_update:
                self._publish_rotation_goal()
                self.state.pending_rotation_update = False
        
        except Exception as e:
            self.get_logger().error(f"Error publishing: {e}")
    
    def _transform_velocity(self, vel: np.ndarray, source: str, 
                           target: str) -> np.ndarray:
        """Transform velocity between frames"""
        if source == target:
            return vel
        
        try:
            tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            rot = Rotation.from_quat([
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w
            ])
            return rot.apply(vel)
        except Exception:
            return vel
    
    def _publish_rotation_goal(self):
        """Publish accumulated rotation as a pose goal"""
        # Convert accumulated rotation to pose
        rot_euler = self.state.accumulated_rotation_deg * np.pi / 180.0
        rot = Rotation.from_euler('xyz', rot_euler)
        
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.state.rotation_frame
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position (keep current)
        if self.state.current_pose:
            pose_msg.pose.position = self.state.current_pose.pose.position
        
        # Orientation
        quat = rot.as_quat()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.pose_goal_pub.publish(pose_msg)
        self.get_logger().info(f"Rotation goal published: {self.state.accumulated_rotation_deg}°")
    
    def pose_callback(self, msg: PoseStamped):
        """Update current pose"""
        self.state.current_pose = msg


def main():
    rclpy.init()
    controller = JacoGazeController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()