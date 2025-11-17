#!/usr/bin/env python3
"""
Refactored Jaco Robot Controller with Mode-Based Behaviors

Architecture:
- Shared utility components (PID, filters, integrators)
- Mode-specific behavior classes
- Clean separation of concerns
- Easy to extend and test
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from kinova_msgs.msg import PoseVelocity, PoseVelocityWithFingerVelocity, FingerPosition
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped, Point
from std_msgs.msg import Int32 as int_msg
from std_msgs.msg import String as str_msg
from geometry_msgs.msg import TwistStamped

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

from scipy.spatial.transform import Rotation
import numpy as np
import traceback
from abc import ABC, abstractmethod
from collections import deque
from typing import Optional, Tuple, Dict

# ============================================================================
# CONSTANTS AND CONFIGURATION
# ============================================================================

REFRESH_RATE = 100.0
START_ROTATION = Rotation.from_euler('xyz', [-180-20, 0-5, 180-10], degrees=True)
MAX_LINEAR_VELOCITY = (0.1, 0.06, 0.08)
MAX_ANGULAR_VELOCITY = 2.0
MAX_FINGER_VELOCITY = 2000.0

# TODO
ROTATION_ASSISTANCE = 2.0   

# ============================================================================
# SHARED UTILITY COMPONENTS
# ============================================================================

class PIDController:
    """
    Generic PID controller for velocity tracking.
    Supports anti-windup and multi-dimensional control.
    """
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0, 
                 refresh_rate: float = 1.0, length: int = 3, anti_windup: bool = True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.refresh_rate = refresh_rate
        self.length = length
        self.prev_error = np.zeros(self.length)
        self.integral_error = np.zeros(self.length)
        self.derivative_error = np.zeros(self.length)
        self.anti_windup = anti_windup

    def update_parameters(self, kp: Optional[float] = None, ki: Optional[float] = None, 
                         kd: Optional[float] = None, refresh_rate: Optional[float] = None):
        """Update PID parameters at runtime."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if refresh_rate is not None:
            self.refresh_rate = refresh_rate

    def control_update(self, target_vel: np.ndarray, current_vel: np.ndarray) -> np.ndarray:
        """
        Compute PID control output.
        
        Args:
            target_vel: Desired velocity
            current_vel: Current measured velocity
            
        Returns:
            Control adjustment to add to target velocity
        """
        if target_vel is None or current_vel is None:
            return np.zeros(self.length)

        current_error = target_vel - current_vel
        # print("Current error:", current_error)

        # Integral term with anti-windup
        self.integral_error += current_error * (1.0 / self.refresh_rate)
        if self.anti_windup:
            self.integral_error = np.clip(self.integral_error, -1.0, 1.0)
        
        # Derivative term
        self.derivative_error = (current_error - self.prev_error) * self.refresh_rate
        self.prev_error = current_error

        compensation = self.kp * current_error + self.ki * self.integral_error + self.kd * self.derivative_error
        # print("PID compensation:", compensation)
        # print("Current velocity:", current_vel)
        return compensation

    def reset(self):
        """Reset PID state (useful when switching modes)."""
        self.prev_error = np.zeros(self.length)
        self.integral_error = np.zeros(self.length)
        self.derivative_error = np.zeros(self.length)

class RollingAverageFilter:
    """Simple rolling average filter for sensor smoothing."""
    
    def __init__(self, window_size: int):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
        self.sum = 0.0

    def update(self, value: float) -> float:
        """Add new value and return filtered output."""
        if len(self.values) == self.window_size:
            self.sum -= self.values[0]
        self.values.append(value)
        self.sum += value
        return self.sum / len(self.values)

    def reset(self):
        """Clear filter state."""
        self.values.clear()
        self.sum = 0.0

class VelocityIntegrator:
    """
    Handles smooth acceleration and deceleration for velocity commands.
    Provides natural feeling control with configurable dynamics.
    """
    
    def __init__(self, max_velocity: Tuple[float, float, float], 
                 accel_time: float = 1.5, brake_time: float = 0.15, 
                 refresh_rate: float = 100.0):
        self.max_velocity = np.array(max_velocity)
        self.prev_velocity = np.zeros(3)
        # TODO: Expose! Super important variable hiding here
        self.forward_acceleration = np.array([0.015, 0.009, 0.012]) 
        self.brake_acceleration = 0.8
        self.min_speed = 0.027
        self.refresh_rate = refresh_rate
        self.deadzone = 0.2 # Input deadzone (0 to 1)


    def update(self, input_vector: np.ndarray, dt: float) -> np.ndarray:
        """
        Apply smooth acceleration/deceleration to input.
        
        Args:
            input_vector: Raw input [-1, 1] range
            dt: Time delta (1/refresh_rate)
            
        Returns:
            Smoothed velocity vector
        """
        velocity = self.prev_velocity.copy()
        hard_brake_threshold = 0.02 # m/s Speed threshold for hard braking
        
        for idx, input_dir in enumerate(input_vector):
            is_hard_brake = ((input_dir * velocity[idx]) < -hard_brake_threshold)
            is_deadzone = abs(input_dir) < self.deadzone
            movement_dir = 1 if velocity[idx] > 0 else -1

            # Deceleration
            if is_deadzone or is_hard_brake:
                velocity[idx] -= movement_dir * self.brake_acceleration * dt
                # self.get_logger().debug(f"Braking on axis {idx}: new velocity {velocity[idx]}")
                
                if abs(velocity[idx]) < self.min_speed * 1.5:
                    velocity[idx] = 0.0
            
            # Acceleration
            else:
                velocity[idx] += input_dir * self.forward_acceleration[idx] * dt
                
                # Fast start from rest
                if abs(velocity[idx]) < self.min_speed:
                    velocity[idx] = self.min_speed * np.sign(input_dir)
            
            # Clamp to max velocity
            velocity[idx] = np.clip(velocity[idx], 
                                   -self.max_velocity[idx], 
                                   self.max_velocity[idx])
        
        self.prev_velocity = velocity
        return velocity

    def reset(self):
        """Reset to zero velocity."""
        self.prev_velocity = np.zeros(3)

class RotationController:
    """
    Manages rotation control with quantization, tracking, and velocity computation.
    """
    
    def __init__(self, start_rotation: Rotation, quantization_degrees: float = 30.0,
                 max_angular_velocity: float = 2.0, refresh_rate: float = 100.0):
        self.start_rotation = start_rotation
        self.quantization_degrees = quantization_degrees
        self.max_angular_velocity = max_angular_velocity
        self.refresh_rate = refresh_rate
        self.cumulative_rotation = np.zeros(3)
        self.discretize_enabled = True

    def update_from_input(self, orientation_input: np.ndarray, dt: float) -> Rotation:
        """
        Bi-stable rotation update:
        - When user is moving -> follow velocity input
        - When user stops -> settle to nearest quantized zone
        """
        
        orientation_change = orientation_input * 10.0
        # ---------------------------------------
        # 1. Clamp user angular input
        # ---------------------------------------
        MAX_ROTATION_CHANGE = 60.0
        orientation_change = np.clip(
            orientation_change,
            -MAX_ROTATION_CHANGE,
            MAX_ROTATION_CHANGE
        )

        # ---------------------------------------
        # 2. Check whether the user is actually moving
        # ---------------------------------------
        input_deadband = 0.01  # small threshold to detect "no movement"
        user_active = np.linalg.norm(orientation_change) > input_deadband

        # ---------------------------------------
        # 3. Integrate continuous motion when user is active
        # ---------------------------------------
        if user_active:
            self.cumulative_rotation += orientation_change * dt

        # ---------------------------------------
        # 4. Compute nearest quantized target (wrap-safe)
        # ---------------------------------------
        q = self.quantization_degrees
        quantized_target = np.round(self.cumulative_rotation / q) * q

        # print in degrees
        # print("Quantized target rotation (deg):", np.degrees(quantized_target))

        # ---------------------------------------
        # 5. If user is inactive → smoothly settle to quantized zone
        # ---------------------------------------
        if not user_active:
            # settling gain per second
            settle_gain = 6.0      # higher → faster snapping (critically damped feel)
            alpha = 1 - np.exp(-settle_gain * dt)
            
            # Smoothly blend toward quantized value
            self.cumulative_rotation = (
                (1 - alpha) * self.cumulative_rotation +
                alpha * quantized_target
            )

        # ---------------------------------------
        # 6. Convert to rotation
        # ---------------------------------------
        rot_controller = Rotation.from_euler('xyz', self.cumulative_rotation, degrees=True)


        return self.start_rotation * rot_controller


    def old_update_from_input(self, orientation_change: np.ndarray, dt: float) -> Rotation:
        """
        Update rotation target from input.
        
        Args:
            orientation_change: Angular velocity input from controller
            dt: Time delta
            
        Returns:
            Target rotation as Rotation object
        """

        # print("Orientation change input:", orientation_change)

        # Clamp rotation rate
        MAX_ROTATION_CHANGE = 60.0
        orientation_change = np.clip(orientation_change, 
                                    -MAX_ROTATION_CHANGE, 
                                    MAX_ROTATION_CHANGE)
        
        # Accumulate rotation
        self.cumulative_rotation += orientation_change * dt
        
        # Discretize if enabled
        target_rotation = self.cumulative_rotation.copy()
        if self.discretize_enabled:
            target_rotation = np.round(
                target_rotation / self.quantization_degrees
            ) * self.quantization_degrees
        
        # Create rotation from euler angles
        rot_controller = Rotation.from_euler('xyz', target_rotation, degrees=True)
        return self.start_rotation * rot_controller

    def compute_velocity(self, target_rotation: Rotation, 
                        current_rotation: Rotation, 
                        pid_controller: Optional[PIDController] = None) -> np.ndarray:
        """
        Compute angular velocity to reach target rotation.
        
        Args:
            target_rotation: Desired rotation
            current_rotation: Current end-effector rotation
            pid_controller: Optional PID controller for angular velocity
            
        Returns:
            Angular velocity vector [rx, ry, rz]
        """
        # Compute rotation difference
        rotation_diff = current_rotation.inv() * target_rotation
        
        # Get magnitude of rotation
        mag = 2 * np.arccos(np.clip(rotation_diff.as_quat()[3], -1.0, 1.0))
        
        # Clamp excessive rotations
        MAX_ANGLE_DIFFERENCE = np.pi / 4
        if mag > MAX_ANGLE_DIFFERENCE:
            # Could implement slerp here if needed
            pass
        
        # Convert to euler angles
        euler_error = rotation_diff.as_euler('xyz', degrees=False)
        
        # Apply PID or proportional control
        if pid_controller is not None:
            angular_velocity = pid_controller.control_update(euler_error, np.zeros(3))
        else:
            angular_velocity = euler_error * self.max_angular_velocity
        
        return angular_velocity

    def reset(self):
        """Reset cumulative rotation."""
        self.cumulative_rotation = np.zeros(3)


# ============================================================================
# BEHAVIOR CLASSES
# ============================================================================

class ControlBehavior(ABC):
    """Abstract base class for control mode behaviors."""
    
    def __init__(self, controller: 'RobotController'):
        self.controller = controller
    
    @abstractmethod
    def process_velocity_command(self, twist: TwistStamped) -> Optional[PoseVelocityWithFingerVelocity]:
        """Process continuous velocity command."""
        pass
    
    @abstractmethod
    def process_discrete_command(self, pose: PoseStamped) -> bool:
        """Process discrete pose command. Returns True if handled."""
        pass
    
    @abstractmethod
    def on_enter(self):
        """Called when entering this mode."""
        pass
    
    @abstractmethod
    def on_exit(self):
        """Called when exiting this mode."""
        pass

class ContinuousTeleopBehavior(ControlBehavior):
    """
    Continuous teleoperation behavior.
    Processes velocity commands with PID control and smooth integration.
    """
    
    def __init__(self, controller: 'RobotController', reference_frame: str = "j2n6s300_link_base"):
        super().__init__(controller)
        self.reference_frame = reference_frame
    
    def process_velocity_command(self, twist: TwistStamped) -> Optional[PoseVelocityWithFingerVelocity]:
        """
        Process continuous velocity command with PID and smoothing.
        """
        if not self.controller.current_pose:
            return None
        
        ### Message Extraction ###
        # Extract linear velocity
        target_linear_vel = np.array([
            twist.twist.linear.x,
            twist.twist.linear.y,
            twist.twist.linear.z
        ])
        
        # Extract angular velocity
        target_angular_vel = np.array([
            twist.twist.angular.x,
            twist.twist.angular.y,
            twist.twist.angular.z
        ])
        
        ### Cartesian Velocity ###
        # Transform velocity to appropriate frame if needed
        if twist.header.frame_id != self.reference_frame:
            target_linear_vel = self.controller.transform_velocity(
                target_linear_vel, 
                twist.header.frame_id, 
                self.reference_frame
            )

        # Apply velocity integrator for smooth acceleration
        target_linear_vel = self.controller.vel_integrator.update(
            target_linear_vel, 
            dt=1.0 / 100.0
        )

        # Apply PID for linear velocity
        if self.controller.current_vel is not None:
            pid_correction = self.controller.pid_linear.control_update(
                target_linear_vel, 
                self.controller.current_vel[0:3]
            )
            # TODO: Fixed, but likely needs retuning
            # target_linear_vel = pid_correction
        
        ### ! Angular Position Control ###
        # print("=== Angular Control ===")
        # print("Target angular vel input:", target_angular_vel)
        # Find the current end-effector rotation
        current_ee_rotation = self.controller.get_ee_rotation()
        # print("Current EE rotation:", current_ee_rotation.as_euler('xyz', degrees=True) if current_ee_rotation else "None")

        ### ! Problems here
        # Find the target rotation pose
        # TODO: AHA Discretize here
        self.rotation_target_pose = self.controller.rotation_controller.update_from_input(
            # self.controller.current_pose, 
            target_angular_vel,
            dt=1.0 / 100.0
        )
        # print("Rotation target pose:", self.rotation_target_pose.as_euler('xyz', degrees=True))

        # Use the current and target rotations to compute a target angular velocity
        if current_ee_rotation is not None:
            target_angular_vel = self.controller.rotation_controller.compute_velocity(
                self.rotation_target_pose, 
                current_ee_rotation, 
                pid_controller= None
                # pid_controller=self.controller.pid_angular if self.controller.pid_enabled else None
            )
        else:
            target_angular_vel = np.zeros(3)
        # print("Computed target angular vel:", target_angular_vel)

        # Apply PID control for angular velocity (if angular command present)
        # if np.linalg.norm(target_angular_vel) > 0.01:
            # For continuous angular control, we can directly use the target
            # Or apply PID if we have angular velocity feedback
            # pid_angular_correction = self.controller.pid_angular.control_update(
            #     target_angular_vel, 
            #     self.controller.current_vel[3:6]
            # )
        # print("PID angular correction:", pid_angular_correction)
        
        # Pack into message
        msg = PoseVelocityWithFingerVelocity()
        msg.twist_linear_x = float(target_linear_vel[0])
        msg.twist_linear_y = float(target_linear_vel[1])
        msg.twist_linear_z = float(target_linear_vel[2])
        msg.twist_angular_x = float(target_angular_vel[0])
        msg.twist_angular_y = float(target_angular_vel[1])
        msg.twist_angular_z = float(target_angular_vel[2])
        
        # Add finger velocities (currently zero, can be extended)
        msg.finger1 = 0.0
        msg.finger2 = 0.0
        msg.finger3 = 0.0
        
        return msg
    
    def process_discrete_command(self, pose: PoseStamped) -> bool:
        """Continuous mode ignores discrete commands."""
        self.controller.get_logger().warn(
            "Discrete command received in continuous mode - ignoring"
        )
        return False
    
    def on_enter(self):
        """Reset integrators when entering continuous mode."""
        self.controller.get_logger().info(
            f"Entered continuous teleop mode (frame: {self.reference_frame})"
        )
        self.controller.pid_linear.reset()
        self.controller.pid_angular.reset()
    
    def on_exit(self):
        """Send zero velocity when exiting."""
        self.controller.get_logger().info("Exiting continuous teleop mode")
        self.controller.publish_zero_velocity()

# ============================================================================
# EXTENDED BEHAVIOR IMPLEMENTATIONS (Optional enhancements)
# ============================================================================

class RotationContinuousBehavior(ContinuousTeleopBehavior):
    """
    Specialized behavior for rotation mode with orientation control.
    Extends continuous teleop with rotation-specific features.
    """
    
    def __init__(self, controller: 'RobotController'):
        super().__init__(controller, reference_frame="j2n6s300_link_base")
        self.get_logger().info("Initialized RotationContinuousBehavior")
    
    def process_velocity_command(self, twist: TwistStamped) -> Optional[PoseVelocityWithFingerVelocity]:
        """
        Process velocity with focus on rotation control.
        """
        if not self.controller.current_pose:
            return None
        
        # Get base processing
        msg = super().process_velocity_command(twist)
        
        if msg is None:
            return None
        
        # Add rotation-specific processing
        current_rotation = self.controller.get_ee_rotation()
        if current_rotation is not None:
            # Could add rotation tracking, constraints, etc.
            pass
        
        return msg


class HybridTeleopBehavior(ControlBehavior):
    """
    Hybrid behavior that combines continuous and discrete control.
    Useful for advanced control schemes.
    """
    
    def __init__(self, controller: 'RobotController'):
        super().__init__(controller)
        self.continuous_behavior = ContinuousTeleopBehavior(controller)
        self.discrete_behavior = DiscreteTeleopBehavior(controller)
    
    def process_velocity_command(self, twist: TwistStamped) -> Optional[PoseVelocityWithFingerVelocity]:
        """Allow continuous control when no discrete action is executing."""
        if self.discrete_behavior.executing_action:
            return None
        return self.continuous_behavior.process_velocity_command(twist)
    
    def process_discrete_command(self, pose: PoseStamped) -> bool:
        """Execute discrete commands."""
        return self.discrete_behavior.process_discrete_command(pose)
    
    def on_enter(self):
        self.controller.get_logger().info("Entered hybrid teleop mode")
        self.continuous_behavior.on_enter()
    
    def on_exit(self):
        self.controller.get_logger().info("Exiting hybrid teleop mode")
        self.continuous_behavior.on_exit()
        if self.discrete_behavior.executing_action:
            self.discrete_behavior.on_exit()



class DiscreteTeleopBehavior(ControlBehavior):
    """
    Discrete teleoperation behavior.
    Executes discrete pose/rotation commands one at a time.
    """
    
    def __init__(self, controller: 'RobotController'):
        super().__init__(controller)
        self.executing_action = False
        self.target_pose = None
    
    def process_velocity_command(self, twist: TwistStamped) -> Optional[PoseVelocityWithFingerVelocity]:
        """
        In discrete mode, continuous commands are ignored or could be used for preview.
        """
        return None
    
    def process_discrete_command(self, pose: PoseStamped) -> bool:
        """
        Execute discrete movement command.
        
        Args:
            pose: Target pose (relative transform)
            
        Returns:
            True if command was accepted
        """
        if self.executing_action:
            self.controller.get_logger().warn(
                "Discrete action already in progress - ignoring new command"
            )
            return False
        
        self.controller.get_logger().info(
            f"Executing discrete command: "
            f"pos=[{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}] "
            f"frame={pose.header.frame_id}"
        )
        
        self.target_pose = pose
        self.executing_action = True
        
        # TODO: Implement actual discrete movement execution
        # Options:
        # 1. Use action client to send pose goal
        # 2. Generate trajectory and execute
        # 3. Use velocity control with goal checking
        
        # For now, simulate with a simple approach
        self._execute_discrete_movement()
        
        return True
    
    def _execute_discrete_movement(self):
        """
        Execute the discrete movement.
        This is a simplified version - in practice you'd use action clients or trajectories.
        """
        # Check if rotation or translation
        quat = self.target_pose.pose.orientation
        is_rotation = not (quat.w == 1.0 and quat.x == 0.0 and quat.y == 0.0 and quat.z == 0.0)
        
        if is_rotation:
            self.controller.get_logger().info("Executing discrete rotation")
            # Apply rotation relative to current pose
            current_rot = self.controller.get_ee_rotation()
            if current_rot is not None:
                target_rot_delta = Rotation.from_quat([
                    quat.x, quat.y, quat.z, quat.w
                ])
                # Compute and execute rotation
                # In real implementation, this would be an action or trajectory
        else:
            self.controller.get_logger().info("Executing discrete translation")
            # Apply translation relative to current pose
            # In real implementation, this would be an action or trajectory
        
        # Mark as complete (in real implementation, this would be done by action callback)
        self.executing_action = False
    
    def on_enter(self):
        """Called when entering discrete mode."""
        self.controller.get_logger().info("Entered discrete teleop mode")
        self.executing_action = False
        self.target_pose = None
    
    def on_exit(self):
        """Cancel any in-progress actions when exiting."""
        if self.executing_action:
            self.controller.get_logger().warn("Canceling in-progress discrete action")
            self.executing_action = False
            self.target_pose = None


class SystemBehavior(ControlBehavior):
    """
    System control behavior.
    Handles system-level commands like reset, emergency stop, etc.
    """
    
    def __init__(self, controller: 'RobotController'):
        super().__init__(controller)
    
    def process_velocity_command(self, twist: TwistStamped) -> Optional[PoseVelocityWithFingerVelocity]:
        """System mode doesn't process velocity commands."""
        return None
    
    def process_discrete_command(self, pose: PoseStamped) -> bool:
        """System mode doesn't process discrete pose commands."""
        return False
    
    def on_enter(self):
        """Stop all motion when entering system mode."""
        self.controller.get_logger().info("Entered system control mode")
        self.controller.publish_zero_velocity()
    
    def on_exit(self):
        """Clean exit from system mode."""
        self.controller.get_logger().info("Exiting system control mode")


# ============================================================================
# MAIN ROBOT CONTROLLER
# ============================================================================

class RobotController(Node):
    """
    Main robot controller with mode-based behaviors.
    Subscribes to teleop commands and publishes velocity commands to hardware.
    """
    
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info("Robot Controller starting...")
        
        # Load parameters
        self._load_parameters()
        
        # Initialize shared components
        self._init_shared_components()
        
        # Initialize behaviors
        self._init_behaviors()
        
        # Initialize state
        self.current_mode = "translation"
        self.current_behavior = self.behaviors["translation"]
        self.current_pose = None
        self.current_vel = None
        self.current_finger_pose = np.array([0.0, 0.0, 0.0])
        self.latest_velocity_cmd = None
        
        # Initialize ROS interfaces
        self._init_publishers()
        self._init_subscribers()
        self._init_tf()
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / REFRESH_RATE, self.control_tick)
        
        self.get_logger().info("Robot Controller initialized successfully")
    
    def _load_parameters(self):
        """Load ROS parameters."""
        # Cartesian movement
        self.cartesian_movement_enabled = self.declare_parameter(
            "cartesian_movement_enabled", True).value
        
        # Max velocities
        self.max_linear_velocity = self.declare_parameter(
            "max_linear_velocity", list(MAX_LINEAR_VELOCITY)).value
        self.max_angular_velocity = self.declare_parameter(
            "max_angular_velocity", MAX_ANGULAR_VELOCITY).value
        self.max_finger_velocity = self.declare_parameter(
            "max_finger_velocity", MAX_FINGER_VELOCITY).value
        
        # PID parameters
        self.pid_enabled = self.declare_parameter("pid_enabled", True).value
        self.kp_linear = self.declare_parameter("kp_linear", 0.8).value
        self.ki_linear = self.declare_parameter("ki_linear", 0.1).value
        self.kd_linear = self.declare_parameter("kd_linear", 0.0).value
        self.kp_angular = self.declare_parameter("kp_angular", 2.0).value
        self.ki_angular = self.declare_parameter("ki_angular", 2.0).value
        self.kd_angular = self.declare_parameter("kd_angular", 5.0).value

        # Rotation
        self.discretize_rotation = self.declare_parameter("discretise_rotation", True).value
        self.quantization_degrees = self.declare_parameter("quantisation_degrees", 30.0).value
        
        # Log parameters
        self.get_logger().info("=== Parameters loaded successfully ===")
        self.get_logger().info(f"Max linear velocity: {self.max_linear_velocity}")
        self.get_logger().info(f"Max angular velocity: {self.max_angular_velocity}")
        self.get_logger().info(f"Max finger velocity: {self.max_finger_velocity}")
        # self.get_logger().info(f"Rotation assistance: {self.rotation_assistance}")

        self.get_logger().info(f"PID enabled: {self.pid_enabled}")
        self.get_logger().info(f"KP linear: {self.kp_linear}")
        self.get_logger().info(f"KI linear: {self.ki_linear}")
        self.get_logger().info(f"KD linear: {self.kd_linear}")
        self.get_logger().info(f"KP angular: {self.kp_angular}")
        self.get_logger().info(f"KI angular: {self.ki_angular}")
        self.get_logger().info(f"KD angular: {self.kd_angular}")
        self.get_logger().info(f"Discretize rotation: {self.discretize_rotation}")
        self.get_logger().info(f"Quantization degrees: {self.quantization_degrees}")

    def _init_shared_components(self):
        """Initialize shared utility components."""
        # PID controllers
        self.pid_linear = PIDController(
            kp=self.kp_linear,
            ki=self.ki_linear,
            kd=self.kd_linear,
            refresh_rate=REFRESH_RATE,
            length=3
        )
        
        self.pid_angular = PIDController(
            kp=self.kp_angular,
            ki=self.ki_angular,
            kd=self.kd_angular,
            refresh_rate=REFRESH_RATE,
            length=3
        )
        
        # Filters
        self.vel_filter = RollingAverageFilter(window_size=5)
        
        # Velocity integrator
        self.vel_integrator = VelocityIntegrator(
            max_velocity=tuple(self.max_linear_velocity),
            refresh_rate=REFRESH_RATE
        )
        
        # Rotation controller
        self.rotation_controller = RotationController(
            start_rotation=START_ROTATION,
            quantization_degrees=self.quantization_degrees,
            max_angular_velocity=self.max_angular_velocity,
            refresh_rate=REFRESH_RATE
        )
    
    def _init_behaviors(self):
        """Initialize mode-specific behaviors."""
        self.behaviors = {
            "translation": ContinuousTeleopBehavior(self, reference_frame="j2n6s300_link_base"),
            "rotation": ContinuousTeleopBehavior(self, reference_frame="j2n6s300_link_base"),
            "discrete": DiscreteTeleopBehavior(self),
            "system": SystemBehavior(self)
        }
    
    def _init_publishers(self):
        """Initialize ROS publishers."""
        self.vel_pub = self.create_publisher(
            PoseVelocityWithFingerVelocity,
            '/j2n6s300_driver/in/cartesian_velocity_with_finger_velocity',
            1
        )
        
        # Debug publishers
        self.test_target_vel_pub = self.create_publisher(Point, '/test/requested_vel_pub', 1)
        self.test_pid_vel_pub = self.create_publisher(Point, '/test/pid_target_vel_pub', 1)
        self.test_measured_vel_pub = self.create_publisher(Point, '/test/measured_vel_pub', 1)
        
        # Haptics
        self.haptics_action_pub = self.create_publisher(str_msg, "/haptic_feedback_robot_string", 1)
    
    def _init_subscribers(self):
        """Initialize ROS subscribers."""
        # Mode management
        self.mode_sub = self.create_subscription(
            str_msg,
            '/teleop/current_mode',
            self.mode_callback,
            10
        )
        
        # Velocity commands from CommandMapper
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/teleop/cartesian_velocity',
            self.velocity_callback,
            10
        )
        
        # Discrete commands from CommandMapper
        self.discrete_sub = self.create_subscription(
            PoseStamped,
            '/teleop/discrete_pose',
            self.discrete_callback,
            10
        )
        
        # System commands
        self.system_sub = self.create_subscription(
            str_msg,
            '/teleop/system',
            self.system_callback,
            10
        )
        
        # Jaco robot state
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/j2n6s300_driver/out/tool_pose',
            self.update_current_pose,
            10
        )
        
        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/j2n6s300_driver/out/tool_wrench',
            self.update_current_force,
            10
        )
        
        self.finger_sub = self.create_subscription(
            FingerPosition,
            '/j2n6s300_driver/out/finger_position',
            self.update_current_finger_pose,
            10
        )
    
    def _init_tf(self):
        """Initialize TF2 components."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
    
        # TODO Target tracking TF listener
        self.tf_target_buffer = Buffer()
        self.tf_target_listener = TransformListener(self.tf_target_buffer, self)

    # ========================================================================
    # CALLBACKS
    # ========================================================================
    
    def mode_callback(self, msg: str_msg):
        """Handle mode change from ModeManager."""
        new_mode = msg.data
        
        if new_mode not in self.behaviors:
            self.get_logger().warn(f"Unknown mode requested: {new_mode}")
            return
        
        if new_mode != self.current_mode:
            self.get_logger().info(f"Switching mode: {self.current_mode} -> {new_mode}")
            
            # Exit old behavior
            if self.current_behavior:
                self.current_behavior.on_exit()
            
            # Switch mode
            self.current_mode = new_mode
            self.current_behavior = self.behaviors[new_mode]
            
            # Enter new behavior
            self.current_behavior.on_enter()
    
    def velocity_callback(self, msg: TwistStamped):
        """Store latest velocity command from CommandMapper."""
        self.latest_velocity_cmd = msg
    
    def discrete_callback(self, msg: PoseStamped):
        """Handle discrete pose commands from CommandMapper."""
        if self.current_behavior:
            self.current_behavior.process_discrete_command(msg)
    
    def system_callback(self, msg: str_msg):
        """Handle system commands."""
        cmd = msg.data
        self.get_logger().info(f"System command received: {cmd}")
        
        if cmd == "reset_pose":
            self._reset_pose()
        elif cmd == "emergency_stop":
            self._emergency_stop()
        else:
            self.get_logger().warn(f"Unknown system command: {cmd}")
    
    def update_current_pose(self, pose_msg: PoseStamped):
        """Update current end-effector pose."""
        prev_pose = self.current_pose
        self.current_pose = pose_msg
        self._update_current_velocity(self.current_pose, prev_pose)
    
    def update_current_finger_pose(self, finger_msg: FingerPosition):
        """Update current finger positions."""
        self.current_finger_pose = np.array([
            finger_msg.finger1,
            finger_msg.finger2,
            finger_msg.finger3
        ])
    
    def update_current_force(self, force_msg: WrenchStamped):
        """Update current force/torque measurements."""
        self.current_force = force_msg
    
    def _update_current_velocity(self, current_pose: PoseStamped, 
                                 prev_pose: Optional[PoseStamped]):
        """Compute current velocity from pose differences."""
        if prev_pose is None:
            self.current_vel = np.zeros(6)
            self.time_prev = self.get_clock().now()
            return
        
        current_time = self.get_clock().now()
        duration = (current_time - self.time_prev).nanoseconds * 1e-9
        
        if duration < 1e-6:
            return
        
        raw_vel = np.array([
            current_pose.pose.position.x - prev_pose.pose.position.x,
            current_pose.pose.position.y - prev_pose.pose.position.y,
            current_pose.pose.position.z - prev_pose.pose.position.z,
            current_pose.pose.orientation.w - prev_pose.pose.orientation.w,
            current_pose.pose.orientation.x - prev_pose.pose.orientation.x,
            current_pose.pose.orientation.y - prev_pose.pose.orientation.y
        ]) / duration
        
        self.current_vel = self.vel_filter.update(raw_vel)
        self.time_prev = current_time
    
    # ========================================================================
    # CONTROL LOOP
    # ========================================================================
    
    def control_tick(self):
        """Main control loop - runs at fixed rate."""
        if not self.current_behavior or not self.current_pose:
            return
        
        try:
            # Process latest velocity command through current behavior
            if self.latest_velocity_cmd:
                cmd = self.current_behavior.process_velocity_command(self.latest_velocity_cmd)
                
                if cmd:
                    # Publish debug information
                    self.test_target_vel_pub.publish(Point(
                        x=cmd.twist_linear_x,
                        y=cmd.twist_linear_y,
                        z=cmd.twist_linear_z
                    ))
                    
                    # TODO: ? Odd, check later
                    if self.current_vel is not None:
                        self.test_measured_vel_pub.publish(Point(
                            x=self.current_vel[0],
                            y=self.current_vel[1],
                            z=self.current_vel[2]
                        ))
                    
                    # Publish velocity command
                    self.vel_pub.publish(cmd)
            else:
                # No command - publish zero velocity
                self.publish_zero_velocity()
        
        except Exception as e:
            self.get_logger().error(f'Error in control_tick: {e}')
            traceback.print_exc()
    
    # ========================================================================
    # UTILITY METHODS
    # ========================================================================
    # TODO: Move to utility module ?

    def transform_velocity(self, velocity: np.ndarray, 
                          source_frame: str, target_frame: str) -> np.ndarray:
        """
        Transform velocity vector between frames.
        
        Args:
            velocity: Velocity in source frame
            source_frame: Source frame ID
            target_frame: Target frame ID
            
        Returns:
            Velocity in target frame
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            
            # Extract rotation and apply to velocity
            rot = Rotation.from_quat([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            
            transformed_vel = rot.inv().apply(velocity)
            return transformed_vel
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Error transforming velocity: {e}')
            return velocity
    
    def get_ee_rotation(self) -> Optional[Rotation]:
        """
        Get current end-effector rotation.
        
        Returns:
            Current rotation as Rotation object, or None if unavailable
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'j2n6s300_link_base',
                'j2n6s300_end_effector',
                rclpy.time.Time()
            )
            
            current_rot = Rotation.from_quat([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            
            return current_rot
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Error getting EE rotation: {e}')
            return None
    
    def publish_zero_velocity(self):
        """Publish zero velocity command to stop robot."""
        msg = PoseVelocityWithFingerVelocity()
        msg.twist_linear_x = 0.0
        msg.twist_linear_y = 0.0
        msg.twist_linear_z = 0.0
        msg.twist_angular_x = 0.0
        msg.twist_angular_y = 0.0
        msg.twist_angular_z = 0.0
        msg.finger1 = 0.0
        msg.finger2 = 0.0
        msg.finger3 = 0.0
        
        self.vel_pub.publish(msg)
    
    def publish_rotation_target(self, rotation: Rotation, frame_id: str, 
                               child_frame_id: str, visual_offset: list = [0.0, 0.0, 0.0]):
        """
        Publish rotation target as TF for visualization.
        
        Args:
            rotation: Target rotation
            frame_id: Parent frame
            child_frame_id: Child frame name
            visual_offset: Position offset for visualization
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id
        
        transform.transform.translation.x = visual_offset[0]
        transform.transform.translation.y = visual_offset[1]
        transform.transform.translation.z = visual_offset[2]
        
        quat = rotation.as_quat()
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)
    
    # ========================================================================
    # SYSTEM COMMANDS
    # ========================================================================
    
    def _reset_pose(self):
        """Reset robot to home pose."""
        self.get_logger().info("Resetting robot pose...")
        
        # Stop current motion
        self.publish_zero_velocity()
        
        # Reset component states
        self.pid_linear.reset()
        self.pid_angular.reset()
        self.vel_integrator.reset()
        self.rotation_controller.reset()
        
        # TODO: Send robot to home position
        # This would typically involve an action client call
        
        self.get_logger().info("Reset complete")
    
    def _emergency_stop(self):
        """Emergency stop - immediately halt all motion."""
        self.get_logger().warn("EMERGENCY STOP ACTIVATED")
        
        # Send zero velocity
        self.publish_zero_velocity()
        
        # Switch to system mode to prevent further commands
        self.current_mode = "system"
        self.current_behavior = self.behaviors["system"]
        self.current_behavior.on_enter()
        
        self.get_logger().warn("Robot stopped. Switch mode to resume operation.")


# ============================================================================
# FINGER CONTROL EXTENSION
# ============================================================================

class FingerController:
    """
    Manages finger/gripper control with collision detection and haptics.
    """
    
    def __init__(self, controller: 'RobotController', max_velocity: float = 2000.0):
        self.controller = controller
        self.max_velocity = max_velocity
        self.prev_finger_pose = np.zeros(3)
        self.collision_threshold = 1.0
    
    def compute_finger_velocity(self, command: float) -> np.ndarray:
        """
        Compute finger velocities from command.
        
        Args:
            command: Normalized command [-1, 1]
            
        Returns:
            Finger velocity array [finger1, finger2, finger3]
        """
        velocity = np.ones(3) * command * self.max_velocity
        return velocity
    
    def check_collision(self, target_velocity: np.ndarray, 
                       current_pose: np.ndarray) -> bool:
        """
        Check if fingers are colliding (not moving despite command).
        
        Args:
            target_velocity: Commanded velocity
            current_pose: Current finger positions
            
        Returns:
            True if collision detected
        """
        fingers_should_move = np.linalg.norm(target_velocity) > 0.2
        if not fingers_should_move:
            return False
        
        # Check if fingers are actually moving
        movement = np.linalg.norm(current_pose - self.prev_finger_pose)
        are_fingers_moving = movement > self.collision_threshold
        
        self.prev_finger_pose = current_pose.copy()
        
        # Collision if should move but aren't
        return fingers_should_move and not are_fingers_moving
    
    def publish_haptic_feedback(self, collision: bool):
        """Publish haptic feedback based on collision state."""
        msg = str_msg()
        msg.data = "grasping" if collision else "none"
        self.controller.haptics_action_pub.publish(msg)


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main(args=None):
    """Main entry point for robot controller node."""
    rclpy.init(args=args)
    
    try:
        controller = RobotController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in main: {e}')
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()