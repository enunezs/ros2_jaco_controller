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
from rclpy.time import Time

from kinova_msgs.msg import PoseVelocity, PoseVelocityWithFingerVelocity, FingerPosition
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped, Point, TwistStamped
from std_msgs.msg import String as str_msg
# from std_msgs.msg import Int32 as int_msg
from nav_msgs.msg import Path

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

import traceback
from abc import ABC, abstractmethod
from collections import deque
from typing import Optional, Tuple, Dict

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


# ============================================================================
# CONSTANTS AND CONFIGURATION
# ============================================================================

REFRESH_RATE = 100.0
# Define start rotation (using Quaternions internally is safer)
START_ROTATION = Rotation.from_euler('xyz', [-180-20, 0-5, 180-10], degrees=True)
MAX_LINEAR_VELOCITY = (0.1, 0.06, 0.08)
MAX_ANGULAR_VELOCITY = 2.0
MAX_FINGER_VELOCITY = 2000.0
ROBOT_BASE_FRAME = "j2n6s300_link_base"

# TODO
ROTATION_ASSISTANCE = 2.0   


    # ==========================
    # TF HELPER FUNCTIONS
    # ==========================

    def transform_vector(self, vector: np.ndarray, source_frame: str, target_frame: str) -> np.ndarray:
        """Rotates a vector from source frame to target frame."""
        if source_frame == target_frame:
            return vector
        try:
            # We only care about rotation for vectors
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            rot = Rotation.from_quat([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            return rot.apply(vector)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(f"Could not transform vector {source_frame} -> {target_frame}")
            return vector

    def get_frame_rotation(self, source_frame: str, target_frame: str) -> Optional[Rotation]:
        """Gets the rotation of source_frame represented in target_frame."""
        if not source_frame or source_frame == target_frame:
            return None # Identity
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return Rotation.from_quat([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def get_ee_rotation(self) -> Optional[Rotation]:
        return self.get_frame_rotation('j2n6s300_end_effector', ROBOT_BASE_FRAME)

    def publish_zero_velocity(self):
        msg = PoseVelocityWithFingerVelocity()
        self.vel_pub.publish(msg)

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

        # Error calculation
        current_error = target_vel - current_vel

        # Integral term with anti-windup
        self.integral_error += current_error * (1.0 / self.refresh_rate)
        if self.anti_windup:
            self.integral_error = np.clip(self.integral_error, -1.0, 1.0)
        
        # Derivative term
        self.derivative_error = (current_error - self.prev_error) * self.refresh_rate
        self.prev_error = current_error

        compensation = self.kp * current_error + self.ki * self.integral_error + self.kd * self.derivative_error
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

    def update(self, value: float) -> float:
        """Add new value and return filtered output."""
        self.values.append(value)
        return sum(self.values) / len(self.values)

    def reset(self):
        """Clear filter state."""
        self.values.clear()

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
            velocity[idx] = np.clip(velocity[idx], -self.max_velocity[idx], self.max_velocity[idx])
        
        self.prev_velocity = velocity
        return velocity

    def reset(self):
        """Reset to zero velocity."""
        self.prev_velocity = np.zeros(3)

class RotationController:
    """
    Manages rotation control with quantization, tracking, and velocity computation.
    """
    
    def __init__(self, 
                 start_rotation: Rotation, 
                 quantization_degrees: float = 30.0,
                 max_angular_velocity: float = 2.0, 
                 refresh_rate: float = 100.0):
        """
        Initialize rotation controller.
        Args:
            start_rotation: Initial rotation state.
            quantization_degrees: Degree of quantization for rotation.
            max_angular_velocity: Maximum angular velocity for rotation.
            refresh_rate: Rate at which to refresh the controller.
        """

        self.start_rotation = start_rotation
        self.quantization_degrees = quantization_degrees
        self.max_angular_velocity = max_angular_velocity
        self.refresh_rate = refresh_rate

        self.discretize_enabled = True

        self.cumulative_rotation : Rotation = START_ROTATION
        # TODO: Add reference frame handling if needed
        self.reference_frame = "j2n6s300_link_base" #-> needs a setter?

    def update_target_rotation_from_input(self, 
                                          orientation_input: np.ndarray, 
                                          dt: float,
                                          basis_rotation: Optional[Rotation] = None) -> Rotation:
        """
        Bi-stable rotation update:
        - When user is moving -> follow velocity input
        - When user stops -> settle to nearest quantized zone

        Updates the internal target rotation based on user input relative to a specific frame.
        
        Args:
            orientation_input: [rx, ry, rz] velocity request from joystick (approx -1 to 1)
            dt: Time delta
            basis_rotation: The rotation of the frame the input is defined in, relative to robot base.
                            If None, assumes input is already in Base frame.
        """
        speed_factor = 2.0 # Rad/s max speed approx
        # speed_factor = 10.0
        
        # 1. Deadband
        if np.linalg.norm(orientation_input) < 0.05:
            user_active = False
            orientation_change_vec = np.zeros(3)
        else:
            user_active = True
            # Raw input vector
            orientation_change_vec = orientation_input * speed_factor * dt

        # 2. Transform Input to Base Frame
        # If the user pushes "Forward" (X) in the Camera Frame, and the Camera is rotated 90 deg,
        # we need to apply that rotation to the vector so the robot moves correctly in Base frame.
        if basis_rotation is not None and user_active:
            # Rotate the input vector by the basis rotation
            orientation_change_vec = basis_rotation.apply(orientation_change_vec)

        # 3. Apply Integration (Quaternion Multiplication)
        if user_active:
            # Create a small rotation from the vector
            # Magnitude of vector = angle in radians, Direction = axis
            rot_delta = Rotation.from_rotvec(orientation_change_vec)
            # Apply: New = Delta * Old (Intrinsic) or Old * Delta (Extrinsic)
            # Usually for "Base Frame" accumulation, we want:
            self.cumulative_rotation = rot_delta * self.cumulative_rotation

        # 4. Quantization / Snapping
        # Convert to Euler for grid snapping, then back
        # Note: We snap in the BASE frame (xyz), which is usually what feels "straight"
        current_euler = self.cumulative_rotation.as_euler('xyz', degrees=True)
        q = self.quantization_degrees
        target_euler_snapped = np.round(current_euler / q) * q
        target_rot_snapped = Rotation.from_euler('xyz', target_euler_snapped, degrees=True)
        
        # 5. Smoothing / Settling
        if not user_active:
            # Use SLERP (Spherical Linear Interpolation) for smooth rotation
            # This fixes the wrap-around bug (359 -> 1 degree)
            settle_gain = 6.0
            alpha = 1 - np.exp(-settle_gain * dt)
            
            # Interpolate between current and snapped
            key_times = [0, 1]
            key_rots = Rotation.concatenate([self.cumulative_rotation, target_rot_snapped])
            slerp = Slerp(key_times, key_rots)
            
            # Update current towards target
            self.cumulative_rotation = slerp(alpha)

        return self.cumulative_rotation

    def compute_angular_velocity(self, target_rotation: Rotation, 
                        current_rotation: Rotation, 
                        pid_controller: Optional[PIDController] = None) -> np.ndarray:
        """
        Compute angular velocity [rx, ry, rz] to reach target.
        """
        # 1. Calculate error: Target * Inverse(Current) -> Difference in Global Frame
        # Or: Current.inv * Target -> Difference in Local Frame
        # We usually want velocity commands in the End Effector (Local) or Base frame depending on robot driver.
        # Assuming Jaco accepts 'PoseVelocity' in Base frame (usually), but check driver!
        
        # error_rot = target_rotation * current_rotation.inv() # Global Frame
        error_rot = current_rotation.inv() * target_rotation # Local Frame
        
        # 2. Convert to Rotation Vector (Axis-Angle)
        rot_vec = error_rot.as_rotvec() # Vector direction = axis, Magnitude = angle (rads)
        
        # 3. Clamp Magnitude (Safety)
        mag = np.linalg.norm(rot_vec)
        MAX_ERROR_RAD = 1.0 # 57 degrees
        if mag > MAX_ERROR_RAD:
            rot_vec = rot_vec * (MAX_ERROR_RAD / mag)
            
        # 4. PID or Proportional
        if pid_controller:
             # Assuming PID controller keeps track of dt internally
            return pid_controller.control_update(rot_vec, np.zeros(3))
        else:
            return rot_vec * self.max_angular_velocity

    def reset(self):
        self.cumulative_rotation = START_ROTATION


    def old_update_target_rotation_from_input(self, 
                                          orientation_input: np.ndarray, 
                                          dt: float,
                                          basis_rotation: Optional[Rotation] = None) -> Rotation:
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
            error = quantized_target - self.cumulative_rotation
            # Wrap error to range [-180, 180] (assuming degrees)
            error = (error + 180) % 360 - 180
            
            self.cumulative_rotation += alpha * error

        # ---------------------------------------
        # 6. Convert to rotation
        # ---------------------------------------
        rot_controller = Rotation.from_euler('xyz', self.cumulative_rotation, degrees=True)

        return self.start_rotation * rot_controller

    def old_compute_angular_velocity(self, target_rotation: Rotation, 
                        current_rotation: Rotation, 
                        pid_controller: Optional[PIDController] = None) -> np.ndarray:
        """
        Compute angular velocity to reach target rotation.cumulative_rotation
        
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
    Based on the Frame ID of the input command
    """
    
    def __init__(self, controller: 'RobotController'):
        super().__init__(controller)
    
    def process_velocity_command(self, twist: TwistStamped) -> Optional[PoseVelocityWithFingerVelocity]:
        """
        Process continuous velocity command with PID and smoothing.
        """
        if not self.controller.current_pose:
            return None
        
        ### Message Extraction ###
        # Extract Linear
        target_linear_vel = np.array([twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z])

        ### 1. Handle Frame Transforms for Linear Velocity ###
        # The Twist message tells us what frame the input is in (e.g., "head_camera", "base_link")
        input_frame = twist.header.frame_id        

        # If input is not in base frame, rotate the linear velocity vector
        if input_frame and input_frame != ROBOT_BASE_FRAME:
            target_linear_vel = self.controller.transform_vector(
                target_linear_vel, input_frame, ROBOT_BASE_FRAME
            )

        # Apply Smoothing & PID (Linear)
        target_linear_vel = self.controller.vel_integrator.update(target_linear_vel, dt=1.0/REFRESH_RATE)
        if self.controller.current_vel is not None:
             # Basic Feedforward + PID correction
             pid_out = self.controller.pid_linear.control_update(target_linear_vel, self.controller.current_vel[0:3])
             target_linear_vel = target_linear_vel + pid_out


        ### 2. Handle Rotation Control ###
        target_angular_input = np.array([twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z])
        
        # Look up the rotation of the input frame relative to base
        # This allows "Up" on joystick to mean "Up" in camera view, etc.
        basis_rotation = self.controller.get_frame_rotation(input_frame, ROBOT_BASE_FRAME)

        # Update the Target Orientation state
        rotation_target = self.controller.rotation_controller.update_target_rotation_from_input(
            target_angular_input,
            dt=1.0/REFRESH_RATE,
            basis_rotation=basis_rotation
        )
        
        # Compute Velocity to reach that target
        current_ee_rot = self.controller.get_ee_rotation()
        if current_ee_rot:
            target_angular_vel = self.controller.rotation_controller.compute_angular_velocity(
                rotation_target,
                current_ee_rot,
                pid_controller=None # or self.controller.pid_angular
            )
        else:
            target_angular_vel = np.zeros(3)
 
         # Construct Message
        msg = PoseVelocityWithFingerVelocity()
        msg.twist_linear_x, msg.twist_linear_y, msg.twist_linear_z = target_linear_vel
        msg.twist_angular_x, msg.twist_angular_y, msg.twist_angular_z = target_angular_vel

        # TODO
        msg.finger_velocity1, msg.finger_velocity2, msg.finger_velocity3 = [0.0, 0.0, 0.0]
        return msg
        
    
    def process_discrete_command(self, pose: PoseStamped) -> bool:
        """Continuous mode ignores discrete commands."""
        self.controller.get_logger().warn(
            "Discrete command received in continuous mode - ignoring"
        )
        return False
    
    def on_enter(self):
        """Reset integrators when entering continuous mode."""
        self.controller.get_logger().info(f"Entered continuous teleop mode")
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
        # self.get_logger().info("Initialized RotationContinuousBehavior")
    
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

class DiscreteTeleopBehavior(ControlBehavior):
    """
    Discrete teleoperation behavior.
    Internal state machine: IDLE → EXECUTING → PAUSED → COMPLETED → IDLE
    
    Features:
    - Executes waypoint paths using velocity control
    - Supports pause/resume (preserves progress mid-waypoint)
    - Position and orientation tracking with thresholds
    - Timeout safety per waypoint
    - Progress logging

    """

    # Internal execution states (independent of mode manager)
    IDLE = "idle"
    EXECUTING = "executing"
    PAUSED = "paused"
    COMPLETED = "completed"

    def __init__(self, controller: 'RobotController'):
        super().__init__(controller)

        # State management
        self.internal_state = self.IDLE

        # Waypoint queue tracking
        self.waypoint_queue = deque()  # Queue of remaining waypoints
        self.total_waypoints = 0       # Total waypoints in original path
        self.current_waypoint_index = 0  # Index in original path
        self.current_target = None      # Current target pose

        # Timing
        self.waypoint_start_time = None  # When we started approaching current waypoint
        self.execution_start_time = None  # When entire path execution started
        self.waypoint_elapsed_accumulated = 0.0      # Cumulative time tracker for pausing and resuming
     
        # Load parameters from controller
        self._load_parameters()
        
        # self.executing_action = False
        # self.target_pose = None
        # self.current_waypoint_index = 0
    
    def _load_parameters(self):
        """Load behavior-specific parameters from controller."""

        # Speed as percentage of max velocity
        self.discrete_motion_speed = getattr(
            self.controller, 'discrete_motion_speed', 0.5
        )
        # Completion thresholds
        self.position_threshold = getattr(
            self.controller, 'waypoint_position_threshold', 0.005  # 5mm
        )
        self.orientation_threshold_deg = getattr(
            self.controller, 'waypoint_orientation_threshold_deg', 5.0  # 5 degrees
        )
        
        # Safety timeout per waypoint
        self.waypoint_timeout_sec = getattr(
            self.controller, 'waypoint_timeout_sec', 20.0  # 8 seconds
        )
    
        self.controller.get_logger().info(
            f"DiscreteTeleopBehavior initialized with thresholds: "
            f"speed={self.discrete_motion_speed*100:.0f}%, "
            f"pos={self.position_threshold*1000:.1f}mm, "
            f"orient={self.orientation_threshold_deg:.1f}deg, "
            f"timeout={self.waypoint_timeout_sec:.1f}s, "
        )
    
    # ========================================================================
    # BEHAVIOR INTERFACE (required by ControlBehavior)
    # Needs to include defs for process_velocity_command and process_discrete_command 
    # ========================================================================
    
    def process_velocity_command(self, twist: TwistStamped) -> Optional[PoseVelocityWithFingerVelocity]:
        """
        Execute waypoint following using velocity control.
        
        This is called every control tick when in discrete mode.
        Ignores incoming twist commands - uses internal waypoint logic instead.
        
        Args:
            twist: Incoming velocity command (ignored in discrete mode)
            
        Returns:
            Velocity command to reach current waypoint, or None if idle, paused or completed.
        """
        # Do nothing if idle or paused
        if self.internal_state in [self.IDLE, self.PAUSED]:
            return None
        
        # Check if execution completed
        if self.internal_state == self.COMPLETED:
            return None
        
        # Check if queue is empty (shouldn't happen if state management is correct)
        if not self.waypoint_queue:
            self._handle_completion()
            return None
        
        # Initialize current target if needed
        if self.current_target is None:
            self.current_target = self.waypoint_queue[0]
            self.waypoint_start_time = self.controller.get_clock().now()
            self.controller.get_logger().info(
                f"Starting waypoint {self.current_waypoint_index + 1}/{self.total_waypoints}"
            )
        
        # Check for timeout on current waypoint
        if self._check_timeout():
            self.controller.get_logger().warn(
                f"Waypoint {self.current_waypoint_index + 1}/{self.total_waypoints} "
                f"timeout after {self.waypoint_timeout_sec}s - skipping to next"
            )
            self._advance_waypoint()
            return None
        
        # Check if we've reached the current waypoint
        if self._check_reached_waypoint():
            self.controller.get_logger().info(
                f"✓ Reached waypoint {self.current_waypoint_index + 1}/{self.total_waypoints}"
            )
            self._advance_waypoint()
            
            # Return None this tick to allow state to settle
            return None
        
        # Compute velocity toward current waypoint
        return self._compute_velocity_to_target()

    def process_discrete_command(self, pose: PoseStamped) -> bool:
        """
        Single discrete pose commands are not used in this behavior.
        Use load_waypoints() instead to load full paths.
        
        Returns:
            False (command not handled)
        """
        self.controller.get_logger().warn(
            "Single discrete pose command received but DiscreteTeleopBehavior "
            "uses waypoint paths. Use /teleop/waypoint_path instead."
        )
        return False

    def on_enter(self):
        """
        Called when mode switches TO discrete.
        
        If execution was paused, this allows it to resume.
        If idle, waits for waypoint list to be loaded.
        """
        
        self.controller.get_logger().info("Entered discrete teleop mode")
        
        if self.internal_state == self.PAUSED:
            self.controller.get_logger().info(
                f"Resuming paused execution at waypoint "
                f"{self.current_waypoint_index + 1}/{self.total_waypoints}"
            )
            # Don't automatically resume - wait for explicit resume command
            # User can send resume_waypoints if desired

        elif self.internal_state == self.IDLE:
            self.controller.get_logger().info(
                "Waiting for waypoint path (send to /teleop/waypoint_path)"
            )
            # Don't automatically resume - wait for explicit resume command

    def on_exit(self):
        """
        Called when mode switches AWAY from discrete.
        
        Option A (implemented): Auto-pause execution, preserving progress.
        User can return to discrete mode to resume.
        """
        self.controller.get_logger().info("Exiting discrete teleop mode")
        
        # If currently executing, auto-pause (Option A)
        if self.internal_state == self.EXECUTING:
            self.internal_state = self.PAUSED
            self.controller.get_logger().info(
                f"Execution auto-paused at waypoint "
                f"{self.current_waypoint_index + 1}/{self.total_waypoints}. "
                "Return to discrete mode to resume."
            )
        
        # Always stop motion when leaving mode
        self.controller.publish_zero_velocity()
    
    # ========================================================================
    # WAYPOINT MANAGEMENT
    # ========================================================================
    
    def load_waypoints(self, path: Path):
        """
        Load a new waypoint path for execution.
        
        Can only load when IDLE. If execution is in progress, must stop first.
        
        Args:
            path: nav_msgs/Path message with waypoint list
            
        Returns:
            True if waypoints loaded successfully, False otherwise
        """
        if self.internal_state != self.IDLE:
            self.controller.get_logger().warn(
                f"Cannot load waypoints in state '{self.internal_state}'. "
                "Send stop_waypoints command first."
            )
            return False
        
        if not path.poses:
            self.controller.get_logger().error("Received empty waypoint path")
            return False
        
        # Load waypoints into queue
        self.waypoint_queue = deque(path.poses)
        self.total_waypoints = len(path.poses)
        self.current_waypoint_index = 0
        self.current_target = None
        
        # Start execution
        self.internal_state = self.EXECUTING
        self.execution_start_time = self.controller.get_clock().now()
        
        self.controller.get_logger().info(
            f"========================================"
        )
        self.controller.get_logger().info(
            f"Loaded {self.total_waypoints} waypoints, starting execution"
        )
        self.controller.get_logger().info(
            f"Speed: {self.discrete_motion_speed*100:.0f}% of max velocity"
        )
        self.controller.get_logger().info(
            f"========================================"
        )
        
        return True
    
    def _advance_waypoint(self):
        """
        Move to the next waypoint in the queue.
        Handles completion when queue is empty.
        """
        # Remove completed waypoint
        if self.waypoint_queue:
            self.waypoint_queue.popleft()
        
        # Increment index
        self.current_waypoint_index += 1
        
        # Clear current target
        self.current_target = None
        
        # Reset timing values for next waypoint
        self.waypoint_elapsed_accumulated = 0.0
        self.waypoint_start_time = self.controller.get_clock().now()


        # Check if we've completed all waypoints
        if not self.waypoint_queue:
            self._handle_completion()
        else:
            # Log progress to next waypoint
            self.controller.get_logger().info(
                f"→ Moving to waypoint {self.current_waypoint_index + 1}/{self.total_waypoints}"
            )
    
    def _handle_completion(self):
        """
        Handle completion of entire waypoint path.
        Logs completion time and returns to IDLE state.
        """
        if self.execution_start_time:
            elapsed = (self.controller.get_clock().now() - self.execution_start_time).nanoseconds / 1e9
            self.controller.get_logger().info(
                f"========================================"
            )
            self.controller.get_logger().info(
                f"✓ Waypoint execution COMPLETED"
            )
            self.controller.get_logger().info(
                f"Total waypoints: {self.total_waypoints}"
            )
            self.controller.get_logger().info(
                f"Total time: {elapsed:.1f} seconds"
            )
            self.controller.get_logger().info(
                f"========================================"
            )
        
        # Return to idle state
        self.internal_state = self.IDLE
        self.waypoint_queue.clear()
        self.current_target = None
        self.current_waypoint_index = 0
        self.total_waypoints = 0
        
        # Stop motion
        self.controller.publish_zero_velocity()
        
    # ========================================================================
    # WAYPOINT TRACKING
    # ========================================================================

    def _check_reached_waypoint(self) -> bool:
        """
        Check if current waypoint has been reached.
        
        Requires both position and orientation to be within thresholds.
        
        Returns:
            True if waypoint reached, False otherwise
        """
        if not self.controller.current_pose or not self.current_target:
            return False
        
        current_pose = self.controller.current_pose
        target_pose = self.current_target.pose
        
        # ===== Position Check =====
        pos_error = np.array([
            target_pose.position.x - current_pose.pose.position.x,
            target_pose.position.y - current_pose.pose.position.y,
            target_pose.position.z - current_pose.pose.position.z
        ])
        position_distance = np.linalg.norm(pos_error)
        position_reached = position_distance < self.position_threshold
        
        # ===== Orientation Check =====
        current_rot = self.controller.get_ee_rotation()
        if current_rot is None:
            # If we can't get rotation, only check position
            return position_reached
        
        target_rot = Rotation.from_quat([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ])
        
        # Compute rotation difference
        rot_diff = current_rot.inv() * target_rot
        
        # Get angle magnitude from quaternion
        angle_error_rad = 2 * np.arccos(np.clip(abs(rot_diff.as_quat()[3]), 0.0, 1.0))
        angle_error_deg = np.degrees(angle_error_rad)
        orientation_reached = angle_error_deg < self.orientation_threshold_deg
        
        # Debug logging (can be commented out for production)
        # self.controller.get_logger().debug(
        #     f"Waypoint tracking - pos: {position_distance*1000:.1f}mm "
        #     f"(thresh: {self.position_threshold*1000:.1f}mm), "
        #     f"orient: {angle_error_deg:.1f}deg "
        #     f"(thresh: {self.orientation_threshold_deg:.1f}deg)"
        # )
        
        return position_reached and orientation_reached
    
    def _check_timeout(self) -> bool:
        """
        Check if current waypoint has timed out.
        Use active execution to allow pausing and resuming cleanly.

        Returns:
            True if timeout exceeded, False otherwise
        """
        if not self.waypoint_start_time:
            return False
        

        # time since last resume
        active_time = (self.controller.get_clock().now() - self.waypoint_start_time).nanoseconds / 1e9

        # total time = accumulated before pause + newly active
        elapsed = self.waypoint_elapsed_accumulated + active_time

        return elapsed > self.waypoint_timeout_sec
    
    # ========================================================================
    # VELOCITY COMPUTATION
    # ========================================================================

    def _compute_velocity_to_target(self) -> PoseVelocityWithFingerVelocity:
        """
        Compute velocity command to move toward current waypoint target.
        
        Uses proportional control with speed scaling:
        - Direction: unit vector toward target
        - Magnitude: scaled by discrete_motion_speed parameter
        
        Returns:
            Velocity command message
        """

        if not self.controller.current_pose or not self.current_target:
            # Safety: return zero velocity if state is invalid
            msg = PoseVelocityWithFingerVelocity()
            return msg
        
        current_pose = self.controller.current_pose
        target_pose = self.current_target.pose
        

        # =====================================================================
        # LINEAR VELOCITY CONTROL (PID version)
        # =====================================================================
        pos_error = np.array([
            target_pose.position.x - current_pose.pose.position.x,
            target_pose.position.y - current_pose.pose.position.y,
            target_pose.position.z - current_pose.pose.position.z
        ])

        # self.controller.get_logger().info(
        #     f"Position error to waypoint: "
        #     f"x={pos_error[0]*1000:.0f}mm, "
        #     f"y={pos_error[1]*1000:.0f}mm, "
        #     f"z={pos_error[2]*1000:.0f}mm"
        # )

         # Compute distance to target
        
        distance = np.linalg.norm(pos_error)
        
        if distance > 0.001:  # Avoid division by zero
            direction = pos_error / distance
            
            # Provisional feed-forward velocity (what we *want* to achieve)
            max_vel = np.array(self.controller.max_linear_velocity)
            target_velocity = direction * max_vel * self.discrete_motion_speed
            
            # Optional: velocity ramping 
            # Ramp down velocity when within 5cm of target
            # ramp_distance = 0.05  # meters
            # if distance < ramp_distance:
            #     ramp_factor = distance / ramp_distance
            #     target_velocity *= max(ramp_factor, 0.2)  # Minimum 20% speed

            pid_correction = self.controller.pid_linear.control_update(
                target_velocity,
                self.controller.current_vel[0:3]     # must contain vx,vy,vz
            )

            target_velocity = pid_correction       # PID output drives robot

        else:
            target_velocity = np.zeros(3)
        
        # ===== Angular Velocity (Orientation Control) =====
        current_rot = self.controller.get_ee_rotation()
        target_rot = Rotation.from_quat([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ])
        
        if current_rot is not None:
            # Use rotation controller to compute angular velocity
            angular_velocity = self.controller.rotation_controller.compute_angular_velocity(
                target_rot, 
                current_rot, 
                pid_controller=None  # Use proportional control only
            )
            
            # Scale by speed parameter
            angular_velocity *= self.discrete_motion_speed
        else:
            angular_velocity = np.zeros(3)
        
        # ===== Pack Into Message =====
        msg = PoseVelocityWithFingerVelocity()
        msg.twist_linear_x = float(target_velocity[0])
        msg.twist_linear_y = float(target_velocity[1])
        msg.twist_linear_z = float(target_velocity[2])
        msg.twist_angular_x = float(angular_velocity[0])
        msg.twist_angular_y = float(angular_velocity[1])
        msg.twist_angular_z = float(angular_velocity[2])
        msg.finger1 = 0.0
        msg.finger2 = 0.0
        msg.finger3 = 0.0
        
        return msg

    # ========================================================================
    # CONTROL COMMANDS (pause/resume/stop)
    # ========================================================================
       
    def pause(self):
        """
        Pause execution, preserving current progress.
        Can be resumed later from the same point.
        """

        if self.internal_state != self.EXECUTING:
            self.controller.get_logger().warn(
                f"Cannot pause from state '{self.internal_state}'"
            )
            return
        
        # Add active time to accumulated buffer
        if self.waypoint_start_time:
            active_time = (self.controller.get_clock().now() - self.waypoint_start_time).nanoseconds / 1e9
            self.waypoint_elapsed_accumulated += active_time
            self.waypoint_start_time = None  # freeze timer
            self.controller.get_logger().info(
                f"Paused waypoint timer at {self.waypoint_elapsed_accumulated:.2f}s"
            )

        self.internal_state = self.PAUSED
        self.controller.publish_zero_velocity()
        
        self.controller.get_logger().info(
            f"Waypoint execution PAUSED at waypoint "
            f"{self.current_waypoint_index + 1}/{self.total_waypoints}"
        )

        # Send sound
        # self.status_message_pub.publish(str_msg("Waypoint execution PAUSED"))
        
    def resume(self):
        """
        Resume execution from paused state.
        Continues from wherever it was paused (even mid-waypoint).
        """
        if self.internal_state != self.PAUSED:
            self.controller.get_logger().warn(
                f"Cannot resume from state '{self.internal_state}'"
            )
            return
        
        self.internal_state = self.EXECUTING
        

        # Resume timing from current moment
        self.waypoint_start_time = self.controller.get_clock().now()

        self.controller.get_logger().info(
            f"Waypoint execution RESUMED at waypoint "
            f"{self.current_waypoint_index + 1}/{self.total_waypoints}"
        )

        # Send sound
        # self.status_message_pub.publish(str_msg("Waypoint execution RESUMED"))

    def stop(self):
        """
        Stop execution and clear waypoint queue.
        Returns to IDLE state, allowing new waypoints to be loaded.
        """
        if self.internal_state == self.IDLE:
            self.controller.get_logger().info("Already in IDLE state")
            return
        
        # Clear all state
        self.waypoint_queue.clear()
        self.current_target = None
        self.current_waypoint_index = 0
        self.total_waypoints = 0
        self.internal_state = self.IDLE
        
        # Stop motion
        self.controller.publish_zero_velocity()
        
        self.controller.get_logger().info(
            "Waypoint execution STOPPED and cleared. Ready for new waypoints."
        )

         # Send sound
        # self.status_message_pub.publish(str_msg("Waypoint execution STOPPED"))

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
        
        # State
        self.current_pose = None
        self.current_vel = None
        self.time_prev = self.get_clock().now()
        self.latest_velocity_cmd = None

        self.current_finger_pose = np.array([0.0, 0.0, 0.0])

        # Behavior

This is a significant improvement. By making the rotation input relative to a specific reference frame (like the camera/user view or the end-effector tip), the control becomes much more intuitive.
Here is the implementation. I have addressed the Mathematical issues from the previous review (Gimbal lock, wrapping bugs) and added the Frame Reference logic.
Key Changes
RotationController: Now accepts a basis_rotation (the rotation of the input frame relative to the robot base). It rotates the user's input vector into the Base Frame before applying it.
ContinuousTeleopBehavior: Now extracts the frame_id from the incoming Twist message, looks up that frame's rotation via TF, and passes it to the controller.
Math Fixes: Replaced Euler addition with Quaternion multiplication and fixed the "wrap-around" smoothing bug using slerp.
Here is the complete, refactored code.
code
Python
#!/usr/bin/env python3
"""
Refactored Jaco Robot Controller with Reference Frame Support
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from kinova_msgs.msg import PoseVelocityWithFingerVelocity, FingerPosition
from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped, Point, TransformStamped
from std_msgs.msg import String as str_msg
from nav_msgs.msg import Path

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

import traceback
from abc import ABC, abstractmethod
from collections import deque
from typing import Optional, Tuple, Dict
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

# ============================================================================
# CONSTANTS
# ============================================================================

REFRESH_RATE = 100.0
# Define start rotation (using Quaternions internally is safer)
START_ROTATION = Rotation.from_euler('xyz', [-180-20, 0-5, 180-10], degrees=True)
MAX_LINEAR_VELOCITY = (0.1, 0.06, 0.08)
MAX_ANGULAR_VELOCITY = 2.0
MAX_FINGER_VELOCITY = 2000.0
ROBOT_BASE_FRAME = "j2n6s300_link_base"

# ============================================================================
# UTILITIES
# ============================================================================

class PIDController:
    """Standard PID Controller"""
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0, 
                 refresh_rate: float = 1.0, length: int = 3, anti_windup: bool = True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.refresh_rate = refresh_rate
        self.length = length
        self.prev_error = np.zeros(self.length)
        self.integral_error = np.zeros(self.length)
        self.anti_windup = anti_windup

    def control_update(self, target_vel: np.ndarray, current_vel: np.ndarray) -> np.ndarray:
        if target_vel is None or current_vel is None:
            return np.zeros(self.length)

        current_error = target_vel - current_vel
        self.integral_error += current_error * (1.0 / self.refresh_rate)
        
        if self.anti_windup:
            self.integral_error = np.clip(self.integral_error, -1.0, 1.0)
        
        derivative_error = (current_error - self.prev_error) * self.refresh_rate
        self.prev_error = current_error

        return self.kp * current_error + self.ki * self.integral_error + self.kd * derivative_error

    def reset(self):
        self.prev_error = np.zeros(self.length)
        self.integral_error = np.zeros(self.length)

class VelocityIntegrator:
    """Handles smooth acceleration/deceleration"""
    def __init__(self, max_velocity: Tuple[float, float, float], refresh_rate: float = 100.0):
        self.max_velocity = np.array(max_velocity)
        self.prev_velocity = np.zeros(3)
        self.forward_acceleration = np.array([0.015, 0.009, 0.012]) 
        self.brake_acceleration = 0.8
        self.min_speed = 0.027
        self.deadzone = 0.2

    def update(self, input_vector: np.ndarray, dt: float) -> np.ndarray:
        velocity = self.prev_velocity.copy()
        hard_brake_threshold = 0.02
        
        for idx, input_dir in enumerate(input_vector):
            is_hard_brake = ((input_dir * velocity[idx]) < -hard_brake_threshold)
            is_deadzone = abs(input_dir) < self.deadzone
            movement_dir = 1 if velocity[idx] > 0 else -1

            if is_deadzone or is_hard_brake:
                velocity[idx] -= movement_dir * self.brake_acceleration * dt
                if abs(velocity[idx]) < self.min_speed * 1.5:
                    velocity[idx] = 0.0
            else:
                velocity[idx] += input_dir * self.forward_acceleration[idx] * dt
                if abs(velocity[idx]) < self.min_speed:
                    velocity[idx] = self.min_speed * np.sign(input_dir)
            
            velocity[idx] = np.clip(velocity[idx], -self.max_velocity[idx], self.max_velocity[idx])
        
        self.prev_velocity = velocity
        return velocity

    def reset(self):
        self.prev_velocity = np.zeros(3)

class RollingAverageFilter:
    def __init__(self, window_size: int):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
    def update(self, value: float) -> float:
        self.values.append(value)
        return sum(self.values) / len(self.values)

# ============================================================================
# ROTATION CONTROLLER (REFACTORED)
# ============================================================================

class RotationController:
    """
    Manages rotation control with frame-relative inputs and robust quaternion math.
    """
    
    def __init__(self, start_rotation: Rotation, quantization_degrees: float = 30.0,
                 max_angular_velocity: float = 2.0, refresh_rate: float = 100.0):
        self.start_rotation = start_rotation
        self.quantization_degrees = quantization_degrees
        self.max_angular_velocity = max_angular_velocity
        self.refresh_rate = refresh_rate
        
        # Internal state is maintained as a Rotation object (Quaternions), not Euler angles
        self.cumulative_rotation = START_ROTATION

    def update_target_rotation_from_input(self, 
                                          orientation_input: np.ndarray, 
                                          dt: float,
                                          basis_rotation: Optional[Rotation] = None) -> Rotation:
        """
        Updates the internal target rotation based on user input relative to a specific frame.
        
        Args:
            orientation_input: [rx, ry, rz] velocity request from joystick (approx -1 to 1)
            dt: Time delta
            basis_rotation: The rotation of the frame the input is defined in, relative to robot base.
                            If None, assumes input is already in Base frame.
        """
        speed_factor = 2.0 # Rad/s max speed approx
        
        # 1. Deadband
        if np.linalg.norm(orientation_input) < 0.05:
            user_active = False
            orientation_change_vec = np.zeros(3)
        else:
            user_active = True
            # Raw input vector
            orientation_change_vec = orientation_input * speed_factor * dt

        # 2. Transform Input to Base Frame
        # If the user pushes "Forward" (X) in the Camera Frame, and the Camera is rotated 90 deg,
        # we need to apply that rotation to the vector so the robot moves correctly in Base frame.
        if basis_rotation is not None and user_active:
            # Rotate the input vector by the basis rotation
            orientation_change_vec = basis_rotation.apply(orientation_change_vec)

        # 3. Apply Integration (Quaternion Multiplication)
        if user_active:
            # Create a small rotation from the vector
            # Magnitude of vector = angle in radians, Direction = axis
            rot_delta = Rotation.from_rotvec(orientation_change_vec)
            # Apply: New = Delta * Old (Intrinsic) or Old * Delta (Extrinsic)
            # Usually for "Base Frame" accumulation, we want:
            self.cumulative_rotation = rot_delta * self.cumulative_rotation

        # 4. Quantization / Snapping
        # Convert to Euler for grid snapping, then back
        # Note: We snap in the BASE frame (xyz), which is usually what feels "straight"
        current_euler = self.cumulative_rotation.as_euler('xyz', degrees=True)
        q = self.quantization_degrees
        target_euler_snapped = np.round(current_euler / q) * q
        target_rot_snapped = Rotation.from_euler('xyz', target_euler_snapped, degrees=True)

        # 5. Smoothing / Settling
        if not user_active:
            # Use SLERP (Spherical Linear Interpolation) for smooth rotation
            # This fixes the wrap-around bug (359 -> 1 degree)
            settle_gain = 6.0
            alpha = 1 - np.exp(-settle_gain * dt)
            
            # Interpolate between current and snapped
            key_times = [0, 1]
            key_rots = Rotation.concatenate([self.cumulative_rotation, target_rot_snapped])
            slerp = Slerp(key_times, key_rots)
            
            # Update current towards target
            self.cumulative_rotation = slerp(alpha)

        return self.cumulative_rotation

    def compute_angular_velocity(self, target_rotation: Rotation, 
                        current_rotation: Rotation, 
                        pid_controller: Optional[PIDController] = None) -> np.ndarray:
        """
        Compute angular velocity [rx, ry, rz] to reach target.
        """
        # 1. Calculate error: Target * Inverse(Current) -> Difference in Global Frame
        # Or: Current.inv * Target -> Difference in Local Frame
        # We usually want velocity commands in the End Effector (Local) or Base frame depending on robot driver.
        # Assuming Jaco accepts 'PoseVelocity' in Base frame (usually), but check driver!
        # If driver takes twists in Base Frame:
        error_rot = target_rotation * current_rotation.inv()
        
        # 2. Convert to Rotation Vector (Axis-Angle)
        rot_vec = error_rot.as_rotvec() # Vector direction = axis, Magnitude = angle (rads)
        
        # 3. Clamp Magnitude (Safety)
        mag = np.linalg.norm(rot_vec)
        MAX_ERROR_RAD = 1.0 # 57 degrees
        if mag > MAX_ERROR_RAD:
            rot_vec = rot_vec * (MAX_ERROR_RAD / mag)
            
        # 4. PID or Proportional
        if pid_controller:
             # Assuming PID controller keeps track of dt internally
            return pid_controller.control_update(rot_vec, np.zeros(3))
        else:
            return rot_vec * self.max_angular_velocity

    def reset(self):
        self.cumulative_rotation = START_ROTATION

# ============================================================================
# BEHAVIORS
# ============================================================================

class ControlBehavior(ABC):
    def __init__(self, controller: 'RobotController'):
        self.controller = controller
    
    @abstractmethod
    def process_velocity_command(self, twist: TwistStamped) -> Optional[PoseVelocityWithFingerVelocity]:
        pass
    
    @abstractmethod
    def on_enter(self):
        pass
    
    @abstractmethod
    def on_exit(self):
        pass

class ContinuousTeleopBehavior(ControlBehavior):
    """
    Standard Teleop that respects the Frame ID of the input command.
    """
    def __init__(self, controller: 'RobotController'):
        super().__init__(controller)
    
    def process_velocity_command(self, twist: TwistStamped) -> Optional[PoseVelocityWithFingerVelocity]:
        if not self.controller.current_pose:
            return None
        
        # 1. Handle Frame Transforms for Linear Velocity
        # The Twist message tells us what frame the input is in (e.g., "head_camera", "base_link")
        input_frame = twist.header.frame_id
        
        # Extract Linear
        target_linear_vel = np.array([twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z])
        
        # If input is not in base frame, rotate the linear velocity vector
        if input_frame and input_frame != ROBOT_BASE_FRAME:
            target_linear_vel = self.controller.transform_vector(
                target_linear_vel, input_frame, ROBOT_BASE_FRAME
            )

        # Apply Smoothing & PID (Linear)
        target_linear_vel = self.controller.vel_integrator.update(target_linear_vel, dt=1.0/REFRESH_RATE)
        if self.controller.current_vel is not None:
             # Basic Feedforward + PID correction
             pid_out = self.controller.pid_linear.control_update(target_linear_vel, self.controller.current_vel[0:3])
             target_linear_vel = target_linear_vel + pid_out

        # 2. Handle Rotation Control
        target_angular_input = np.array([twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z])
        
        # Look up the rotation of the input frame relative to base
        # This allows "Up" on joystick to mean "Up" in camera view, etc.
        basis_rotation = self.controller.get_frame_rotation(input_frame, ROBOT_BASE_FRAME)

        # Update the Target Orientation state
        rotation_target = self.controller.rotation_controller.update_target_rotation_from_input(
            target_angular_input,
            dt=1.0/REFRESH_RATE,
            basis_rotation=basis_rotation
        )
        
        # Compute Velocity to reach that target
        current_ee_rot = self.controller.get_ee_rotation()
        if current_ee_rot:
            target_angular_vel = self.controller.rotation_controller.compute_angular_velocity(
                rotation_target,
                current_ee_rot,
                pid_controller=None # or self.controller.pid_angular
            )
        else:
            target_angular_vel = np.zeros(3)

        # Construct Message
        msg = PoseVelocityWithFingerVelocity()
        msg.twist_linear_x, msg.twist_linear_y, msg.twist_linear_z = target_linear_vel
        msg.twist_angular_x, msg.twist_angular_y, msg.twist_angular_z = target_angular_vel
        return msg

    def on_enter(self):
        self.controller.get_logger().info("Entered Continuous Teleop")
        self.controller.pid_linear.reset()
        self.controller.pid_angular.reset()

    def on_exit(self):
        self.controller.publish_zero_velocity()

# ============================================================================
# DISCRETE BEHAVIOR (Shortened for brevity, use previous implementation)
# ============================================================================
class DiscreteTeleopBehavior(ControlBehavior):
    # ... (Keep your previous implementation of DiscreteTeleopBehavior here) ...
    # ... It was logically sound in the previous step ...
    def process_velocity_command(self, twist: TwistStamped):
        return None 
    def on_enter(self): pass
    def on_exit(self): pass
    def load_waypoints(self, path): pass
    def pause(self): pass
    def resume(self): pass
    def stop(self): pass

# ============================================================================
# MAIN CONTROLLER
# ============================================================================

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Params
        self.max_linear_velocity = self.declare_parameter("max_linear_velocity", MAX_LINEAR_VELOCITY).value
        self.quantization_degrees = self.declare_parameter("quantisation_degrees", 30.0).value
        
        # Components
        self.pid_linear = PIDController(kp=0.8, ki=0.1, length=3)
        self.pid_angular = PIDController(kp=2.0, ki=0.5, length=3) # PID for angular velocity
        self.vel_filter = RollingAverageFilter(window_size=5)
        self.vel_integrator = VelocityIntegrator(self.max_linear_velocity)
        
        self.rotation_controller = RotationController(
            start_rotation=START_ROTATION,
            quantization_degrees=self.quantization_degrees
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Pub/Sub
        self.vel_pub = self.create_publisher(PoseVelocityWithFingerVelocity, '/j2n6s300_driver/in/cartesian_velocity_with_finger_velocity', 1)
        self.velocity_sub = self.create_subscription(TwistStamped, '/teleop/cartesian_velocity', self.velocity_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_pose', self.update_current_pose, 10)
        
        # State
        self.current_pose = None
        self.current_vel = None
        self.time_prev = self.get_clock().now()
        self.latest_velocity_cmd = None
        
        # Behavior
        self.behaviors = {
            "translation": ContinuousTeleopBehavior(self),
            "discrete": DiscreteTeleopBehavior(self),
            # Add system, etc.
        }
        # self.current_mode = "translation"
        self.current_behavior = self.behaviors["translation"]
        
        # Initialize ROS interfaces
        self._init_publishers()
        self._init_subscribers()
        self._init_tf()
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / REFRESH_RATE, self.control_tick)
        
        self.get_logger().info("Robot Controller initialized. Let's Rock")
    
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
        
        # NEW: Discrete waypoint execution parameters
        self.discrete_motion_speed = self.declare_parameter(
            "discrete_motion_speed", 1.5).value  # 150% of max velocity
        
        self.waypoint_position_threshold = self.declare_parameter(
            "waypoint_position_threshold", 0.005).value  # 5mm
        
        self.waypoint_orientation_threshold_deg = self.declare_parameter(
            "waypoint_orientation_threshold_deg", 5.0).value  # 5 degrees
        
        self.waypoint_timeout_sec = self.declare_parameter(
            "waypoint_timeout_sec", 20.0).value  # 20 seconds
        

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

        # Log new parameters
        self.get_logger().info(f"Discrete motion speed: {self.discrete_motion_speed*100:.0f}%")
        self.get_logger().info(f"Waypoint position threshold: {self.waypoint_position_threshold*1000:.1f}mm")
        self.get_logger().info(f"Waypoint orientation threshold: {self.waypoint_orientation_threshold_deg:.1f}deg")
        self.get_logger().info(f"Waypoint timeout: {self.waypoint_timeout_sec:.1f}s")

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
        # TODO: Change to Exponential Moving Average?
        # self.vel_filter = ExponentialMovingAverageFilter(alpha=0.1)

        # Velocity integrator
        self.vel_integrator = VelocityIntegrator(max_velocity=self.max_linear_velocity)
        
        # Rotation controller
        self.rotation_controller = RotationController(
            start_rotation=START_ROTATION,
            quantization_degrees=self.quantization_degrees,
            max_angular_velocity=self.max_angular_velocity,
            # refresh_rate=REFRESH_RATE
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
        # Used for sound
        self.status_message_pub = self.create_publisher(str_msg, '/controller/controller_status_info', 1)

        self.test_target_vel_pub = self.create_publisher(Point, '/test/requested_vel_pub', 1)
        self.test_pid_vel_pub = self.create_publisher(Point, '/test/pid_target_vel_pub', 1)
        self.test_measured_vel_pub = self.create_publisher(Point, '/test/measured_vel_pub', 1)
        
        # Haptics
        # self.haptics_action_pub = self.create_publisher(str_msg, "/haptic_feedback_robot_string", 1)
    
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
        self.waypoint_sub = self.create_subscription(
            Path,
            '/teleop/waypoint_path',
            self.waypoint_path_callback,
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
        
        self.finger_sub = self.create_subscription(
            FingerPosition,
            '/j2n6s300_driver/out/finger_position',
            self.update_current_finger_pose,
            10
        )

        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/j2n6s300_driver/out/tool_wrench',
            self.update_current_force,
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
    # SUBSCRIBER CALLBACKS
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
    
    def waypoint_path_callback(self, msg: PoseStamped):
        """
        Handle incoming waypoint path messages.
        Forwards to discrete behavior if in discrete mode.
        """    
        if self.current_mode != "discrete":
            self.get_logger().warn(
                f"Waypoint path received but current mode is '{self.current_mode}'. "
                "Switch to discrete mode first."
            )
            return
        
        # Forward to discrete behavior
        success = self.behaviors["discrete"].load_waypoints(msg)
        
        if not success:
            self.get_logger().error("Failed to load waypoint path")
                    
    def system_callback(self, msg: str_msg):
        """Handle system commands."""
        cmd = msg.data
        self.get_logger().info(f"System command received: {cmd}")

        if cmd == "pause_waypoints":
            if self.current_mode == "discrete":
                self.behaviors["discrete"].pause()
            else:
                self.get_logger().warn("pause_waypoints only works in discrete mode")
    
        elif cmd == "resume_waypoints":
            if self.current_mode == "discrete":
                self.behaviors["discrete"].resume()
            else:
                self.get_logger().warn("resume_waypoints only works in discrete mode")
    
        elif cmd == "stop_waypoints":
            if self.current_mode == "discrete":
                self.behaviors["discrete"].stop()
            else:
                self.get_logger().warn("stop_waypoints only works in discrete mode")
        
        elif cmd == "reset_pose":
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
    
    def send_info_message(self, info: str):
        """Send informational status message."""
        msg = str_msg()
        msg.data = info
        self.status_message_pub.publish(msg)

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