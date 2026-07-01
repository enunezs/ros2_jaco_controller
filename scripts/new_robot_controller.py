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
from std_msgs.msg import Float64MultiArray

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
# START_ROTATION = Rotation.from_euler('xyz', [-180-20, 0-5, 180-10], degrees=True)
START_ROTATION = Rotation.from_euler('xyz', [-180, 0-5, 180], degrees=True)
MAX_LINEAR_VELOCITY = (0.1, 0.06, 0.08)
MAX_ANGULAR_VELOCITY = 2.0
MAX_FINGER_VELOCITY = 2000.0
ROBOT_BASE_FRAME = "j2n6s300_link_base"

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
        self.prev_vel = 0.0

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

        # 1. If target is zero, bleed the integral error to prevent drift
        if np.linalg.norm(target_vel) < 1e-4:
            self.integral_error *= 0.90  # Quickly decay the integral 
            return np.zeros(self.length) # Return absolute zero

        # Error calculation
        current_error = target_vel - current_vel
        current_error[np.abs(current_error) < 1e-4] = 0.0

        # Integral term with anti-windup
        self.integral_error += current_error * (1.0 / self.refresh_rate)
        if self.anti_windup:
            self.integral_error = np.clip(self.integral_error, -1.0, 1.0)
        
        # Derivative term
        dx = (current_vel - self.prev_vel) * self.refresh_rate
        self.derivative_error = -dx # Sign change
        self.prev_vel = current_vel
        
        # Better implementation: Derivative on PV (Measurement)
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
        # self.values.append(value)
        # return sum(self.values) / len(self.values)

        self.values.append(np.array(value, dtype=float))
        return np.mean(self.values, axis=0)

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
        # self.forward_acceleration = np.array([0.0, 0.0, 0.0])/3
        self.forward_acceleration = np.array([0.015, 0.009, 0.02])/3
        self.brake_acceleration = 0.8
        self.min_speed = [0.0135, 0.0135, 0.014]
        # self.min_speed = [0.008, 0.014, 0.01]
        self.deadzone = 0.05 # Input deadzone (0 to 1)

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
                
                if abs(velocity[idx]) < self.min_speed[idx] * 1.5:
                    velocity[idx] = 0.0
            
            # Acceleration
            else:
                velocity[idx] += input_dir * self.forward_acceleration[idx] * dt 
                
                # Fast start from rest
                if abs(velocity[idx]) < self.min_speed[idx]:
                    velocity[idx] = self.min_speed[idx] * np.sign(input_dir)
            
            # Clamp to max velocity
            velocity[idx] = np.clip(velocity[idx], -self.max_velocity[idx], self.max_velocity[idx])
        
        self.prev_velocity = velocity
        return velocity

    def reset(self):
        """Reset to zero velocity."""
        self.prev_velocity = np.zeros(3)

class RotationController:
    """
    Manages the 'User Controlled' part of the rotation chain.
    Handles: 
    - local integration
    - quantization / grid-snapping
    - friction feel.
    - velocity computation.
    """
    
    def __init__(self, 
                 start_rotation: Rotation, 
                 quantization_degrees: float = 45.0,
                 max_angular_velocity: float = 1.0, # Rad/s max speed approx
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

        # SETTINGS
        self.discretize_enabled = False  # Toggle this to False for continuous mode
        self.use_virtual_gimbal = False  # True = Virtual Gimbal, False = Local Quaternion

        # PERSISTENT STATE
        self.cumulative_rotation : Rotation = Rotation.from_quat([0,0,0,1]) # Start from no rotation offset
        self.accumulated_euler = np.zeros(3) 

        self.rotation_reference_frame_buffer : Optional[Rotation] = None
        self.reference_frame = "j2n6s300_link_base" #-> needs a setter?


    def update_target_rotation_from_input(self, 
                                          orientation_input: np.ndarray, 
                                          dt: float,
                                          rotation_reference_frame: Optional[Rotation] = None) -> Rotation:
        
        """
        Updates the internal target rotation based on user input relative to a specific frame.
        
        Bi-stable rotation update:
        - When user is moving -> follow velocity input
        - When user stops -> settle to nearest quantized zone
        
        Args:
            orientation_input: [rx, ry, rz] velocity request from joystick (approx -1 to 1)
            dt: Time delta
            rotation_reference_frame: The rotation of the frame the input is defined in, relative to robot base.
                            If None, assumes input is already in Base frame.
        """

        # TODO: Not doing anything...
        if rotation_reference_frame:
            self.rotation_reference_frame_buffer = rotation_reference_frame
      

        # --- Initialization for Virtual Gimbal ---
        self.accumulated_euler = self.cumulative_rotation.as_euler('xyz', degrees=True)

        # ---------------------------------------------------------
        # 1. Input deadband + user activity detection 
        # ---------------------------------------------------------
        user_active = np.linalg.norm(orientation_input) >= 0.05

        # TODO: Expose speed factor as parameter
        speed_factor = 0.15 # Rad/s max speed approx
        settle_gain = 6.0

        min_breakout_speed = 0.4
        max_falling_speed  = 1.2

        # ---------------------------------------------------------
        # 2. Update Cumulative Rotation (Local Space)
        # ---------------------------------------------------------
        if self.use_virtual_gimbal:
            # === MODE A: VIRTUAL GIMBAL (Float Accumulation) ===
            if user_active:
                if self.discretize_enabled:
                    # --- MAGNETIC GRID GAIN ---

                    # 1. Calculate gain axis-by-axis
                    # distance factor 0 at snap (N*45), 1 at midpoint
                    dist_factors = np.abs(np.sin(np.pi * (self.accumulated_euler / self.quantization_degrees)))
                    # 2. Apply dynamic gain per axis
                    dynamic_gains = min_breakout_speed + (max_falling_speed - min_breakout_speed) * dist_factors
                
                else:
                    # --- CONTINUOUS MODE GAIN ---
                    dynamic_gains = 1.0

                # 3. Integrate floats (convert input rad/s to deg/s for the floats)
                input_degrees_delta = np.degrees(orientation_input * speed_factor * dt)
                self.accumulated_euler += input_degrees_delta * dynamic_gains
                # print(f"Accumulated Euler: {self.accumulated_euler}, Input: {orientation_input}") #, Dynamic Gains: {dynamic_gains}")
            else:
                if self.discretize_enabled:
                    # 4. Snap floats to grid axis-by-axis
                    target_euler_snapped = np.round(self.accumulated_euler / self.quantization_degrees) * self.quantization_degrees
                    
                    # Smoothly lerp the floats
                    alpha = 1 - np.exp(-settle_gain * dt)
                    self.accumulated_euler += alpha * (target_euler_snapped - self.accumulated_euler)

            # Keep angles within [-180, 180] to prevent overflow
            # self.accumulated_euler = (self.accumulated_euler + 180) % 360 - 180
            
            # Rebuild the rotation object from the gimbal setting
            self.cumulative_rotation = Rotation.from_euler('xyz', self.accumulated_euler, degrees=True)


        else:
            # === MODE B: QUATERNION INTEGRATION ===
            if user_active:
                if self.discretize_enabled:
                    ### Apply integration and magnetic gain
                    # 1. Get current local angles
                    curr_euler = self.cumulative_rotation.as_euler('xyz', degrees=True)
                    # 2. Calculate distance factor using a sine wave pattern. sin(0) = 0 (slow at snap), sin(90) = 1 (fast at midpoint)
                    dist_factors = np.abs(np.sin(np.pi * (curr_euler / self.quantization_degrees))) # 45 deg by default
                    # 3. Lerp between min and max based on distance from snap point
                    dynamic_gain = min_breakout_speed + (max_falling_speed - min_breakout_speed) * dist_factors
                else:
                    dynamic_gain = 1

                orientation_change_vec = orientation_input * dynamic_gain * speed_factor * dt
                ### 5. INTEGRATE: Apply input to current cumulative rotation
                rot_delta = Rotation.from_euler('xyz', orientation_change_vec, degrees=False)
                # Local Space Rotation
                self.cumulative_rotation = self.cumulative_rotation * rot_delta
                # A*B you can think of it as applying A as a global rotation to B. Or as applying B as a local rotation to A(*).
                #  A*B is B applied on local A, or global A applied to B, 
                # For reference only, global Space Rotation
                # self.cumulative_rotation = rot_delta * self.cumulative_rotation
                
            else:
                if self.discretize_enabled:
                    ### Settle towards nearest grid in LOCAL space ###
                    
                    # A. Calculate the Snap Target (Local)
                    current_euler = self.cumulative_rotation.as_euler('xyz', degrees=True)
                    target_euler_snapped = np.round(current_euler / self.quantization_degrees) * self.quantization_degrees
                    target_rot_snapped = Rotation.from_euler('xyz', target_euler_snapped, degrees=True)

                    # B. SLERP (Smooth) the Cumulative Rotation towards Snap Target
                    key_rots = Rotation.concatenate([self.cumulative_rotation, target_rot_snapped])
                    key_times = [0, 1]
                    slerp = Slerp(key_times, key_rots)
                    
                    # Update the persistent cumulative rotation
                    settle_gain = 6.0
                    alpha = 1 - np.exp(-settle_gain * dt) # Could be replaced by a constant...
            
                    self.cumulative_rotation = slerp(alpha)

        # ---------------------------------------------------------
        # 3. Apply Reference Frame (Global Space)
        # ---------------------------------------------------------
        # Now we apply the reference frame to the (potentially smoothed) cumulative value.
        # Since user_active=False implies no new reference frame input, we rely on the buffer.
        
        # Determine which reference frame to use
        active_reference_frame = self.rotation_reference_frame_buffer
        
        if active_reference_frame is not None:
            # Reference Frame * Offset * Smoothed_Local
            target_rot_composite = self.cumulative_rotation * active_reference_frame
        else:
            # Fallback if no reference frame ever received
            target_rot_composite = self.cumulative_rotation

        return target_rot_composite

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
        rot_vec = error_rot.as_rotvec() * self.max_angular_velocity # Vector direction = axis, Magnitude = angle (rads)
        
        # 3. Clamp Magnitude (Safety)
        mag = np.linalg.norm(rot_vec)
        MAX_ERROR_RAD = 1.0 # 57 degrees
        if mag > MAX_ERROR_RAD:
            rot_vec = rot_vec * (MAX_ERROR_RAD / mag)
            
        # 4. PID or Proportional
        if pid_controller:
             # Assuming PID controller keeps track of dt internally
            return pid_controller.control_update(rot_vec , np.zeros(3))
        else:
            return rot_vec



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
        self.cumulative_rotation = START_ROTATION

    # def reset(self):
    #     """Reset cumulative rotation."""
    #     # self.cumulative_rotation = np.zeros(3)
    #     self.cumulative_rotation = Rotation.from_quat([0, 0, 0, 1])

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
        
        # dt = 1.0 / REFRESH_RATE
        # input_frame = twist.header.frame_id or ROBOT_BASE_FRAME

        # 1. Compute Linear Velocity
        target_linear_vel = np.array([twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z])
        # Deadzone
        if np.linalg.norm(target_linear_vel) < 0.01: # 1% threshold
            target_linear_vel = np.zeros(3)
        
        # 2. Determine Reference Frame and User Compensation

        ######################################################
        ### 1. Handle Frame Transforms for Linear Velocity ###
        ######################################################

        # The Twist message tells us what frame the input is in (e.g., "head_camera", "base_link")
        input_frame = twist.header.frame_id        

        # 2B. We apply an offset to account for the robot weird shape
        ROTATION_OFFSET = Rotation.from_euler('xyz', [0-15, 180, -12], degrees=True) # Minor offset to account for only two fingers

        # If input is not in base frame, rotate the linear velocity vector
        if input_frame and input_frame != ROBOT_BASE_FRAME:
            target_linear_vel = self.controller.transform_vector(
                target_linear_vel, input_frame, ROBOT_BASE_FRAME
            )
            target_linear_vel = ROTATION_OFFSET.inv().apply(target_linear_vel)

        # Apply Smoothing & PID (Linear)
        target_linear_vel = self.controller.vel_integrator.update(target_linear_vel, dt=1.0/REFRESH_RATE)
        if self.controller.current_vel is not None:
             # Basic Feedforward + PID correction
             pid_out = self.controller.pid_linear.control_update(target_linear_vel, self.controller.current_vel[0:3])
             target_linear_vel = target_linear_vel + pid_out


        ##################################
        ### 2. Handle Rotation Control ###
        ##################################

        # 2A. We use the rotation reference frame as the baseline for starting
        # This allows "Up" on joystick to mean "Up" in camera view
        # print(f"Rotation reference frame is: {rotation_reference_frame}")

        rotation_reference_frame  = self.controller.get_frame_rotation(input_frame, ROBOT_BASE_FRAME)
        # rotation_reference_frame = self.controller.get_ee_rotation()
        if rotation_reference_frame is None:
            rotation_reference_frame = Rotation.from_quat([0,0,0,1])
        else:
            pass
            # print(f"Found ref frame {rotation_reference_frame.as_euler('xyz', degrees=True)}")

        # Force for now
        rotation_reference_frame = Rotation.from_quat([0,0,0,1])


        # 2. Final rotation
        # ROTATION_OFFSET = Rotation.from_euler('xyz', [0, 180, 0], degrees=True) # Minor offset to account for only two fingers
        rotation_reference_frame = rotation_reference_frame * ROTATION_OFFSET
        # rotation_reference_frame = rotation_reference_frame


        # =========================================================
        # DEBUG: Visualize the Rotation Reference Frame
        # =========================================================
        if self.controller.current_pose and rotation_reference_frame is not None:
            # 1. Create a visual offset so it doesn't clip inside the robot
            #    Let's put it 10cm above the current end effector
            debug_xyz = [
                self.controller.current_pose.pose.position.x+ 0.005,
                self.controller.current_pose.pose.position.y+ 0.005,
                self.controller.current_pose.pose.position.z + 0.005
            ]

            # 2. Publish the TF
            #    Name the frame "debug_rotation_reference_frame"
            self.controller.publish_rotation_target(
                rotation=rotation_reference_frame,
                frame_id=ROBOT_BASE_FRAME,
                child_frame_id="01_baseline_rotation", 
                visual_offset=debug_xyz
            )
        # =========================================================

        # 2B. User Compensation (Optional)
        # TODO: Forcing for now
        input_frame = "camera_optical_frame"
        target_frame = "bt_UpHybridRe"

        user_compensation = False
        if user_compensation:
            user_tracking_rot = self.controller.get_tracking_compensation(camera_frame=input_frame, target_frame=target_frame)
        else:
            user_tracking_rot : Rotation =  Rotation.from_quat([0,0,0,1])
            
        compensated_rotation_reference_frame = rotation_reference_frame * user_tracking_rot

        # =========================================================
        # DEBUG: Visualize the Rotation Reference Frame
        # =========================================================
        if self.controller.current_pose and rotation_reference_frame is not None:
            # 1. Create a visual offset so it doesn't clip inside the robot
            #    Let's put it 10cm above the current end effector
            debug_xyz = [
                self.controller.current_pose.pose.position.x+ 0.01,
                self.controller.current_pose.pose.position.y+ 0.01,
                self.controller.current_pose.pose.position.z + 0.01
            ]

            # 2. Publish the TF
            #    Name the frame "debug_rotation_reference_frame"
            self.controller.publish_rotation_target(
                rotation=compensated_rotation_reference_frame,
                frame_id=ROBOT_BASE_FRAME,
                child_frame_id="02_post_user_compensation", 
                visual_offset=debug_xyz
            )
        # =========================================================

        # 2C. Apply user inputs, quantize, and apply offsets
        # Update the Target Orientation state
        target_angular_input = np.array([twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z])

        rotation_target = self.controller.rotation_controller.update_target_rotation_from_input(
            target_angular_input,
            dt=1.0/REFRESH_RATE,
            # rotation_reference_frame=rotation_reference_frame # -> TODO: add rotation frame of the message
            rotation_reference_frame=compensated_rotation_reference_frame # -> TODO: add rotation frame of the message
        )

        # final_rotation_target = rotation_reference_frame * user_tracking_rot * rotation_target
        final_rotation_target = rotation_target

        # Visualize Target
        if self.controller.current_pose:
            # Offset to avoid total overlap
            offset = 0.02
            current_xyz = [
                self.controller.current_pose.pose.position.x + offset,
                self.controller.current_pose.pose.position.y + offset,
                self.controller.current_pose.pose.position.z + offset
            ]

            # Publish the TF
            self.controller.publish_rotation_target(
                rotation=rotation_target,
                frame_id=ROBOT_BASE_FRAME,        # Parent frame (usually link_base)
                child_frame_id="03_user_target_orientation", 
                visual_offset=current_xyz
            )

        # Compute Base Angular Velocity from Rotation Controller
        current_ee_rot = self.controller.get_ee_rotation()
        if current_ee_rot:
            target_angular_vel = self.controller.rotation_controller.compute_angular_velocity(
                final_rotation_target,
                current_ee_rot,
                pid_controller= self.controller.pid_angular
            )
        else:
            target_angular_vel = np.zeros(3)


         # Construct Message
        msg = PoseVelocityWithFingerVelocity()
        msg.twist_linear_x, msg.twist_linear_y, msg.twist_linear_z = target_linear_vel
        msg.twist_angular_x, msg.twist_angular_y, msg.twist_angular_z = target_angular_vel

        # TODO Finger!
        # msg.finger_velocity1, msg.finger_velocity2, msg.finger_velocity3 = [0.0, 0.0, 0.0]
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
        self.finger_queue = deque() # Parallel queue for fingers

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
            self.controller, 'discrete_motion_speed', 0.4
        )
        # Completion thresholds
        self.position_threshold = getattr(
            self.controller, 'waypoint_position_threshold', 0.01  # 10mm
        )
        self.orientation_threshold_deg = getattr(
            self.controller, 'waypoint_orientation_threshold_deg', 10.0  # 5 degrees
        )
        
        # Safety timeout per waypoint
        self.waypoint_timeout_sec = getattr(
            self.controller, 'waypoint_timeout_sec', 10.0  # 8 seco        
        )
        # Inside DiscreteTeleopBehavior._load_parameters
        self.finger_tolerance = getattr(
            self.controller, 'waypoint_finger_tolerance', 100.0
        )

        # Inside RobotController._load_parameters
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
    
        self.controller.get_logger().info(f"Entered discrete mode")
        self.controller.pid_linear.reset()
        self.controller.pid_angular.reset()

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
    
    def load_waypoints(self, path: Path, finger_data) -> bool:
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
        # Re-group the flattened [f1,f2,f3, f1,f2,f3] into [[f1,f2,f3], [f1,f2,f3]]
        self.finger_queue = deque([finger_data[i:i+3] for i in range(0, len(finger_data), 3)]) if finger_data else deque()
        
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
        if self.finger_queue:
            self.finger_queue.popleft() # Keep them in sync

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
        Check if current waypoint has been reached (Position, Orientation, AND Fingers).
        
        Requires both position and orientation to be within thresholds.
        
        Returns:
            True if waypoint reached, False otherwise
        """

        if not self.controller.current_pose or not self.current_target:
            self.controller.get_logger().warn("Cannot check waypoint - missing current pose or target")
            return False
        
        current_pose = self.controller.current_pose
        target_pose = self.current_target.pose
        
        # ===== 1. Position Check =====
        pos_error = np.array([
            target_pose.position.x - current_pose.pose.position.x,
            target_pose.position.y - current_pose.pose.position.y,
            target_pose.position.z - current_pose.pose.position.z
        ])
        position_distance = np.linalg.norm(pos_error)
        position_reached = position_distance < self.position_threshold

        # if position_reached:
            # self.cpmtrget_logger().info("Position reached!" , once = True)


        # ===== 2. Orientation Check =====
        current_rot = self.controller.get_ee_rotation() 
        # todo: SHOULD USE INSTEAD PRIOR TO COMP -> COARSE TARGET?
        if current_rot is None:
            # If we can't get rotation, only check position
            return position_reached
        
        target_rot = Rotation.from_quat([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ])
        
        # Use the same dynamic target for the error check
        tracking_offset = self.controller.get_tracking_compensation()
        dynamic_target_rot = target_rot * tracking_offset

        rot_diff = current_rot.inv() * dynamic_target_rot

        # Compute rotation difference
        # rot_diff = current_rot.inv() * target_rot
        
        # Get angle magnitude from quaternion
        angle_error_rad = 2 * np.arccos(np.clip(abs(rot_diff.as_quat()[3]), 0.0, 1.0))
        angle_error_deg = np.degrees(angle_error_rad)
        orientation_reached = angle_error_deg < self.orientation_threshold_deg
        
        # Debug logging (can be commented out for production)
        self.controller.get_logger().debug(
            f"Waypoint tracking - pos: {position_distance*1000:.1f}mm "
            f"(thresh: {self.position_threshold*1000:.1f}mm), "
            f"orient: {angle_error_deg:.1f}deg "
            f"(thresh: {self.orientation_threshold_deg:.1f}deg)"
        )
        orientation_reached = True


        # ===== 3. Finger Check (NEW) =====
        finger_reached = True
        if self.finger_queue:
            target_f = self.finger_queue[0]
            current_f = self.controller.current_finger_pose
            
            # Check absolute difference for each finger
            f_errors = np.abs(target_f - current_f)
            max_f_error = np.max(f_errors)
            self.controller.get_logger().debug(
                f"Finger tracking - max error: {max_f_error:.1f} "
                f"(thresh: {self.finger_tolerance:.1f})"
            )
            
            finger_reached = max_f_error < self.finger_tolerance
            
            # Optional Debugging
            if not finger_reached and position_reached and orientation_reached:
                self.controller.get_logger().info(
                    f"Waiting for fingers: Max error {max_f_error:.1f} > {self.finger_tolerance}"
                )

        return position_reached and orientation_reached and finger_reached 
    
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

        ### Arm velocity ###
        if not self.controller.current_pose or not self.current_target:
            # Safety: return zero velocity if state is invalid
            msg = PoseVelocityWithFingerVelocity()
            return msg
        
        current_pose = self.controller.current_pose
        target_pose = self.current_target.pose
        

        # ===================================== # 
        # LINEAR VELOCITY CONTROL (PID version) #
        # ===================================== #

        # 1. Compute distance to target
        pos_error = np.array([
            target_pose.position.x - current_pose.pose.position.x,
            target_pose.position.y - current_pose.pose.position.y,
            target_pose.position.z - current_pose.pose.position.z
        ])
       
        distance = np.linalg.norm(pos_error)
        
        # 2. Compute desired velocity based on the distance
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

            target_velocity = target_velocity + pid_correction

        else:
            target_velocity = np.zeros(3)
        
        # ====================================== # 
        # Angular Velocity (Orientation Control) #
        # ====================================== # 

        # 1. Get the base target rotation from the waypoint
        target_pose = self.current_target.pose
        
        # target_rot = Rotation.from_quat([
        #     target_pose.orientation.x,
        #     target_pose.orientation.y,
        #     target_pose.orientation.z,
        #     target_pose.orientation.w
        # ])

        waypoint_rot = Rotation.from_quat([
            target_pose.orientation.x, target_pose.orientation.y,
            target_pose.orientation.z, target_pose.orientation.w
        ])

        # 2. Apply Dynamic Tracking Compensation
        # If tracking is enabled, we 'bend' the waypoint's orientation 
        # towards the camera in real-time.
        tracking_offset = self.controller.get_tracking_compensation()
        dynamic_target_rot = waypoint_rot * tracking_offset 


        # 3. Compute velocity to this DYNAMIC target
        current_rot = self.controller.get_ee_rotation()
        if current_rot is not None:
            angular_velocity = self.controller.rotation_controller.compute_angular_velocity(
                dynamic_target_rot, # Use the dynamic one, not the static one
                current_rot,
                pid_controller=self.controller.pid_angular
            )
        else:
            angular_velocity = np.zeros(3)

        ### Finger velocity ###
        # ! ERROR HERE
        # 2. Finger velocity logic
        target_finger_velocity = np.zeros(3)
        if self.finger_queue:
            target_f = self.finger_queue[0] # The fingers for the current waypoint
            current_f = self.controller.current_finger_pose

            # P-Control (Error * Gain)
            # Using a gain of ~0.5 means it takes ~2 seconds to close fully
            kp = 0.5 
            f_vels = (target_f - current_f) * kp
            
            target_finger_velocity = np.clip(f_vels, -2000, 2000)


        # ===== Pack Into Message =====
        msg = PoseVelocityWithFingerVelocity()
        msg.twist_linear_x = float(target_velocity[0])
        msg.twist_linear_y = float(target_velocity[1])
        msg.twist_linear_z = float(target_velocity[2])
        msg.twist_angular_x = float(angular_velocity[0])
        msg.twist_angular_y = float(angular_velocity[1])
        msg.twist_angular_z = float(angular_velocity[2])
        msg.finger1 = float(target_finger_velocity[0])
        msg.finger2 = float(target_finger_velocity[1])
        msg.finger3 = float(target_finger_velocity[2])

        
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
    def __init__(self):
        super().__init__('robot_controller')
        
        # Params
        self._load_parameters()
        
        # Components
        self.pid_linear = PIDController(kp=self.kp_linear, ki=self.ki_linear, kd=self.kd_linear, length=3)
        self.pid_angular = PIDController(kp=self.kp_angular, ki=self.ki_angular, kd=self.kd_angular, length=3) # PID for angular velocity

        self.vel_filter = RollingAverageFilter(window_size=10)
        self.vel_integrator = VelocityIntegrator(self.max_linear_velocity)
        
        self.rotation_controller = RotationController(
            start_rotation=START_ROTATION,
            quantization_degrees=self.quantization_degrees
        )
        # Finger
        self.finger_controller = FingerController(self, max_velocity=self.max_finger_velocity)
        self.current_finger_pose = np.zeros(3) # Track current state
        self.target_finger_vels = [0.0, 0.0, 0.0]
        
        self.cached_waypoint_fingers = None # Cache for incoming discrete finger sequences

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Pub/Sub
        self.vel_pub = self.create_publisher(PoseVelocityWithFingerVelocity, '/j2n6s300_driver/in/cartesian_velocity_with_finger_velocity', 1)
        self.velocity_sub = self.create_subscription(TwistStamped, '/teleop/cartesian_velocity', self.velocity_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_pose', self.update_current_pose, 10)
        
        self.finger_vel_sub = self.create_subscription(Float64MultiArray, '/teleop/finger_velocity', self.finger_vel_callback,10)
        self.waypoint_finger_sub = self.create_subscription(Float64MultiArray, '/teleop/waypoint_fingers', self.waypoint_finger_callback, 10)

        # State
        self.current_pose = None
        self.current_vel = None
        self.time_prev = self.get_clock().now()
        self.latest_velocity_cmd = None
        
        # Behavior
        self.behaviors = {
            "translation": ContinuousTeleopBehavior(self),
            "rotation": ContinuousTeleopBehavior(self),
            "discrete": DiscreteTeleopBehavior(self),
            # Add system, etc.
        }
        self.current_mode = "translation"
        self.current_behavior = self.behaviors["translation"]
        self.get_logger().info(f"Initial control mode: {self.current_mode}")
        
        # Initialize ROS interfaces
        self._init_publishers()
        self._init_subscribers()
        self._init_tf()
        
        # Tracking for self-alignment
        self.last_valid_tracking_offset = Rotation.from_quat([0, 0, 0, 1])
        self.last_valid_input_to_base_rot = Rotation.from_quat([0, 0, 0, 1])
        self.smoothed_camera_pos = None 
        self.last_tracking_time = None  

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
        self.kp_linear = self.declare_parameter("kp_linear", 0.7).value
        self.ki_linear = self.declare_parameter("ki_linear", 0.1).value
        self.kd_linear = self.declare_parameter("kd_linear", 0.0).value
        self.kp_angular = self.declare_parameter("kp_angular", 2.0).value
        self.ki_angular = self.declare_parameter("ki_angular", 2.0).value
        self.kd_angular = self.declare_parameter("kd_angular", 5.0).value

        # Rotation
        self.discretize_rotation = self.declare_parameter("discretise_rotation", True).value
        self.quantization_degrees = self.declare_parameter("quantisation_degrees", 45).value
        
        
        self.camera_rotation_compensation = self.declare_parameter("camera_rotation_compensation", False).value
        
        
        # NEW: Discrete waypoint execution parameters
        self.discrete_motion_speed = self.declare_parameter(
            "discrete_motion_speed", 0.50).value  # 150% of max velocity
        
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
    
    def finger_vel_callback(self, msg):
        if len(msg.data) == 3:
            self.target_finger_vels = list(msg.data)

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

        # If no finger data was cached, create a default "zero movement" list
        if self.cached_waypoint_fingers is None:
            self.get_logger().warn("No finger targets cached. Using default [0,0,0] for all waypoints.")
            self.cached_waypoint_fingers = [0.0, 0.0, 0.0] * len(msg.poses)

        # Forward BOTH to the behavior
        # Helps in case finger arrives later
        success = self.behaviors["discrete"].load_waypoints(msg, self.cached_waypoint_fingers)
        
        if success:
            # Clear cache after loading
            self.cached_waypoint_fingers = None
        else:
            self.get_logger().error("Failed to load waypoint path")
                    
    def waypoint_finger_callback(self, msg: Float64MultiArray):
            """Cache the finger targets for the next incoming path."""
            self.cached_waypoint_fingers = list(msg.data)
            self.get_logger().info(f"Cached {len(msg.data)//3} finger targets for discrete path.")


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
            # 1. Ask the behavior (Continuous or Discrete) for the next command
            cmd = self.current_behavior.process_velocity_command(self.latest_velocity_cmd or TwistStamped())

            if cmd:
                # 2. If we are in Continuous modes, we still need to inject 
                # manual finger velocities (Option 2.B)
                if self.current_mode in ["translation", "rotation"]:
                    cmd.finger1 = float(self.target_finger_vels[0])
                    cmd.finger2 = float(self.target_finger_vels[1])
                    cmd.finger3 = float(self.target_finger_vels[2])
                
                # 3. Publish Debugging (Arm velocity)
                self.test_target_vel_pub.publish(Point(x=cmd.twist_linear_x, y=cmd.twist_linear_y, z=cmd.twist_linear_z))
                
                # 4. Final Publish to Jaco
                self.vel_pub.publish(cmd)
            else:
                self.publish_zero_velocity()

        
        except Exception as e:
            self.get_logger().error(f'Error in control_tick: {e}')
            traceback.print_exc()
        
    def get_tracking_compensation(self, camera_frame="camera_optical_frame", target_frame="bt_UpHybridRe_ee") -> Rotation:

        x_enabled = True
        y_enabled = False
        STALENESS_TIMEOUT_SEC = 2.5

        """Computes the rotation offset required to point the target_frame at the camera_frame."""
        pos_camera = self.get_frame_position(camera_frame, ROBOT_BASE_FRAME)
        pos_ee = self.get_frame_position(target_frame, ROBOT_BASE_FRAME)
        rot_ee = self.get_frame_rotation(target_frame, ROBOT_BASE_FRAME)

        # pos_target = self.get_frame_position(target_frame, ROBOT_BASE_FRAME)
        # if pos_target is None:
        #     pos_target = self.get_frame_position("j2n6s300_end_effector", ROBOT_BASE_FRAME)

        current_time = self.get_clock().now()
        # --- Staleness / fallback logic ---
        tracking_valid = (
            pos_camera is not None and
            pos_ee     is not None and
            rot_ee     is not None and
            np.linalg.norm(pos_camera) >= 0.01
        )

        if not tracking_valid:
            # Check how long we've been without a valid fix
            if self.last_tracking_time is not None:
                stale_sec = (current_time - self.last_tracking_time).nanoseconds * 1e-9
                if stale_sec > STALENESS_TIMEOUT_SEC:
                    # Decay back to identity
                    return Rotation.from_quat([0, 0, 0, 1])
            return self.last_valid_tracking_offset

        # Basic validity check
        if pos_camera is None or pos_ee is None or rot_ee is None or np.linalg.norm(pos_camera) < 0.01:
            return self.last_valid_tracking_offset

        # =================================================================
        # 1. Calculate actual time delta (dt)
        # =================================================================
        if self.last_tracking_time is None:
            dt = 1.0 / REFRESH_RATE
        else:
            dt = (current_time - self.last_tracking_time).nanoseconds * 1e-9
        
        self.last_tracking_time = current_time
        
        # Safety clamp: if the node hangs for a second, pretend only 0.1s passed 
        # so it doesn't suddenly jump
        dt = min(dt, 0.1)

        # =================================================================
        # 1. Smooth the user's head position (EMA Filter)
        # =================================================================
        settle_gain = 2.5  # Higher = faster settling, but more jitter. Adjust as needed.
        alpha = 1.0 - np.exp(-settle_gain * dt)
        # alpha = 0.002  # Smoothing factor (0.0 to 1.0). Lower = smoother, less jitter.

        if self.smoothed_camera_pos is None:
            # First valid frame: initialize to exact position to prevent slow sweep
            self.smoothed_camera_pos = pos_camera
        else:
            # Check if there's a massive jump (e.g., tracking lost and regained far away)
            if np.linalg.norm(pos_camera - self.smoothed_camera_pos) > 0.5: # 50cm jump
                self.smoothed_camera_pos = pos_camera
            else:
                # Standard EMA: New = (alpha * current) + ((1 - alpha) * previous)
                self.smoothed_camera_pos = (alpha * pos_camera) + ((1.0 - alpha) * self.smoothed_camera_pos)

        # Vector from End Effector to Camera in Base Frame
        vec_to_target = self.smoothed_camera_pos - pos_ee
        vec_local = rot_ee.inv().apply(vec_to_target)

        # Pitch (X) and Yaw (Z) logic from your working code
        angle_x = np.arctan2(-vec_local[1], vec_local[2])    if y_enabled  else 0
        yz_magnitude = np.hypot(-vec_local[1], vec_local[2])
        angle_z = -np.arctan2(vec_local[0], yz_magnitude)   if x_enabled else  0


        result = Rotation.from_euler('xz', [angle_x, angle_z], degrees=False)
        self.last_valid_tracking_offset = result
        return result

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
            self.last_valid_input_to_base_rot = rot
            return rot.apply(vector)
        
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(f"Could not transform vector {source_frame} -> {target_frame}")
            return self.last_valid_input_to_base_rot.apply(vector)

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
            return self.last_valid_input_to_base_rot

    def get_frame_position(self, source_frame: str, target_frame: str) -> Optional[np.ndarray]:
        """Gets the position of source_frame represented in target_frame."""
        if not source_frame or source_frame == target_frame:
            return None # Identity
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def get_ee_rotation(self) -> Optional[Rotation]:
        return self.get_frame_rotation('j2n6s300_end_effector', ROBOT_BASE_FRAME)

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