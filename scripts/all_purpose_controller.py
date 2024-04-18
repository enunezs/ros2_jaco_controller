#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node

from kinova_msgs.action import ArmPose
from kinova_msgs.msg import PoseVelocity, PoseVelocityWithFingerVelocity
from geometry_msgs.msg import TransformStamped, Transform
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose, Point

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

from enum import Enum

from scipy.spatial.transform import Rotation # IMPORTANT, USES x,y,z,w

import numpy as np
import traceback

# Constants for control modes
class ControlMode(Enum):
    GLOBAL = 0
    RELATIVE_EE = 1
    #HEAD_FOLLOW_SPHERICAL = 2
    #HEAD_FOLLOW_CYLINDRICAL = 3

active_control_mode = ControlMode.RELATIVE_EE

# ! LOCK at 100Hz. As per documentation
REFRESH_RATE = 100.0

# ros2 service call /j2n6s300_driver/in/set_torque_control_mode kinova_msgs/srv/SetTorqueControlMode state:\ 1\
# ros2 service call /j2n6s300_driver/in/start kinova_msgs/srv/Start {}\ 



import numpy as np

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, refresh_rate=1.0,length=3):
        self.kp = kpself.
        self.length = length
        self.prev_error = np.zeros(self.length)
        self.integral_error = np.zeros(self.length)
        self.derivative_error = np.zeros(self.length)

    def update_parameters(self, kp=None, ki=None, kd=None, refresh_rate=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if refresh_rate is not None:
            self.refresh_rate = refresh_rate

    def control_update(self, target_vel, current_vel):
        if target_vel is None or current_vel is None:
            return np.zeros(self.length)

        current_error = target_vel - current_vel

        self.integral_error += current_error * (1.0 / self.refresh_rate)
        self.derivative_error = (current_error - self.prev_error) * self.refresh_rate

        self.prev_error = current_error

        return self.kp * current_error + self.ki * self.integral_error + self.kd * self.derivative_error


from collections import deque

class RollingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
        self.sum = 0.0

    def update(self, value):
        if len(self.values) == self.window_size:
            self.sum -= self.values[0]
        self.values.append(value)
        self.sum += value
        return self.sum / len(self.values)

class JacoController(Node):
    def __init__(self):
        super().__init__('jaco_simple_pid_controller')
        self.get_logger().info("Robot control node is Running...")

        # Create subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.update_target_pose, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_pose', self.update_current_pose, 10)
        self.force_sub = self.create_subscription(WrenchStamped, '/j2n6s300_driver/out/tool_wrench', self.update_current_force, 10)

        self.tf_target_buffer = Buffer()
        self.tf_target_listener = TransformListener(self.tf_target_buffer, self)
        
        # Create publishers
        self.vel_pub = self.create_publisher(PoseVelocityWithFingerVelocity, '/j2n6s300_driver/in/cartesian_velocity_with_finger_velocity', 1)
        self.dynamic_broadcaster = TransformBroadcaster(self)

        self.test_target_vel_pub = self.create_publisher(Point, '/jaco/test_target_vel_pub', 1)
        self.test_measured_vel_pub = self.create_publisher(Point, '/jaco/test_measured_vel_pub', 1)

        self.update = self.create_timer(1.0/REFRESH_RATE, self.update_controller)

        ### Parameters ### 
        # Cartesian movement
        self.cartesian_movement_enabled = self.declare_and_get_parameter("cartesian_movement_enabled", True)
        self.normalise_cartesian_speed = self.declare_and_get_parameter("normalise_cartesian_speed", False)
        self.relative_cartesian_movement_enabled = self.declare_and_get_parameter("relative_cartesian_movement_enabled", False)

        # Max velocities
        self.max_linear_velocity = self.declare_and_get_parameter("max_linear_velocity", 0.070)
        self.max_angular_velocity = self.declare_and_get_parameter("max_angular_velocity", 2.0)
        self.max_finger_velocity = self.declare_and_get_parameter("max_finger_velocity", 2000.0)

        # Rotation
        self.discretise_rotation = self.declare_and_get_parameter("discretise_rotation", True)
        self.quantisation_degrees = self.declare_and_get_parameter("quantisation_degrees", 30.0)

        # PID
        self.pid_enabled = self.declare_and_get_parameter("pid_enabled", False)
        self.kp_linear = self.declare_and_get_parameter("kp_linear", 1.0)
        self.ki_linear = self.declare_and_get_parameter("ki_linear", 0.0)
        self.kd_linear = self.declare_and_get_parameter("kd_linear", 0.0)
        self.kp_angular = self.declare_and_get_parameter("kp_angular", 1.0)
        self.ki_angular = self.declare_and_get_parameter("ki_angular", 0.0)
        self.kd_angular = self.declare_and_get_parameter("kd_angular", 0.0)

        # Camera rotation compensation
        self.camera_rotation_compensation = self.declare_and_get_parameter("camera_rotation_compensation", False)
        self.rotation_assistance = self.declare_and_get_parameter("rotation_assistance", 2.0)

        # Robot state
        self.target_pose = None
        self.current_pose = None
        self.current_vel = None # Unused right now
        #self.prev_error = None
        #self.start_pose = None

        # Joystick buffer
        self.joy_msg = None
        self.prev_joy_msg = None

        # Rotation changes buffer
        self.cumulative_rotation = np.zeros(3) # Euler angles
        
        # Rotation start
        self.roll = 0 
        self.pitch = 0

        # PID
        self.PID_linear_vel = PIDController(kp=self.kp_linear, ki=self.ki_linear, kd=self.kd_linear, refresh_rate=REFRESH_RATE)
        self.PID_angular_vel = PIDController(kp=self.kp_angular, ki=self.ki_angular, kd=self.kd_angular, refresh_rate=REFRESH_RATE)

        self.vel_filter = RollingAverageFilter(window_size=5)  # Adjust window_size as needed


    def declare_and_get_parameter(self, name, default):
        self.declare_parameter(name, default)
        self.get_logger().info(
            f"Loaded parameter {name}: {self.get_parameter(name).value}"
        )
        return self.get_parameter(name).value

    ### MAIN CALLBACK ### 
    def update_controller(self):

        # Only update if we have all needed data: controller and current pose
        # TODO check time stamps are recent 
        if self.joy_msg is None or self.current_pose is None :
            return
        try:

            ### Preprocess discrete actions
            self.discrete_actions(self.joy_msg)

            ### Cartesian velocity update ###
            global_vel: np.ndarray = self.find_linear_velocity(self.joy_msg)
            target_vel: np.ndarray = global_vel
            
            global active_control_mode
            if active_control_mode == ControlMode.RELATIVE_EE:
                local_vel = self.find_relative_velocity(global_vel, target_frame = 'j2n6s300_end_effector')
                target_vel = local_vel

            ### Rotation velocity update ###
            self.rotation_target_pose : Rotation = self.find_rotation_pose(self.joy_msg) 
            current_ee_rotation = self.get_ee_rotation()

            if current_ee_rotation is not None:
                rotation_vel_target = self.find_rotation_velocity(self.rotation_target_pose, current_ee_rotation)
            else:
                rotation_vel_target = np.zeros(3)

            ### PID ###
            if self.pid_enabled and self.current_vel is not None:
                #self.get_logger().info(f"{target_vel}")
                #self.get_logger().info(f"{self.current_vel[0:3]}")
        
                # PID decoupled for linear and angular
                # Values are different
                target_vel = self.PID_linear_vel.control_update(target_vel, self.current_vel[0:3]) #, rotation_vel_target, current_ee_rotation)
                # rotation_vel_target = self.PID_angular_vel.control_update(rotation_vel_target, self.current_vel[3:6]) #, rotation_vel_target, current_ee_rotation)

                # Publish for debugging controller
                #self.test_measured_vel_pub.publish(Point(x=self.current_vel[0], y=self.current_vel[1], z=self.current_vel[2]))
                #self.test_target_vel_pub.publish(Point(x=target_vel[0], y=target_vel[1], z=target_vel[2]))

            # --- Finger Control ---
            finger_target_velocity = self.find_finger_velocity(self.joy_msg)

            ### Pack pose goal into message ###
            target_vel_msg = PoseVelocityWithFingerVelocity() 

            target_vel_msg.twist_linear_x = target_vel[0]
            target_vel_msg.twist_linear_y = target_vel[1]
            target_vel_msg.twist_linear_z = target_vel[2] 

            target_vel_msg.twist_angular_x = rotation_vel_target[0]
            target_vel_msg.twist_angular_y = rotation_vel_target[1]
            target_vel_msg.twist_angular_z = rotation_vel_target[2]
            
            target_vel_msg.finger1 = finger_target_velocity[0]
            target_vel_msg.finger2 = finger_target_velocity[1]
            target_vel_msg.finger3 = finger_target_velocity[2]

            # Pack pose goal into action message
            self.vel_pub.publish(target_vel_msg)

            # Debugging
            #self.get_logger().info("Sending")
            """self.log_results(current_rot = True, 
                             target_rot=True ,  
                             separator=True)"""

        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {e}')

    def log_results(self, current_pose=False, target_pose=False, error=False, target_vel=False, current_vel=False, current_rot=False, target_rot=True, separator=True):

        if current_pose:
            self.get_logger().info("Current pose: %0.3f, %0.3f, %0.3f" % (self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z))
        if target_pose:
            self.get_logger().info("Target pose: %0.3f, %0.3f, %0.3f" % (self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z))
        if error:
            self.get_logger().info("Error: %0.3f, %0.3f, %0.3f" % (error[0], error[1], error[2]))
        if current_vel:
            self.get_logger().info("Current vel: %0.3f, %0.3f, %0.3f" % (self.current_force.pose.position.x, self.current_force.pose.position.y, self.current_force.pose.position.z))
        if current_rot:
            self.get_logger().info("Current rot: %0.3f, %0.3f, %0.3f, %0.3f" % (self.current_pose.pose.orientation.w, self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z))
        if target_rot:
            self.get_logger().info("Target rot: %0.3f, %0.3f, %0.3f, %0.3f" % (self.rotation_target_pose[0], self.rotation_target_pose[1], self.rotation_target_pose[2], self.rotation_target_pose[3]))
        if separator:
            self.get_logger().info("------------------------------------------------------")

        pass

    def discrete_actions(self, joy_msg:Joy):

        if self.prev_joy_msg is None:
            self.prev_joy_msg = joy_msg
            return
        
        ### Discrete actions ###

        ### Cycle to next control mode when pressing START ###
        if joy_msg.buttons[7] == 1 and self.prev_joy_msg.buttons[7] == 0:
            # Switch control mode, increase index by one
            global active_control_mode
            active_control_mode = ControlMode((active_control_mode.value + 1) % len(ControlMode))
            self.get_logger().info(f"Switching control mode to: {active_control_mode}")


        # Update previous joystick for next iteration
        self.prev_joy_msg = joy_msg


    def find_linear_velocity(self, joy_msg) -> np.ndarray:

        # If cartesian movement is disabled, return 0,0,0
        if not self.cartesian_movement_enabled:
            return np.zeros(3)

        # Target vel update
        cartesian_vel_target = np.zeros(3)

        # Read from controller 
        cartesian_vel_target[0] = (joy_msg.axes[0] ) # Joystick left horizontal
        cartesian_vel_target[1] = ((joy_msg.axes[5] - joy_msg.axes[2]) ) # Trigger difference
        cartesian_vel_target[2] = (joy_msg.axes[1] ) # Joystick left vertical
        
        
        # Normalise and multiply by max velocity
        if self.normalise_cartesian_speed :
            cartesian_vel_target = cartesian_vel_target / np.linalg.norm(cartesian_vel_target)
            
        # Multiply by max velocity
        cartesian_vel_target = cartesian_vel_target * self.max_linear_velocity

        return cartesian_vel_target
    
    def find_relative_velocity(self, global_vel : np.ndarray , target_frame = 'j2n6s300_end_effector' , source_frame = 'j2n6s300_link_base') -> np.ndarray:

        # buffer may not be ready. Ugly temp fix
        try:
            transform: TransformStamped = self.tf_target_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            # Turn from quaternion to rotation matrix
            rot = Rotation.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            local_vel = rot.inv().apply(global_vel[0:3])

        except Exception as e:
            self.get_logger().error(f'Error in lookup_transform: {e}')
            local_vel = global_vel[0:3]

        return local_vel


    def find_rotation_pose(self, joy_msg:Joy) -> Rotation:
        # Define the target
        world_frame = 'j2n6s300_link_base'

        ### START ROTATION POINT ###
        #rot_base = Rotation.from_quat([0.5, 0.5, -0.5, 0.5]) # Pointing forward, buttons left # scipy convention (x, y, z, w)  
        #rot_base = Rotation.from_quat([0.0, 0.0, 0, 1]) # Pointing up # scipy convention (x, y, z, w)  
        #rot_base = Rotation.from_quat([0.707, 0.0, 0, 0.707]) # Pointing forward # scipy convention (x, y, z, w)  
        rot_base = Rotation.from_euler('xyz', [ -180,0,90], degrees=True)
        rot_base = Rotation.from_euler('xyz', [ -180,0,180], degrees=True)


        ### ROTATION FROM CONTROLLER ###
        orientation_change = np.zeros(3)
        orientation_change[0] = (-joy_msg.axes[4] ) 
        orientation_change[1] = (-joy_msg.axes[3] )  
        orientation_change[2] = (joy_msg.buttons[5] - joy_msg.buttons[4]) 
        orientation_change = orientation_change * 20.0

        # Clamp rotation
        MAX_ROTATION_CHANGE = 60.0
        orientation_change = np.clip(orientation_change, -MAX_ROTATION_CHANGE, MAX_ROTATION_CHANGE)

        # Apply to cummulative rotation
        self.cumulative_rotation += orientation_change * (1.0 / REFRESH_RATE) # Not accurate!
        controller_target_rotation = self.cumulative_rotation
        # Discretise
        if self.discretise_rotation:
            controller_target_rotation = np.round(controller_target_rotation / self.quantisation_degrees) * self.quantisation_degrees
        #self.get_logger().info(f"Discretised rotation: {controller_target_rotation}")

        # Apply
        rot_controller_target = Rotation.from_euler('xyz', [ controller_target_rotation[0], 
                                                            controller_target_rotation[1], 
                                                            controller_target_rotation[2] ], 
                                                            degrees=True)

        ### ROTATION FROM CAMERA ANGLE COMPENSATION ###
        # Get the position of the target frame in the world frame
        roll = 0 
        pitch = 0
        
        # ! Fix!!
        """target_frame = 'marker_5'
        aligning_frame = 'camera'

        try:
            glasses_from_marker = self.get_frame(target_frame, aligning_frame)

            if not (glasses_from_marker is None) and self.camera_rotation_compensation:

                # Up / down
                # Easiest way, find angles and apply to the aligning frame
                # Differences in position indicate the direction of the target
                reference_position = glasses_from_marker.transform.translation
                reference_position = [reference_position.x , reference_position.y, reference_position.z]
                self.get_logger().info(f"Camera Pos Relative to marker: {reference_position}")

                roll =  ( np.arctan2(reference_position[1], reference_position[2]) )# 'ballerina'
                roll = np.degrees(roll)

                pitch = ( np.arctan2(reference_position[0], reference_position[2]) )# 'ballerina'
                pitch = np.degrees(pitch)
                #pitch = np.arctan2(-reference_position[2], np.sqrt(reference_position[0]**2 + reference_position[1]**2)) 
        except Exception as e:
            #pass
            self.get_logger().info(f"Target roll: {roll}") 
            #self.get_logger().error(f'Error in lookup_transform: {e}')
        #self.get_logger().info(f"Target pitch: {pitch}") """

        self.roll += roll
        self.pitch += pitch 
        rot_compensation = Rotation.from_euler('xyz', [self.roll, 0, self.pitch], degrees=True)

        # Apply rotation to base
        quaternion_target = rot_base * rot_controller_target * rot_compensation 

        self.publish_rotation_target(quaternion_target, 'world', 'end_effector_rot_controller_target', visual_offset=[0.0 ,-0.35,0.35])

        return quaternion_target
    
    def get_ee_rotation(self):
        _ee_transform = self.get_frame('j2n6s300_link_base','j2n6s300_end_effector')

        if _ee_transform is not None:
            _current_rot = _ee_transform.transform.rotation
            _current_pose_quat = Rotation.from_quat([ #scipy convention (x, y, z, w) 
                _current_rot.x,
                _current_rot.y, 
                _current_rot.z, 
                _current_rot.w])   
            return _current_pose_quat
        else:
            self.get_logger().info("Failed to get current pose")
            return None


    def find_rotation_velocity(self, quaternion_target:Rotation, quaternion_current:Rotation):

        # Find quaternion difference
        quaternion_diff = (quaternion_current.inv())*(quaternion_target)

        self.publish_rotation_target( quaternion_diff, 'j2n6s300_end_effector', 'rot_change', visual_offset=[0.0 ,0.01, 0.2 ])
        #self.publish_rotation_target(_current_pose_quat.as_quat(), 'j2n6s300_link_base', 'current_pose', visual_offset=[0,0.05,0.15])

        # Get magnitude of rotation
        mag = 2*np.arccos(quaternion_diff.as_quat()[3]) # scipy convention (x, y, z, w)  
        #self.get_logger().info(f"Rotation magnitude is: {mag * 180/3.14}") 

        # TODO Clamp max angular velocity or tracking. If angle is extreme ignore   
        # https://math.stackexchange.com/questions/3219887/downscale-quaternion
        MAX_ANGLE_DIFFERENCE = 3.14/4
        if mag > MAX_ANGLE_DIFFERENCE:
            #self.get_logger().info("ABOVE MAX ROTATION")
            #self.get_logger().info(f"Max angle difference is: {mag * 180/3.14}")
        
            new_mag = MAX_ANGLE_DIFFERENCE
            #quaternion_diff =slerp(quaternion_diff, 1, new_mag/mag)
            pass

        # Convert to euler
        convention = "xyz" # As per the docs
    
        self.rotation_euler_target = quaternion_diff.as_euler(convention, degrees=False) # Matches docs

        # Find angular velocity
        #euler_velocity_target = self.rotation_euler_target * self.max_angular_velocity

        euler_velocity_target = np.zeros(3)
        euler_velocity_target[0] = self.rotation_euler_target[0] * self.max_angular_velocity
        euler_velocity_target[1] = self.rotation_euler_target[1] * self.max_angular_velocity
        euler_velocity_target[2] = self.rotation_euler_target[2] * self.max_angular_velocity 

        return euler_velocity_target

    def find_finger_velocity(self, joy_msg:Joy) -> np.ndarray:
        # A - B
        finger_target_velocity = np.zeros(3)
        finger_target_velocity[0] = (self.joy_msg.buttons[1] - self.joy_msg.buttons[0]) * self.max_finger_velocity
        finger_target_velocity[1] = (self.joy_msg.buttons[1] - self.joy_msg.buttons[0]) * self.max_finger_velocity
        finger_target_velocity[2] = (self.joy_msg.buttons[1] - self.joy_msg.buttons[0]) * self.max_finger_velocity

        return finger_target_velocity


    # Better to do as a class?
    def PID_controller(self, target_vel:np.ndarray, current_vel:np.ndarray) -> np.ndarray:

        if target_vel is None or current_vel is None: #  or self.current_force is None:
            return np.zeros(6)
        if self.prev_error is None:
            self.prev_error = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


        current_error = target_vel - current_vel

        self.integral_error += current_error * (1.0 / REFRESH_RATE)
        
        self.derivative_error = (current_error - self.prev_error) * REFRESH_RATE

        prev_error = current_error
        

    def get_frame(self, target_frame, source_frame):
        try:
            # Get the transform from the source frame to the target frame
            transform: TransformStamped = self.tf_target_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())

            # The position of the frame is in the transform's translation
            return transform

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error('Failed to get transform: %s' % e)

    def publish_rotation_target(self, quaternion_target:Rotation, frame_id, child_frame_id, visual_offset=[0.0,0.0,0.0]):
        dynamic_transform = TransformStamped()
        dynamic_transform.header.stamp = self.get_clock().now().to_msg()
        dynamic_transform.header.frame_id = frame_id # From global frame
        dynamic_transform.child_frame_id = child_frame_id

        dynamic_transform.transform.translation.x = visual_offset[0]
        dynamic_transform.transform.translation.y = visual_offset[1]
        dynamic_transform.transform.translation.z = visual_offset[2]

        quaternion_target = quaternion_target.as_quat() # scipy convention (x, y, z, w)  

        dynamic_transform.transform.rotation.x = quaternion_target[0]
        dynamic_transform.transform.rotation.y = quaternion_target[1]
        dynamic_transform.transform.rotation.z = quaternion_target[2]
        dynamic_transform.transform.rotation.w = quaternion_target[3]

        self.dynamic_broadcaster.sendTransform(dynamic_transform)
        #self.get_logger().info("Published rotation target")

    # Subscriber buffer callbacks
    def update_current_pose(self, pose_msg):
        prev_pose = self.current_pose
        self.current_pose = pose_msg
        
        #self.update_current_vel(self.current_pose, prev_pose)




    def update_current_vel(self, current_pose, prev_pose):
        if prev_pose is None:
            self.current_vel = np.zeros(6)
            self.time_prev = self.get_clock().now()
            self.get_clock().now()
            return
        
        
        current_time = self.get_clock().now()
        duration = (current_time - self.time_prev).nanoseconds() * 1e-9
        raw_vel = np.array([current_pose.pose.position.x - prev_pose.pose.position.x,
                                     current_pose.pose.position.y - prev_pose.pose.position.y,
                                     current_pose.pose.position.z - prev_pose.pose.position.z,
                                     current_pose.pose.orientation.w - prev_pose.pose.orientation.w,
                                     current_pose.pose.orientation.x - prev_pose.pose.orientation.x,
                                     current_pose.pose.orientation.y - prev_pose.pose.orientation.y])/ duration
        
        self.current_vel = self.vel_filter.update(raw_vel)

        #1.0/(current_time-self.time_prev)
        
        self.test_measured_vel_pub.publish(Point(x=self.current_vel[0], y=self.current_vel[1], z=self.current_vel[2]))
        self.time_prev = current_time


    def update_current_force(self, force_msg):
        self.current_force = force_msg 

    def update_target_pose(self, joy_msg): 
        self.joy_msg = joy_msg

def main():
    rclpy.init()
    controller = JacoController()
    try:
        rclpy.spin(controller)
    except Exception as e:
        print(f'Error in main: {e}')
        traceback.print_exc()  # This will print the complete error stack trace

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
