#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node

from kinova_msgs.action import ArmPose
from kinova_msgs.msg import PoseVelocity
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from kinova_msgs.msg import PoseVelocity, PoseVelocityWithFingerVelocity

from tf2_ros.transform_listener import TransformListener

from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform

from scipy.spatial.transform import Rotation # IMPORTANT, USES x,y,z,w
import numpy as np

from enum import Enum

"""# Movement enabled? Useful for testing
CARTESIAN_MOVEMENT_ENABLED = True
CAMERA_ROTATION_COMPENSATION = False

# Constants for max velocities
MAX_LINEAR_VELOCITY = 0.070    # m/s
MAX_ANGULAR_VELOCITY = 2.0      
MAX_FINGER_VELOCITY = 2000.0    # mm/s
ROTATION_ASSISTANCE = 2.0
ROTATION_INTEGRATION = 0.01

# Constants for cartesian movement behaviour
NORMALISE_CARTESIAN_SPEED = False
RELATIVE_CARTESIAN_MOVEMENT_ENABLED = False

# ROTATION
DISCRETISE_ROTATION = True
QUANTISATION = 30.0"""

class ControlMode(Enum):
    GLOBAL = 0
    RELATIVE_EE = 1
    #HEAD_FOLLOW_SPHERICAL = 2
    #HEAD_FOLLOW_CYLINDRICAL = 3

active_control_mode = ControlMode.RELATIVE_EE

# ! Keep at 100Hz. As per documentation
REFRESH_RATE = 100.0

# ros2 service call /j2n6s300_driver/in/set_torque_control_mode kinova_msgs/srv/SetTorqueControlMode state:\ 1\
# ros2 service call /j2n6s300_driver/in/start kinova_msgs/srv/Start {}\ 

class JacoController(Node):
    def __init__(self):
        super().__init__('jaco_main_controller')
        self.get_logger().info("Robot control node is Running...")

        # Create subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.update_target_pose, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_pose', self.update_current_pose, 10)

        self.tf_target_buffer = Buffer()
        self.tf_target_listener = TransformListener(self.tf_target_buffer, self)
        
        # Create publishers
        self.vel_pub = self.create_publisher(PoseVelocityWithFingerVelocity, '/j2n6s300_driver/in/cartesian_velocity_with_finger_velocity', 1)
        self.dynamic_broadcaster = TransformBroadcaster(self)

        self.update = self.create_timer(1.0/REFRESH_RATE, self.update_controller)

        # Parameters
        self.MAX_LINEAR_VELOCITY = self.declare_and_get_parameter('max_linear_velocity', 0.070)
        self.MAX_ANGULAR_VELOCITY = self.declare_and_get_parameter('max_angular_velocity', 2.0)
        self.MAX_FINGER_VELOCITY = self.declare_and_get_parameter('max_finger_velocity', 2000.0)
        self.CARTESIAN_MOVEMENT_ENABLED = self.declare_and_get_parameter('cartesian_movement_enabled', True)
        self.CAMERA_ROTATION_COMPENSATION = self.declare_and_get_parameter('camera_rotation_compensation', False)
        self.NORMALISE_CARTESIAN_SPEED = self.declare_and_get_parameter('normalise_cartesian_speed', False)
        self.RELATIVE_CARTESIAN_MOVEMENT_ENABLED = self.declare_and_get_parameter('relative_cartesian_movement_enabled', False)
        self.DISCRETISE_ROTATION = self.declare_and_get_parameter('discretise_rotation', True)
        self.QUANTISATION = self.declare_and_get_parameter('quantisation', 30.0)

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

    def declare_and_get_parameter(self, name, default):
        self.declare_parameter(name, default)
        self.get_logger().info(
            f"Loaded parameter {name}: {self.get_parameter(name).value}"
        )
        return self.get_parameter(name).value

    ### MAIN CALLBACK ### 
    def update_controller(self):

        # Only update if we have all needed data: controller and current pose
        if self.joy_msg is None or self.current_pose is None :
            return

        """# TODO check time stamps are at least less than 0.1s apart
        current_time = self.get_clock().now()
        if self.joy_msg.header.stamp.sec == 0 or self.current_pose.header.stamp.sec == 0:
            return
        if (current_time - self.joy_msg.header.stamp).nanoseconds > 1e9 or (current_time - self.current_pose.header.stamp).nanoseconds > 1e9:
            return"""

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
            # Generate a velocity for the rotation target using PID
            # TODO: Implement PID for rotation
            # Decouple the rotation and linear velocity
            # library? or class?
                
            ### Pack pose goal into message ###
            target_vel_msg = PoseVelocityWithFingerVelocity() 

            target_vel_msg.twist_linear_x = target_vel[0]
            target_vel_msg.twist_linear_y = target_vel[1]
            target_vel_msg.twist_linear_z = target_vel[2] 

            target_vel_msg.twist_angular_x = rotation_vel_target[0]
            target_vel_msg.twist_angular_y = rotation_vel_target[1]
            target_vel_msg.twist_angular_z = rotation_vel_target[2]

            # --- Finger Control ---
            finger_target_velocity = self.find_finger_velocity(self.joy_msg)
            
            target_vel_msg.finger1 = finger_target_velocity[0]
            target_vel_msg.finger2 = finger_target_velocity[1]
            target_vel_msg.finger3 = finger_target_velocity[2]

            # Pack pose goal into action message
            self.vel_pub.publish(target_vel_msg)

            #self.get_logger().info("Sending")
            #self.log_results(separator=True)

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
        if not self.CARTESIAN_MOVEMENT_ENABLED:
            return np.zeros(3)

        # Target vel update
        cartesian_vel_target = np.zeros(3)

        # Read from controller 
        cartesian_vel_target[0] = (joy_msg.axes[0] ) # Joystick left horizontal
        cartesian_vel_target[1] = ((joy_msg.axes[5] - joy_msg.axes[2]) ) # Trigger difference
        cartesian_vel_target[2] = (joy_msg.axes[1] ) # Joystick left vertical
        
        
        # Normalise and multiply by max velocity
        if self.NORMALISE_CARTESIAN_SPEED :
            cartesian_vel_target = cartesian_vel_target / np.linalg.norm(cartesian_vel_target)
            
        # Multiply by max velocity
        cartesian_vel_target = cartesian_vel_target * self.MAX_LINEAR_VELOCITY

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
        if self.DISCRETISE_ROTATION:
            controller_target_rotation = np.round(controller_target_rotation / QUANTISATION) * QUANTISATION
        self.get_logger().info(f"Discretised rotation: {controller_target_rotation}")

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

            if not (glasses_from_marker is None) and CAMERA_ROTATION_COMPENSATION:

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
            self.get_logger().info("ABOVE MAX ROTATION")
            #self.get_logger().info(f"Max angle difference is: {mag * 180/3.14}")
        
            new_mag = MAX_ANGLE_DIFFERENCE
            #quaternion_diff =slerp(quaternion_diff, 1, new_mag/mag)
            pass

        # Convert to euler
        convention = "xyz" # As per the docs
    
        self.rotation_euler_target = quaternion_diff.as_euler(convention, degrees=False) # Matches docs

        # Find angular velocity
        #euler_velocity_target = self.rotation_euler_target * MAX_ANGULAR_VELOCITY

        euler_velocity_target = np.zeros(3)
        euler_velocity_target[0] = self.rotation_euler_target[0] * self.MAX_ANGULAR_VELOCITY
        euler_velocity_target[1] = self.rotation_euler_target[1] * self.MAX_ANGULAR_VELOCITY
        euler_velocity_target[2] = self.rotation_euler_target[2] * self.MAX_ANGULAR_VELOCITY 

        return euler_velocity_target

    def find_finger_velocity(self, joy_msg:Joy) -> np.ndarray:
        # A - B
        finger_target_velocity = np.zeros(3)
        finger_target_velocity[0] = (self.joy_msg.buttons[1] - self.joy_msg.buttons[0]) * self.MAX_FINGER_VELOCITY
        finger_target_velocity[1] = (self.joy_msg.buttons[1] - self.joy_msg.buttons[0]) * self.MAX_FINGER_VELOCITY
        finger_target_velocity[2] = (self.joy_msg.buttons[1] - self.joy_msg.buttons[0]) * self.MAX_FINGER_VELOCITY

        return finger_target_velocity


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
        self.current_pose = pose_msg

    def update_current_force(self, force_msg):
        self.current_force = force_msg 

    def update_target_pose(self, joy_msg): 
        self.joy_msg = joy_msg

def main():
    rclpy.init()
    try:
        controller = JacoController()
        rclpy.spin(controller)
    except Exception as e:
        print(f'Error in main: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
