import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node

from kinova_msgs.action import ArmPose
from kinova_msgs.msg import PoseVelocity
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose


from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform

from scipy.spatial.transform import Rotation # IMPORTANT, USES x,y,z,w
import numpy as np

# Constants for max velocities
MAX_LINEAR_VELOCITY = 0.2
MAX_ANGULAR_VELOCITY = 1.0
ANGULAR_Z_VELOCITY = 1.0  

REFRESH_RATE = 50.0

# ros2 service call /j2n6s300_driver/in/set_torque_control_mode kinova_msgs/srv/SetTorqueControlMode state:\ 1\
# ros2 service call /j2n6s300_driver/in/start kinova_msgs/srv/Start {}\ 

class JacoController(Node):
    def __init__(self):
        super().__init__('jaco_simple_pid_controller')
        self.get_logger().info("Robot control node is Running...")

        # Create subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.update_target_pose, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_pose', self.update_current_pose, 10)
        
        # Create publishers
        self.vel_pub = self.create_publisher(PoseVelocity, '/j2n6s300_driver/in/cartesian_velocity', 1)
        self.dynamic_broadcaster = TransformBroadcaster(self)

        self.update = self.create_timer(1.0/REFRESH_RATE, self.update_controller)

        # Robot state
        self.target_pose = None
        self.current_pose = None
        self.current_vel = None
        self.prev_error = None
        self.start_pose = None
        self.joy_msg = None


    ### MAIN CALLBACK ### 
    def update_controller(self):

        # Only update if we have all needed data
        if self.joy_msg is None or self.current_pose is None :
            return

        try:
            ### Cartesian vel update ###
            # Target vel update
            self.cartesian_vel_target = self.find_linear_velocity(self.joy_msg)

            ### Rotation vel update ###
            # Find rotation target, convert to euler
            self.rotation_quaternion_target = self.find_rotation_pose(self.joy_msg)

            # Find rotation velocity targets
            # using scipy convention for quaternions (w,x,y,z)
            _rotation_quaternion_target = Rotation.from_quat([
                self.rotation_quaternion_target[1] ,
                self.rotation_quaternion_target[2], 
                self.rotation_quaternion_target[3], 
                self.rotation_quaternion_target[0]])
            
            _current_pose_quat = Rotation.from_quat([self.current_pose.pose.orientation.x, 
                                                     self.current_pose.pose.orientation.y, 
                                                     self.current_pose.pose.orientation.z,
                                                     self.current_pose.pose.orientation.w])     
                   
            rotation_vel_target = self.find_rotation_velocity(_rotation_quaternion_target, _current_pose_quat)            

            ### PID ###
            # Generate a velocity for the rotation target using PID
            # TODO: Implement PID for rotation

            ### Pack pose goal into message ###
            target_vel_msg = PoseVelocity() 

            target_vel_msg.twist_linear_x = self.cartesian_vel_target[0]
            target_vel_msg.twist_linear_y = self.cartesian_vel_target[1]
            target_vel_msg.twist_linear_z = self.cartesian_vel_target[2] 

            target_vel_msg.twist_angular_x = rotation_vel_target[0]
            target_vel_msg.twist_angular_y = rotation_vel_target[1]
            target_vel_msg.twist_angular_z = rotation_vel_target[2]

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
            self.get_logger().info("Target rot: %0.3f, %0.3f, %0.3f, %0.3f" % (self.rotation_quaternion_target[0], self.rotation_quaternion_target[1], self.rotation_quaternion_target[2], self.rotation_quaternion_target[3]))
        if separator:
            self.get_logger().info("------------------------------------------------------")

        pass

    def find_linear_velocity(self, joy_msg):

        # Target vel update
        cartesian_vel_target = np.zeros(3)
        cartesian_vel_target[0] = (joy_msg.axes[0] ) # Joystick left horizontal
        cartesian_vel_target[1] = ((joy_msg.axes[2] - joy_msg.axes[5]) ) # Trigger difference
        cartesian_vel_target[2] = (joy_msg.axes[1] ) # Joystick left vertical
        

        # Normalise and multiply by max velocity
        if np.linalg.norm(cartesian_vel_target) > 1:
            #cartesian_vel_target = MAX_LINEAR_VELOCITY * cartesian_vel_target / np.linalg.norm(cartesian_vel_target)
            cartesian_vel_target = MAX_LINEAR_VELOCITY * cartesian_vel_target #/ np.linalg.norm(cartesian_vel_target)

        return cartesian_vel_target
    
    def find_rotation_pose(self, joy_msg):
        # Define the target
        # TODO 


        # For now, simply look forwards
        quaternion_target = np.array([0.707, 0.707, 0, 0]) # Pointing up w, x, y, z # Fine
        #quaternion_target = np.array([0.5, 0.5, 0.5, -0.5]) # Pointing to user  # Goes crazy
        #quaternion_target = np.array([0.5, 0.5, -0.5, 0.5]) # w, x, y, z # Goes crazy
        


        self.publish_rotation_target(quaternion_target)
        return quaternion_target
    
    def find_rotation_velocity(self, quaternion_target, quaternion_current):

        # Find quaternion difference
        quaternion_diff = (quaternion_target) * (quaternion_current).inv()

        # Get magnitude of rotation
        mag = 2*np.arccos(quaternion_diff.as_quat()[3]) # w
        self.get_logger().info(f"Rotation magnitude is: {mag}") 

        # TODO Clamp max angular velocity or tracking. If angle is extreme ignore   
        # https://math.stackexchange.com/questions/3219887/downscale-quaternion
        MAX_ANGLE_DIFFERENCE = 6.0
        if mag > MAX_ANGLE_DIFFERENCE:
            self.get_logger().info("ABOVE MAX ROTATION")
        
            new_mag = MAX_ANGLE_DIFFERENCE
            #quaternion_diff =slerp(quaternion_diff, 1, new_mag/mag)
            pass

        # Convert to euler
        convention = "XZY" # Original
        convention = "xzy" # Seems to work better?
        convention = "xyz" # As per the docs
        

        self.rotation_euler_target = quaternion_diff.as_euler(convention, degrees=False) # Matches docs
        self.get_logger().info(f"Rotation euler target: {quaternion_diff.as_euler(convention, degrees=True)}")

        # Find angular velocity
        euler_velocity_target = self.rotation_euler_target * ANGULAR_Z_VELOCITY
        #euler_velocity_target[2] = -euler_velocity_target[2] #Original

        # Log results
        self.get_logger().info(f"Rotation quaternion target: {quaternion_target.as_quat()}")
        self.get_logger().info(f"Rotation quaternion current: {quaternion_current.as_quat()}")
        self.get_logger().info(f"Rotation quaternion diff: {quaternion_diff.as_quat()}")
        
        self.get_logger().info(f"Rotation euler target: {quaternion_diff.as_euler(convention, degrees=True)}")
        self.get_logger().info(f"---")

        return euler_velocity_target

    def publish_rotation_target(self, quaternion_target):
        dynamic_transform = TransformStamped()
        dynamic_transform.header.stamp = self.get_clock().now().to_msg()
        dynamic_transform.header.frame_id = 'j2n6s300_link_base' # From global frame
        dynamic_transform.child_frame_id = 'j2n6s300_link_6_target'

        VISUAL_OFFSET = [0.05,-0.05,0] 
        dynamic_transform.transform.translation.x = self.current_pose.pose.position.x + VISUAL_OFFSET[0]
        dynamic_transform.transform.translation.y = self.current_pose.pose.position.y + VISUAL_OFFSET[1]
        dynamic_transform.transform.translation.z = self.current_pose.pose.position.z + VISUAL_OFFSET[2]

        dynamic_transform.transform.rotation.w = quaternion_target[0]
        dynamic_transform.transform.rotation.x = quaternion_target[1]
        dynamic_transform.transform.rotation.y = quaternion_target[2]
        dynamic_transform.transform.rotation.z = quaternion_target[3]

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