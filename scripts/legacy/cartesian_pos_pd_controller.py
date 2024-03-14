import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node

from kinova_msgs.action import ArmPose
from kinova_msgs.msg import PoseVelocity
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose
from rclpy.action import ActionClient

from tf2_ros import TransformListener, Buffer
import tf2_ros
from geometry_msgs.msg import TransformStamped

# Standard library imports
from math import sqrt
import numpy as np
import math

# from tf2_ros import TransformStamped
#import tf_transformations as tf

# Constants for max velocities
MAX_POSITION_CHANGE = 0.002
MAX_ROTATION_CHANGE = 0.0001
ANGULAR_Z_VELOCITY = 1.0  # Define this as per your requirements

K_P = 1.0  # Proportional gain
K_D = 0.010  # Derivative gain
REFRESH_RATE = 100.0

# ros2 service call /j2n6s300_driver/in/set_torque_control_mode kinova_msgs/srv/SetTorqueControlMode state:\ 1\
# ros2 service call /j2n6s300_driver/in/start kinova_msgs/srv/Start {}\ 

class JacoController(Node):
    def __init__(self):

        super().__init__('jaco_simple_pid_controller')

        # Create subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.update_target_pose, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_pose', self.update_current_pose, 10)
        self.force_sub = self.create_subscription(WrenchStamped, '/j2n6s300_driver/out/tool_wrench', self.update_current_force, 10)

        # Create tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.update = self.create_timer(1.0/REFRESH_RATE, self.update_controller)



        # Create publishers
        #self.force_pub = self.create_publisher(PoseStamped, '/j2n6s300_driver/in/cartesian_force', 10)
        self.vel_pub = self.create_publisher(PoseVelocity, '/j2n6s300_driver/in/cartesian_velocity', 1)

        # Create action clients
        #self.pose_client = ActionClient(self, ArmPose, '/j2n6s300_driver/tool_pose')
        #self.force_client = ActionClient(self, ArmPose, '/j2n6s300_driver/tool_wrench')
        
        
        self.get_logger().info('JacoController has been started')

        self.current_pose_stamped = PoseStamped()  # Initialize current pose
        self.current_pose_stamped.header.frame_id = 'j2n6s300_link_base'
        self.current_pose_stamped.header.stamp = self.get_clock().now().to_msg()

        # Initialize pose goal  
        default_cartesian_vel = [0,0,0,0,0,0] # default home in unit mq
        
        # PD controller parameters
        #self.Kd = 0.1  # Derivative gain

        # Robot state
        self.target_pose = None
        self.current_pose = None
        self.current_vel = None
        self.prev_error = None
        self.start_pose = None

    def update_current_pose(self, pose_msg):
        self.current_pose = pose_msg

    def update_current_force(self, force_msg):
        self.current_force = force_msg 


    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error('Error getting transform')
            return None


    def update_target_pose(self, joy_msg): 
        if self.current_pose is None:
            return
        elif self.target_pose is None:
            self.target_pose = self.current_pose

            self.start_pose = self.current_pose
        
        try:
            # Map joystick values to cartesian translation changes
            pose_goal = self.target_pose
            pose_goal.pose.position.x += (joy_msg.axes[0] * MAX_POSITION_CHANGE)
            pose_goal.pose.position.y += (joy_msg.axes[4] * MAX_POSITION_CHANGE)
            pose_goal.pose.position.z += (joy_msg.axes[1] * MAX_POSITION_CHANGE)

            # For rotation, get pose of the camera
            transform = self.get_transform('j2n6s300_link_base', 'camera')

            # adjust / offset the rotation of the camera
            adjusted_transform = self.offset_transform(transform)

            # This is the new rotation target
            pose_goal.pose.orientation.x = adjusted_transform.transform.rotation.x
            pose_goal.pose.orientation.y = adjusted_transform.transform.rotation.y
            pose_goal.pose.orientation.z = adjusted_transform.transform.rotation.z
            pose_goal.pose.orientation.w = adjusted_transform.transform.rotation.w


            self.target_pose = pose_goal
            
            # print pose_goal
            #print("Current pose: %0.3f, %0.3f, %0.3f" % (self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z))
            #print("Target pose: %0.3f, %0.3f, %0.3f" % (self.target_pose.pose.orientation.x, self.target_pose.pose.orientation.y, self.target_pose.pose.orientation.z))
            #print("------------------------------------------------------")
 


            # Pack pose goal into action message
            self.target_pose.header.frame_id = 'j2n6s300_link_base'
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {e}')

    def offset_transform(self, transform):
        def rotation_matrix_to_quaternion(m):
            tr = m[0][0] + m[1][1] + m[2][2]
            if tr > 0:
                S = sqrt(tr + 1.0) * 2
                qw = 0.25 * S
                qx = (m[2][1] - m[1][2]) / S
                qy = (m[0][2] - m[2][0]) / S
                qz = (m[1][0] - m[0][1]) / S
            elif (m[0][0] > m[1][1]) and (m[0][0] > m[2][2]):
                S = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
                qw = (m[2][1] - m[1][2]) / S
                qx = 0.25 * S
                qy = (m[0][1] + m[1][0]) / S
                qz = (m[0][2] + m[2][0]) / S
            elif m[1][1] > m[2][2]:
                S = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
                qw = (m[0][2] - m[2][0]) / S
                qx = (m[0][1] + m[1][0]) / S
                qy = 0.25 * S
                qz = (m[1][2] + m[2][1]) / S
            else:
                S = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
                qw = (m[1][0] - m[0][1]) / S
                qx = (m[0][2] + m[2][0]) / S
                qy = (m[1][2] + m[2][1]) / S
                qz = 0.25 * S
            return [qx, qy, qz, qw]
        
        def quaternion_multiply(quaternion1, quaternion2):
            w1, x1, y1, z1 = quaternion1
            w2, x2, y2, z2 = quaternion2
            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
            y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
            z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

            return w, x, y, z

        # make a new transform to return
        result = TransformStamped()

        # Get the original rotation as a quaternion
        original_quaternion = [transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w]
        
        
        self.get_logger().info(f'Original quaternion: {original_quaternion}')

        # Create a rotation matrix for a 180-degree rotation around the x-axis
        angle = 0.0 * np.pi / 180.0
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(angle), -np.sin(angle)],
                                    [0, np.sin(angle), np.cos(angle)]])

        # Convert the rotation matrix to a quaternion
        rotation_quaternion = rotation_matrix_to_quaternion(rotation_matrix)

        # Add the rotation to the original quaternion by multiplying them
        new_quaternion = quaternion_multiply(original_quaternion, rotation_quaternion)

        # Apply the new quaternion to the transform
        result.transform.rotation.x = new_quaternion[1]
        result.transform.rotation.y = new_quaternion[2]
        result.transform.rotation.z = new_quaternion[3]
        result.transform.rotation.w = new_quaternion[0]


        return result

    def quaternion_to_euler(self, quaternion):
        # Unpack the quaternion
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

        # Calculate the Euler angles
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        return roll, pitch, yaw
    
    def update_controller(self):

        if self.target_pose is None or self.current_pose is None: #  or self.current_force is None:
            return
        if self.prev_error is None:
            self.prev_error = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # convert rotation to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(self.current_pose.pose.orientation)        

        # Implement PD controller
            
            

        # P controller
        current_error = [ ( self.target_pose.pose.position.x - self.current_pose.pose.position.x ),
                  ( self.target_pose.pose.position.y - self.current_pose.pose.position.y ),
                  ( self.target_pose.pose.position.z - self.current_pose.pose.position.z ),
                  ( roll - self.current_pose.pose.orientation.x),
                  ( pitch - self.current_pose.pose.orientation.y),
                  ( yaw - self.current_pose.pose.orientation.z) ]
        
        # D controller
        error_dot = [ ( current_error[0] - self.prev_error[0] ) * REFRESH_RATE,
                    ( current_error[1] - self.prev_error[1] ) *REFRESH_RATE,
                    ( current_error[2] - self.prev_error[2] ) * REFRESH_RATE,
                     ( current_error[3] - self.prev_error[3] )*REFRESH_RATE,
                    ( current_error[4] - self.prev_error[4] )*REFRESH_RATE,
                    ( current_error[5] - self.prev_error[5] )*REFRESH_RATE ]
        
        prev_error = current_error
        
        target_vel = [ K_P * current_error[0] + K_D * error_dot[0], 
                      K_P * current_error[1] + K_D * error_dot[1], 
                      K_P * current_error[2] + K_D * error_dot[2],
                      K_P * current_error[3] + K_D * error_dot[3],
                      K_P * current_error[4] + K_D * error_dot[4],
                      K_P * current_error[5] + K_D * error_dot[5],
                     ] # force in unit m/s, rad/s
                      

        #self.get_logger().info("Current pose: %0.3f, %0.3f, %0.3f" % (self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z))
        #self.get_logger().info("Target pose: %0.3f, %0.3f, %0.3f" % (self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z))
        #self.get_logger().info("Error: %0.3f, %0.3f, %0.3f" % (error[0], error[1], error[2]))        
        #self.get_logger().info("Target vel: %0.3f, %0.3f, %0.3f" % (target_vel[3], target_vel[4], target_vel[5]))
        #self.get_logger().info("Current vel: %0.3f, %0.3f, %0.3f" % (self.current_force.pose.position.x, self.current_force.pose.position.y, self.current_force.pose.position.z))
        #self.get_logger().info("------------------------------------------------------")


        print("Current pose: %0.3f, %0.3f, %0.3f" % (self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z))
        print("Target pose: %0.3f, %0.3f, %0.3f" % (roll, pitch, yaw))
        #print("Error: %0.3f, %0.3f, %0.3f" % (error[0], error[1], error[2]))
        #print("Target force: %0.3f, %0.3f, %0.3f" % (target_vel[0], target_vel[1], target_vel[2]))
        #print("Current force: %0.3f, %0.3f, %0.3f" % (self.current_force.pose.position.x, self.current_force.pose.position.y, self.current_force.pose.position.z))
        #print("------------------------------------------------------")

        #self.get_logger().info(f'Force: {target_vel}')

        # Fill message for vel publisher
        target_vel_msg = PoseVelocity() 

        #kinova_msgs.msg.PoseVelocity(twist_linear_x=0.1, twist_linear_y=0.0, twist_linear_z=0.0, twist_angular_x=0.0, twist_angular_y=0.0, twist_angular_z=0.0)

        target_vel_msg.twist_linear_x = target_vel[0]
        target_vel_msg.twist_linear_y = target_vel[1]
        target_vel_msg.twist_linear_z = target_vel[2]
        target_vel_msg.twist_angular_x = target_vel[3]
        target_vel_msg.twist_angular_y = target_vel[4]
        target_vel_msg.twist_angular_z = target_vel[5]


        # Send pose goal to subscriber
        #self.vel_pub.publish(target_vel_msg)



def main():
    rclpy.init()
    try:
        controller = JacoController()
        rclpy.spin(controller)
    except Exception as e:
        controller.get_logger().error(f'Error in main: {e}')
    finally:
        controller.get_logger().info('JacoController has been stopped')
        rclpy.shutdown()

if __name__ == '__main__':
    main()