import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node

from kinova_msgs.action import ArmPose
from kinova_msgs.msg import PoseVelocity, PoseVelocityWithFingerVelocity
from geometry_msgs.msg import PoseStamped, Pose

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation # IMPORTANT, USES x,y,z,w
import numpy as np

# Constants for max velocities
MAX_LINEAR_VELOCITY = 0.10
MAX_ANGULAR_VELOCITY = 1.0
MAX_FINGER_VELOCITY = 2000.0

REFRESH_RATE = 50.0

class JacoController(Node):
    def __init__(self):
        super().__init__('jaco_simple_controller')
        self.get_logger().info('JacoController has been started')

        # Define subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Listen to align rotation with                
        self.tf_target_buffer = Buffer()
        self.tf_target_listener = TransformListener(self.tf_target_buffer, self)

        # Define publishers
        self.vel_pub = self.create_publisher(PoseVelocityWithFingerVelocity, '/j2n6s300_driver/in/cartesian_velocity_with_finger_velocity', 1)

        self.update = self.create_timer(1.0/REFRESH_RATE, self.update_controller)

        self.joy_msg = None

    def joy_callback(self, joy_msg): 
        self.joy_msg = joy_msg

    def update_controller(self):

        # Only update if we have all needed data
        if self.joy_msg is None:
            return
        
        joy_msg = self.joy_msg
        try:

            # --- Relative Velocity Control ---
            # Target vel update
            # Apply velocity relative to the end effector.

            global_vel = np.array([joy_msg.axes[0], joy_msg.axes[5] - joy_msg.axes[2] , joy_msg.axes[1], 1])

            target_frame = 'j2n6s300_end_effector'
            source_frame = 'j2n6s300_link_base'

            try:
                transform: TransformStamped = self.tf_target_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                # Turn from quaternion to rotation matrix
                rot = Rotation.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
                local_vel = rot.inv().apply(global_vel[0:3])

            except Exception as e:
                self.get_logger().error(f'Error in lookup_transform: {e}')
                local_vel = global_vel[0:3]

            target_vel_msg = PoseVelocityWithFingerVelocity() 
            target_vel_msg.twist_linear_x = (local_vel[0] * MAX_LINEAR_VELOCITY)
            target_vel_msg.twist_linear_y = (local_vel[1] * MAX_LINEAR_VELOCITY)
            target_vel_msg.twist_linear_z = (local_vel[2] * MAX_LINEAR_VELOCITY)

            # --- Relative Rotation Control ---
            target_vel_msg.twist_angular_x = (joy_msg.buttons[5] - joy_msg.buttons[4]) * MAX_ANGULAR_VELOCITY  
            target_vel_msg.twist_angular_y = -joy_msg.axes[3] * MAX_ANGULAR_VELOCITY
            target_vel_msg.twist_angular_z = -joy_msg.axes[4] * MAX_ANGULAR_VELOCITY

            # --- Finger Control ---
            # A - B
            target_vel_msg.finger1 = (joy_msg.buttons[1] - joy_msg.buttons[0]) * MAX_FINGER_VELOCITY
            target_vel_msg.finger2 = (joy_msg.buttons[1] - joy_msg.buttons[0]) * MAX_FINGER_VELOCITY
            target_vel_msg.finger3 = (joy_msg.buttons[1] - joy_msg.buttons[0]) * MAX_FINGER_VELOCITY
            self.get_logger().info(f'Publishing target_vel_msg: {target_vel_msg.finger1} {target_vel_msg.finger2} {target_vel_msg.finger3}')

            # Send pose goal to action server
            #self.get_logger().info(f'Publishing target_vel_msg: {target_vel_msg}')
            self.vel_pub.publish(target_vel_msg)

            #self.pose_client.send_goal(pose_goal)
        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {e}')

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