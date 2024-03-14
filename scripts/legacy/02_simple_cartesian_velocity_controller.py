import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node

from kinova_msgs.action import ArmPose
from kinova_msgs.msg import PoseVelocity
from geometry_msgs.msg import PoseStamped, Pose

# Constants for max velocities
MAX_LINEAR_VELOCITY = 0.4
MAX_ANGULAR_VELOCITY = 0.1
ANGULAR_Z_VELOCITY = 1.0  # Define this as per your requirements

REFRESH_RATE = 50.0

class JacoController(Node):
    def __init__(self):
        super().__init__('jaco_simple_controller')
        self.get_logger().info('JacoController has been started')

        # Define subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Define publishers
        self.vel_pub = self.create_publisher(PoseVelocity, '/j2n6s300_driver/in/cartesian_velocity', 1)

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

            # Target vel update
            target_vel_msg = PoseVelocity() 
            target_vel_msg.twist_linear_x = (joy_msg.axes[0] * MAX_LINEAR_VELOCITY)
            target_vel_msg.twist_linear_y = ((joy_msg.axes[2] - joy_msg.axes[5]) * MAX_LINEAR_VELOCITY)
            target_vel_msg.twist_linear_z = (joy_msg.axes[1] * MAX_LINEAR_VELOCITY)

            target_vel_msg.twist_angular_x = 1.0*(joy_msg.buttons[5] - joy_msg.buttons[4])  
            target_vel_msg.twist_angular_y = (joy_msg.axes[3] ) 
            target_vel_msg.twist_angular_z = joy_msg.axes[4]

            # Send pose goal to action server
            self.get_logger().info(f'Publishing target_vel_msg: {target_vel_msg}')
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