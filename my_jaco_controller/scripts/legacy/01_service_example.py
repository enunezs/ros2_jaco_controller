import rclpy
from sensor_msgs.msg import Joy
from kinova_msgs.msg import JointAngles
from rclpy.node import Node

from kinova_msgs.action import ArmPose
from kinova_msgs.msg import PoseVelocity
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.action import ActionClient

# Constants for max velocities
MAX_POSITION_CHANGE = 0.1
MAX_ROTATION_CHANGE = 0.1
ANGULAR_Z_VELOCITY = 1.0  # Define this as per your requirements

class JacoController(Node):
    def __init__(self):
        super().__init__('jaco_simple_controller')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        #self.pose_client = ActionClient(self, ArmPose, '/ /tool_pose')
        self.get_logger().info('JacoController has been started')


        self.current_pose_stamped = PoseStamped()  # Initialize current pose
        self.current_pose_stamped.header.frame_id = 'j2n6s300_link_base'
        self.current_pose_stamped.header.stamp = self.get_clock().now().to_msg()

        # Initialize pose goal
        self.current_pose_stamped.pose.position.x = 0.0
        self.current_pose_stamped.pose.position.y = 50.0
        self.current_pose_stamped.pose.position.z = 50.0
        # Forward quaternion
        self.current_pose_stamped.pose.orientation.w = 0.0
        self.current_pose_stamped.pose.orientation.x = -1.0
        self.current_pose_stamped.pose.orientation.y = 0.0
        self.current_pose_stamped.pose.orientation.z = 0.0

        #self.current_pose_stamped.header.seq = 0


    def joy_callback(self, joy_msg):
        try:
            # Map joystick values to pose changes
            pose_goal = Pose()
            pose_goal.position.x += (joy_msg.axes[0] * MAX_POSITION_CHANGE)
            pose_goal.position.y += ((joy_msg.axes[2] - joy_msg.axes[5]) * MAX_POSITION_CHANGE)
            pose_goal.position.z += (joy_msg.axes[1] * MAX_POSITION_CHANGE)
            #pose_goal.pose.orientation.x = min(joy_msg.axes[4] * MAX_ROTATION_CHANGE, MAX_ROTATION_CHANGE)
            #pose_goal.pose.orientation.y = min(joy_msg.axes[3] * MAX_ROTATION_CHANGE, MAX_ROTATION_CHANGE)

            # Set z to shoulder buttons
            # Forward quaternion
            pose_goal.orientation.w = 0.0
            pose_goal.orientation.x = -1.0
            pose_goal.orientation.y = 0.0
            pose_goal.orientation.z = 0.0

            # Pack pose goal into action message
            self.current_pose_stamped.header.frame_id = 'j2n6s300_link_base'
            self.current_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            #self.current_pose_stamped.header.seq += 1
            self.current_pose_stamped.pose = pose_goal

            # Send pose goal to action server
            self.pose_client.wait_for_server()
            self.get_logger().info(f'Pose goal is being sent: {self.current_pose_stamped}')
            self.pose_client.send_goal_async(ArmPose.Goal(pose=self.current_pose_stamped))

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