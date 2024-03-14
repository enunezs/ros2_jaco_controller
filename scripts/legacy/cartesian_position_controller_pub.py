import rclpy
from sensor_msgs.msg import Joy
from kinova_msgs.msg import JointAngles
from rclpy.node import Node

from kinova_msgs.action import ArmPose
from kinova_msgs.msg import PoseVelocity
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.action import ActionClient

# Constants for max velocities
MAX_POSITION_CHANGE = 0.001
MAX_ROTATION_CHANGE = 0.001
ANGULAR_Z_VELOCITY = 1.0  # Define this as per your requirements

K_P = 100  # Proportional gain

# ros2 service call /j2n6s300_driver/in/set_torque_control_mode kinova_msgs/srv/SetTorqueControlMode state:\ 1\
# ros2 service call /j2n6s300_driver/in/start kinova_msgs/srv/Start {}\ 

class JacoController(Node):
    def __init__(self):

        super().__init__('jaco_simple_pid_controller')

        # Create subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.update_target_pose, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_pose', self.update_current_pose, 10)
        self.force_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_wrench', self.update_current_force, 10)
        
        self.update = self.create_timer(1.0/100, self.update_controller)

        # Create publishers
        #self.force_pub = self.create_publisher(PoseStamped, '/j2n6s300_driver/in/cartesian_force', 10)
        self.vel_pub = self.create_publisher(PoseStamped, '/j2n6s300_driver/in/cartesian_velocity', 10)
        #self.pos_pub = self.create_publisher(PoseStamped, '/j2n6s300_driver/in/cartesian_velocity', 10)

        # Create action clients
        self.pose_client = ActionClient(self, ArmPose, '/j2n6s300_driver/tool_pose')
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

    def update_current_pose(self, pose_msg):
        self.current_pose = pose_msg

    def update_current_force(self, force_msg):
        self.current_force = force_msg 

    def update_target_pose(self, joy_msg): 
        if self.current_pose is None:
            return
        elif self.target_pose is None:
            self.target_pose = self.current_pose
        
        try:
            # Map joystick values to pose changes
            pose_goal = self.target_pose
            pose_goal.pose.position.x += (joy_msg.axes[0] * MAX_POSITION_CHANGE)
            pose_goal.pose.position.y += ((joy_msg.axes[4] - joy_msg.axes[5]) * MAX_POSITION_CHANGE)
            pose_goal.pose.position.z += (joy_msg.axes[1] * MAX_POSITION_CHANGE)
            #pose_goal.pose.orientation.x = min(joy_msg.axes[4] * MAX_ROTATION_CHANGE, MAX_ROTATION_CHANGE)
            #pose_goal.pose.orientation.y = min(joy_msg.axes[3] * MAX_ROTATION_CHANGE, MAX_ROTATION_CHANGE)

            # Set z to shoulder buttons
            # Forward quaternion
            pose_goal.pose.orientation.w = 0.0
            pose_goal.pose.orientation.x = 0.0
            pose_goal.pose.orientation.y = 1.0
            pose_goal.pose.orientation.z = 0.0

            self.target_pose = pose_goal

            # Pack pose goal into action message
            self.target_pose.header.frame_id = 'j2n6s300_link_base'
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {e}')


    def update_controller(self):

        if self.target_pose is None or self.current_pose is None: #  or self.current_force is None:
            return
        
        # P controller
        error = [ ( self.target_pose.pose.position.x - self.current_pose.pose.position.x ),
                  ( self.target_pose.pose.position.y - self.current_pose.pose.position.y ),
                  ( self.target_pose.pose.position.z - self.current_pose.pose.position.z ) ]
        
        target_vel = [ K_P * error[0], K_P * error[1], K_P * error[2] , 0.0, -1.0, 0.0, 0.0 ] # force in unit m/s, rad/s

        #self.get_logger().info("Current pose: %0.3f, %0.3f, %0.3f" % (self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z))
        #self.get_logger().info("Target pose: %0.3f, %0.3f, %0.3f" % (self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z))
        #self.get_logger().info("Error: %0.3f, %0.3f, %0.3f" % (error[0], error[1], error[2]))        
        self.get_logger().info("Target vel: %0.3f, %0.3f, %0.3f" % (target_vel[0], target_vel[1], target_vel[2]))
        #self.get_logger().info("Current vel: %0.3f, %0.3f, %0.3f" % (self.current_force.pose.position.x, self.current_force.pose.position.y, self.current_force.pose.position.z))
        self.get_logger().info("------------------------------------------------------")


        #print("Current pose: %0.3f, %0.3f, %0.3f" % (self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z))
        #print("Target pose: %0.3f, %0.3f, %0.3f" % (self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z))
        #print("Error: %0.3f, %0.3f, %0.3f" % (error[0], error[1], error[2]))
        #print("Target force: %0.3f, %0.3f, %0.3f" % (target_vel[0], target_vel[1], target_vel[2]))
        #print("Current force: %0.3f, %0.3f, %0.3f" % (self.current_force.pose.position.x, self.current_force.pose.position.y, self.current_force.pose.position.z))
        #print("------------------------------------------------------")


        #self.get_logger().info(f'Force: {target_vel}')

        # Fill message for vel publisher
        target_vel_msg = PoseStamped()
        target_vel_msg.header.frame_id = 'j2n6s300_link_base'
        target_vel_msg.header.stamp = self.get_clock().now().to_msg()
        target_vel_msg.pose.position.x = target_vel[0]
        target_vel_msg.pose.position.y = target_vel[1]
        target_vel_msg.pose.position.z = target_vel[2]
        target_vel_msg.pose.orientation.w = target_vel[3]
        target_vel_msg.pose.orientation.x = target_vel[4]
        target_vel_msg.pose.orientation.y = target_vel[5]
        target_vel_msg.pose.orientation.z = target_vel[6]


        # Send pose goal to subscriber
        self.vel_pub.publish(self.target_pose)


        # Send pose goal to action server
        #self.pose_client.wait_for_server()
        #self.get_logger().info(f'Pose goal is being sent: {self.target_pose}')
        #self.pose_client.send_goal_async(ArmPose.Goal(pose=self.target_pose))


        #self.pose_client.wait_for_server()
        #self.get_logger().info(f'Wrench goal is being sent: {self.target_pose}')
        #self.pose_client.send_goal_async(ArmPose.Goal(pose=self.target_pose))



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