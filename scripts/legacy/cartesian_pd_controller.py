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

K_P = 10000  # Proportional gain

# ros2 service call /j2n6s300_driver/in/set_torque_control_mode kinova_msgs/srv/SetTorqueControlMode state:\ 1\
# ros2 service call /j2n6s300_driver/in/start kinova_msgs/srv/Start {}\ 

class JacoController(Node):
    def __init__(self):

        super().__init__('jaco_simple_pid_controller')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.update_target_pose, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_pose', self.update_current_pose, 10)
        self.force_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_wrench', self.update_current_force, 10)
        self.update = self.create_timer(1.0/100, self.update_controller)

        #self.pose_client = ActionClient(self, ArmPose, '/j2n6s300_driver/tool_pose')
        #self.force_client = ActionClient(self, ArmPose, '/j2n6s300_driver/tool_wrench')
        # create force publisher
        self.force_pub = self.create_publisher(PoseStamped, '/j2n6s300_driver/in/cartesian_force', 10)
        
        self.get_logger().info('JacoController has been started')

        self.current_pose_stamped = PoseStamped()  # Initialize current pose
        self.current_pose_stamped.header.frame_id = 'j2n6s300_link_base'
        self.current_pose_stamped.header.stamp = self.get_clock().now().to_msg()

        # Initialize pose goal  
        default_cartesian_pose = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq
        self.current_pose_stamped.pose.position.x = default_cartesian_pose[0]
        self.current_pose_stamped.pose.position.y = default_cartesian_pose[1]
        self.current_pose_stamped.pose.position.z = default_cartesian_pose[2]
        
        # Forward quaternion
        self.current_pose_stamped.pose.orientation.w = 0.0
        self.current_pose_stamped.pose.orientation.x = -1.0
        self.current_pose_stamped.pose.orientation.y = 0.0
        self.current_pose_stamped.pose.orientation.z = 0.0

        # PD controller parameters
        #self.Kd = 0.1  # Derivative gain

        # Robot state
        self.target_pose = None
        self.current_pose = None
        self.current_force = None

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
            pose_goal.pose.orientation.x = -1.0
            pose_goal.pose.orientation.y = 0.0
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
        
        target_force = [ K_P * error[0], K_P * error[1], K_P * error[2] , 0.0, -1.0, 0.0, 0.0 ] # force in unit N, Nm

        #self.get_logger().info("Current pose: %0.3f, %0.3f, %0.3f" % (self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z))
        #self.get_logger().info("Target pose: %0.3f, %0.3f, %0.3f" % (self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z))
        #self.get_logger().info("Error: %0.3f, %0.3f, %0.3f" % (error[0], error[1], error[2]))        
        self.get_logger().info("Target force: %0.3f, %0.3f, %0.3f" % (target_force[0], target_force[1], target_force[2]))
        #self.get_logger().info("Current force: %0.3f, %0.3f, %0.3f" % (self.current_force.pose.position.x, self.current_force.pose.position.y, self.current_force.pose.position.z))
        self.get_logger().info("------------------------------------------------------")


        #print("Current pose: %0.3f, %0.3f, %0.3f" % (self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z))
        #print("Target pose: %0.3f, %0.3f, %0.3f" % (self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z))
        #print("Error: %0.3f, %0.3f, %0.3f" % (error[0], error[1], error[2]))
        #print("Target force: %0.3f, %0.3f, %0.3f" % (target_force[0], target_force[1], target_force[2]))
        #print("Current force: %0.3f, %0.3f, %0.3f" % (self.current_force.pose.position.x, self.current_force.pose.position.y, self.current_force.pose.position.z))
        #print("------------------------------------------------------")


        #self.get_logger().info(f'Force: {target_force}')

        # Fill message for force publisher
        target_force_msg = PoseStamped()
        target_force_msg.header.frame_id = 'j2n6s300_link_base'
        target_force_msg.header.stamp = self.get_clock().now().to_msg()
        target_force_msg.pose.position.x = target_force[0]
        target_force_msg.pose.position.y = target_force[1]
        target_force_msg.pose.position.z = target_force[2]
        target_force_msg.pose.orientation.w = target_force[3]
        target_force_msg.pose.orientation.x = target_force[4]
        target_force_msg.pose.orientation.y = target_force[5]
        target_force_msg.pose.orientation.z = target_force[6]


        # Send pose goal to subscriber
        self.force_pub.publish(target_force_msg)
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