#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StrMsg
from geometry_msgs.msg import TwistStamped, PoseStamped, Point
from kinova_msgs.msg import PoseVelocityWithFingerVelocity, FingerPosition
from geometry_msgs.msg import WrenchStamped
from scipy.spatial.transform import Rotation
import numpy as np

REFRESH_RATE = 100.0

class JacoController(Node):
    def __init__(self):
        super().__init__('jaco_controller_reworked')
        self.get_logger().info("JacoController (reworked) starting...")

        # Subscriptions: now listening to teleop topics instead of /joy
        self.vel_sub = self.create_subscription(TwistStamped, '/teleop/cartesian_velocity', self.vel_cb, 10)
        self.discrete_pose_sub = self.create_subscription(PoseStamped, '/teleop/discrete_pose', self.discrete_cb, 10)
        self.sys_sub = self.create_subscription(StrMsg, '/teleop/system', self.system_cb, 10)
        self.mode_sub = self.create_subscription(StrMsg, '/teleop/current_mode', self.mode_cb, 10)

        # Existing sensors
        self.pose_sub = self.create_subscription(PoseStamped, '/j2n6s300_driver/out/tool_pose', self.update_current_pose, 10)
        self.force_sub = self.create_subscription(WrenchStamped, '/j2n6s300_driver/out/tool_wrench', self.update_current_force, 10)
        try:
            self.finger_sub = self.create_subscription(FingerPosition, '/j2n6s300_driver/out/finger_position', self.update_current_finger_pose, 10)
        except Exception:
            # Keep node working even if kinova_msgs types are missing locally
            self.get_logger().warn("FingerPosition message not available locally - replace import if needed.")

        # Publisher to Kinova driver (cartesian velocity with fingers)
        self.vel_pub = self.create_publisher(PoseVelocityWithFingerVelocity, '/j2n6s300_driver/in/cartesian_velocity_with_finger_velocity', 1)

        # Testing publishers (as in original)
        self.test_target_vel_pub = self.create_publisher(Point, '/test/requested_vel_pub', 1)
        self.test_pid_vel_pub = self.create_publisher(Point, '/test/pid_target_vel_pub', 1)
        self.test_measured_vel_pub = self.create_publisher(Point, '/test/measured_vel_pub', 1)
        self.haptics_action_pub = self.create_publisher(StrMsg, "/haptic_feedback_robot_string", 1)

        # Internal state
        self.current_mode = "translation"
        self.latest_twist = None  # last velocity command received
        self.pending_discrete_pose = None  # last discrete rotation command received
        self.current_pose = None
        self.current_vel = np.zeros(3)

        # Parameters (some defaults kept from your original)
        self.declare_parameter("discretise_rotation", True)
        self.declare_parameter("quantisation_degrees", 30.0)
        self.declare_parameter("pid_enabled", True)
        self.quantisation_degrees = self.get_parameter("quantisation_degrees").value
        self.discretise_rotation = self.get_parameter("discretise_rotation").value

        # Timer to run update_controller at REFRESH_RATE
        self.timer = self.create_timer(1.0 / REFRESH_RATE, self.update_controller)

        # Small filters & PID placeholders (you can reuse your PID class)
        self.prev_time = self.get_clock().now()

    def mode_cb(self, msg: StrMsg):
        self.current_mode = msg.data
        self.get_logger().info(f"JacoController: current_mode set to {self.current_mode}")

    def vel_cb(self, msg: TwistStamped):
        # Save last twist; update loop will check mode and publish values or ignore
        self.latest_twist = msg

    def discrete_cb(self, msg: PoseStamped):
        # Here discrete pose encodes a rotation delta in orientation quaternion
        # We store it and apply during update_controller if mode allows
        self.pending_discrete_pose = msg

    def system_cb(self, msg: StrMsg):
        cmd = msg.data
        self.get_logger().info(f"Received system command: {cmd}")
        if cmd == "reset_pose":
            self.reset_to_home()
        # handle other system commands as needed

    def update_current_pose(self, msg: PoseStamped):
        self.current_pose = msg

    def update_current_force(self, msg: WrenchStamped):
        # optional use later
        pass

    def update_current_finger_pose(self, msg: FingerPosition):
        # optional use later
        pass

    def reset_to_home(self):
        # Implementation depends on your robot API.
        # For now just log and set a simple target pose equal to current pose
        self.get_logger().info("Reset to home requested (not implemented).")

    def apply_quantized_rotation(self, quat_delta):
        """
        quat_delta: quaternion representing the incremental rotation to apply
        This function:
         - reads current_pose orientation
         - composes the rotation with the quantisation to nearest step (if enabled)
         - sets a new target pose (keeps position)
        Returns the target PoseStamped (relative)
        """
        if self.current_pose is None:
            self.get_logger().warn("No current_pose available; cannot apply discrete rotation")
            return None

        # current orientation
        cur_q = self.current_pose.pose.orientation
        cur_rot = Rotation.from_quat([cur_q.x, cur_q.y, cur_q.z, cur_q.w])

        delta_rot = Rotation.from_quat([quat_delta.x, quat_delta.y, quat_delta.z, quat_delta.w])

        # compose: new_rot = cur_rot * delta_rot (apply delta in ee frame)
        new_rot = cur_rot * delta_rot

        if self.discretise_rotation and self.quantisation_degrees > 0:
            # convert to euler, quantise each axis to nearest step (in degrees)
            euler = new_rot.as_euler('xyz', degrees=True)
            step = self.quantisation_degrees
            quantised = np.round(euler / step) * step
            new_rot = Rotation.from_euler('xyz', quantised, degrees=True)

        q = new_rot.as_quat()  # x,y,z,w
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.current_pose.header.frame_id
        # keep current position, change orientation
        target.pose.position = self.current_pose.pose.position
        target.pose.orientation.x = float(q[0])
        target.pose.orientation.y = float(q[1])
        target.pose.orientation.z = float(q[2])
        target.pose.orientation.w = float(q[3])

        return target

    def update_controller(self):
        # Called at REFRESH_RATE. Decide what to publish to Kinova driver.

        # 1) Handle discrete rotation if available & permitted by mode
        if self.pending_discrete_pose is not None:
            if self.current_mode in ("rotation", "discrete"):
                # apply rotation delta to current pose and set as target orientation
                delta_q = self.pending_discrete_pose.pose.orientation
                target_pose = self.apply_quantized_rotation(delta_q)
                if target_pose is not None:
                    # For a go-to-pose action you might want to call an action server.
                    # Here we simply set the internal target pose (or could publish)
                    self.get_logger().info("Applied discrete rotation; new target pose computed.")
                    # Clear pending after applied
                    self.pending_discrete_pose = None
                    # Optionally: publish the new target as a PoseStamped for other nodes
                    # self.some_pose_pub.publish(target_pose)
                else:
                    self.pending_discrete_pose = None
            else:
                # mode doesn't allow discrete rotations
                self.get_logger().info("Ignored discrete rotation because current mode doesn't allow it.")
                self.pending_discrete_pose = None

        # 2) Handle continuous velocities
        if self.latest_twist is not None:
            if self.current_mode != "translation":
                # Ignore velocity commands when not in translation
                # (optionally, you could allow angular velocities in rotation mode)
                # For now, ignore them.
                # self.get_logger().info("Ignored velocity because mode != translation")
                pass
            else:
                # Convert TwistStamped -> PoseVelocityWithFingerVelocity
                t = self.latest_twist
                pv = PoseVelocityWithFingerVelocity()

                # linear
                pv.twist_linear_x = getattr(t.twist.linear, 'x', 0.0)
                pv.twist_linear_y = getattr(t.twist.linear, 'y', 0.0)
                pv.twist_linear_z = getattr(t.twist.linear, 'z', 0.0)

                # angular
                pv.twist_angular_x = getattr(t.twist.angular, 'x', 0.0)
                pv.twist_angular_y = getattr(t.twist.angular, 'y', 0.0)
                pv.twist_angular_z = getattr(t.twist.angular, 'z', 0.0)

                # fingers (zero here)
                try:
                    pv.finger1 = 0.0
                    pv.finger2 = 0.0
                    pv.finger3 = 0.0
                except Exception:
                    pass

                self.vel_pub.publish(pv)
                # debug
                try:
                    self.test_target_vel_pub.publish(Point(x=pv.twist_linear_x, y=pv.twist_linear_y, z=pv.twist_linear_z))
                except Exception:
                    pass

        # end update_controller

def main(args=None):
    rclpy.init(args=args)
    node = JacoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
