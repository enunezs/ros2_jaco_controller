#!/usr/bin/env python3
# mode_manager.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path

class ModeManager(Node):
    """
    Centralized mode manager.
    Subscribes to /teleop/mode_command (String) and publishes /teleop/current_mode (String).
    
    Manages high-level control modes:
    - translation: Continuous cartesian translation control
    - rotation: Continuous rotation control
    - discrete: Waypoint-based autonomous execution
    - system: System control mode (stops all motion)
"""

    VALID_MODES = ["translation", "rotation", "discrete", "system"]

    def __init__(self):
        super().__init__('mode_manager')
        self.get_logger().info("ModeManager starting...")
        self.current_mode = "translation"

        # Subscribers
        self.mode_cmd_sub = self.create_subscription(String, '/teleop/mode_command', self.mode_command_cb, 10)
        
        # Here or on robot?
        self.waypoint_sub = self.create_subscription(Path, '/teleop/waypoint_path', self.waypoint_cb, 10)

        # Publishers 
        self.mode_pub = self.create_publisher(String, '/teleop/current_mode', 10)

        # Publish initial mode
        self.publish_mode()

    def publish_mode(self):
        """Publish current mode to /teleop/current_mode"""
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)
        # self.get_logger().info(f"Published current_mode: {self.current_mode}")

    # Discrete modes override continous. Continuous can only resume when discrete is finished.
    def waypoint_cb(self, msg: Path):
        
        if self.current_mode != "discrete":
            # cHANGE TO DISCRETE MODE
            self.get_logger().info("Waypoint path received. Switching to discrete mode.")
            self.current_mode = "discrete"
            self.publish_mode()

        if self.current_mode == "discrete" and len(msg.poses) == 0:
            self.get_logger().info("Waypoint path completed. Resuming previous mode.")
            self.current_mode = "translation"  # or store previous mode if needed
            self.publish_mode()

    # TODO: Add mutex to avoid changes on when running a discrete unfinished action
    def mode_command_cb(self, msg: String):
        """
        Handle mode change requests.
        
        Supports:
        - "toggle_next": Cycle through modes sequentially
        - Direct mode name: Switch to specific mode
        """
                
        requested = msg.data
        self.get_logger().info(f"Mode command received: {requested}")

        if self.current_mode == "discrete":
            self.get_logger().info("Currently in discrete mode; ignoring mode change request.")
            return


        if requested == "toggle_next":
            # rotate through modes
            idx = self.VALID_MODES.index(self.current_mode)
            idx = (idx + 1) % len(self.VALID_MODES)
            self.current_mode = self.VALID_MODES[idx]
            self.get_logger().info(f"Toggled mode -> {self.current_mode}")
            self.publish_mode()
            return

        if requested in self.VALID_MODES:
            if requested != self.current_mode:
                self.current_mode = requested
                self.get_logger().info(f"Mode changed -> {self.current_mode}")
                self.publish_mode()
            else:
                self.get_logger().info("Requested mode is already active.")
        else:
            self.get_logger().warn(f"Unknown mode requested: {requested}")


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    else:
        node.get_logger().info("ModeManager stopped unexpectedly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
