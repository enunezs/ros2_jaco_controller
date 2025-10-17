#!/usr/bin/env python3
# mode_manager.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ModeManager(Node):
    """
    Centralized mode manager.
    Subscribes to /teleop/mode_command (String) and publishes /teleop/current_mode (String).
    Also accepts /teleop/mode_command messages that may be sent by CommandMapper.
    """

    VALID_MODES = ["translation", "rotation", "discrete", "system"]

    def __init__(self):
        super().__init__('mode_manager')
        self.get_logger().info("ModeManager starting...")
        self.current_mode = "translation"

        # Publishers & subscribers
        self.mode_pub = self.create_publisher(String, '/teleop/current_mode', 10)
        self.mode_cmd_sub = self.create_subscription(String, '/teleop/mode_command', self.mode_command_cb, 10)

        # Publish initial mode
        self.publish_mode()

    def publish_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"Published current_mode: {self.current_mode}")

    def mode_command_cb(self, msg: String):
        requested = msg.data
        self.get_logger().info(f"Mode command received: {requested}")
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
