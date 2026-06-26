#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class AIWheelHoldAgent(Node):
    def __init__(self):
        super().__init__("ai_wheel_hold_agent")

        self.target_angle = None
        self.kp = 1.0
        self.max_action_deg = 4.0

        self.sub = self.create_subscription(
            Float32,
            "/wheel/position",
            self.on_wheel_position,
            10,
        )

        self.pub = self.create_publisher(
            Float32,
            "/ai/wheel_hold_action",
            10,
        )

        self.get_logger().info("AI wheel hold agent started")
        self.get_logger().info("Input:  /wheel/position")
        self.get_logger().info("Output: /ai/wheel_hold_action")

    def on_wheel_position(self, msg):
        wheel_now = float(msg.data)

        if self.target_angle is None:
            self.target_angle = wheel_now
            self.get_logger().info(f"Captured target angle: {self.target_angle:.3f} deg")

        error = self.target_angle - wheel_now

        # Temporary policy.
        # Later we replace this with a trained neural network.
        action = self.kp * error
        action = max(-self.max_action_deg, min(self.max_action_deg, action))

        out = Float32()
        out.data = float(action)
        self.pub.publish(out)

        line = (
            f"wheel={wheel_now:.3f},"
            f"target={self.target_angle:.3f},"
            f"error={error:.3f},"
            f"action={action:.3f}"
        )

        self.get_logger().info(line)

        with open("/tmp/ai_hold_log.csv", "a") as f:
            f.write(line + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = AIWheelHoldAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
