#!/usr/bin/env python3

import time
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from stable_baselines3 import PPO


MODEL_PATH = "/tmp/wheel_hold_policy_big_action.zip"


class PPOWheelInferenceAgent(Node):
    def __init__(self):
        super().__init__("ppo_wheel_inference_agent")

        self.max_action_deg = 5.0
        self.deadband_deg = 0.03

        self.target_angle = None
        self.wheel_now = None
        self.prev_error = 0.0
        self.prev_action = 0.0

        self.test_duration = 60.0
        self.test_start_time = None
        self.max_abs_error = 0.0
        self.sum_abs_error = 0.0
        self.sample_count = 0

        self.model = PPO.load(MODEL_PATH)

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

        self.get_logger().info(f"Loaded PPO model: {MODEL_PATH}")
        self.get_logger().info("Input:  /wheel/position")
        self.get_logger().info("Output: /ai/wheel_hold_action")
        self.get_logger().info(f"Test duration: {self.test_duration:.1f} s")

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def publish_action(self, action_deg):
        out = Float32()
        out.data = float(action_deg)
        self.pub.publish(out)

    def print_test_result_and_stop(self, elapsed):
        mean_abs_error = self.sum_abs_error / max(1, self.sample_count)

        self.get_logger().info("========== HOLD TEST RESULT ==========")
        self.get_logger().info(f"Duration       : {elapsed:.1f} s")
        self.get_logger().info(f"Target angle   : {self.target_angle:.3f} deg")
        self.get_logger().info(f"Max abs error  : {self.max_abs_error:.3f} deg")
        self.get_logger().info(f"Mean abs error : {mean_abs_error:.3f} deg")
        self.get_logger().info(f"Samples        : {self.sample_count}")
        self.get_logger().info("======================================")

        self.publish_action(0.0)
        rclpy.shutdown()

    def on_wheel_position(self, msg):
        self.wheel_now = float(msg.data)

        if self.target_angle is None:
            self.target_angle = self.wheel_now
            self.prev_error = 0.0
            self.prev_action = 0.0
            self.test_start_time = time.time()

            self.get_logger().info(
                f"Captured inference target angle: {self.target_angle:.3f} deg"
            )
            return

        error = self.target_angle - self.wheel_now
        error_velocity = error - self.prev_error

        abs_error = abs(error)
        self.max_abs_error = max(self.max_abs_error, abs_error)
        self.sum_abs_error += abs_error
        self.sample_count += 1

        obs = np.array(
            [error, error_velocity, self.prev_action],
            dtype=np.float32,
        )

        if abs(error) < self.deadband_deg:
            action_deg = 0.0
        else:
            action, _ = self.model.predict(obs, deterministic=True)
            action_deg = float(action[0])

        action_deg = self.clamp(
            action_deg,
            -self.max_action_deg,
            self.max_action_deg,
        )

        self.publish_action(action_deg)

        self.prev_error = error
        self.prev_action = action_deg

        elapsed = time.time() - self.test_start_time

        self.get_logger().info(
            f"time={elapsed:.1f}s "
            f"target={self.target_angle:.3f} "
            f"wheel={self.wheel_now:.3f} "
            f"error={error:.3f} "
            f"action={action_deg:.3f}"
        )

        if elapsed >= self.test_duration:
            self.print_test_result_and_stop(elapsed)


def main(args=None):
    rclpy.init(args=args)
    node = PPOWheelInferenceAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_action(0.0)
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()