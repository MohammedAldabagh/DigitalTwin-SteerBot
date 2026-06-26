#!/usr/bin/env python3

import time
import numpy as np
import gymnasium as gym

import rclpy
from std_msgs.msg import Float32

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback


MODEL_PATH = "/tmp/wheel_hold_policy_big_action.zip"


class WheelHoldIsaacRosEnv(gym.Env):
    def __init__(self):
        super().__init__()

        self.max_action_deg = 8.0
        self.episode_length = 300
        self.step_count = 0

        self.wheel_now = None
        self.target_angle = None

        self.prev_error = 0.0
        self.prev_action = 0.0
        self.last_msg_time = 0.0

        self.action_space = gym.spaces.Box(
            low=np.array([-self.max_action_deg], dtype=np.float32),
            high=np.array([self.max_action_deg], dtype=np.float32),
            dtype=np.float32,
        )

        self.observation_space = gym.spaces.Box(
            low=np.array([-180.0, -500.0, -8.0], dtype=np.float32),
            high=np.array([180.0, 500.0, 8.0], dtype=np.float32),
            dtype=np.float32,
        )

        self.node = rclpy.create_node("ppo_wheel_training_env_big_action")

        self.pub = self.node.create_publisher(
            Float32,
            "/ai/wheel_hold_action",
            10,
        )

        self.sub = self.node.create_subscription(
            Float32,
            "/wheel/position",
            self.on_wheel_position,
            10,
        )

        print("WheelHoldIsaacRosEnv BIG ACTION started")
        print("Subscribing to /wheel/position")
        print("Publishing to /ai/wheel_hold_action")
        print(f"Action range: ±{self.max_action_deg} deg")

    def on_wheel_position(self, msg):
        self.wheel_now = float(msg.data)
        self.last_msg_time = time.time()

    def wait_for_wheel(self):
        while rclpy.ok() and self.wheel_now is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def publish_action(self, action_deg):
        msg = Float32()
        msg.data = float(action_deg)
        self.pub.publish(msg)

    def get_obs(self):
        error = self.target_angle - self.wheel_now
        error_velocity = error - self.prev_error

        obs = np.array(
            [error, error_velocity, self.prev_action],
            dtype=np.float32,
        )

        self.prev_error = error
        return obs

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.step_count = 0
        self.wait_for_wheel()

        self.publish_action(0.0)

        # Same logic as first training:
        # target is whatever angle the wheel has at episode start.
        self.target_angle = self.wheel_now

        self.prev_error = 0.0
        self.prev_action = 0.0

        obs = self.get_obs()

        info = {
            "target_angle": self.target_angle,
            "wheel_now": self.wheel_now,
            "error": self.target_angle - self.wheel_now,
        }

        print("")
        print("======================================")
        print(f"New episode target angle: {self.target_angle:.3f} deg")
        print("Goal: hold wheel at current angle")
        print("BIG ACTION training mode")
        print("======================================")
        print("")

        return obs, info

    def step(self, action):
        self.step_count += 1

        action_deg = float(action[0])
        action_deg = float(
            np.clip(action_deg, -self.max_action_deg, self.max_action_deg)
        )

        self.publish_action(action_deg)
        self.prev_action = action_deg

        old_msg_time = self.last_msg_time
        timeout_time = time.time() + 1.0

        while rclpy.ok() and time.time() < timeout_time:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.last_msg_time > old_msg_time:
                break

        obs = self.get_obs()

        error = float(obs[0])
        error_velocity = float(obs[1])

        reward = -0.1 * (error ** 2) - 0.02 * abs(action_deg)

        terminated = abs(error) > 30.0
        truncated = self.step_count >= self.episode_length

        if terminated:
            self.publish_action(0.0)

        info = {
            "wheel_now": self.wheel_now,
            "target_angle": self.target_angle,
            "error": error,
            "error_velocity": error_velocity,
            "action": action_deg,
            "reward": reward,
            "step": self.step_count,
        }

        if self.step_count % 10 == 0 or terminated:
            print(
                f"step={self.step_count:04d} "
                f"target={self.target_angle:.3f} "
                f"wheel={self.wheel_now:.3f} "
                f"error={error:.3f} "
                f"action={action_deg:.3f} "
                f"reward={reward:.3f}"
            )

        return obs, reward, terminated, truncated, info

    def close(self):
        self.publish_action(0.0)
        self.node.destroy_node()


def main():
    rclpy.init()

    env = WheelHoldIsaacRosEnv()
    env = Monitor(env)

    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path="./ppo_wheel_models_big_action",
        name_prefix="wheel_hold_big_action_model",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )

    print("")
    print("======================================")
    print("Starting NEW PPO training from scratch")
    print("Same logic as first successful training")
    print("Big-action policy: ±8 deg")
    print(f"Final model will save to: {MODEL_PATH}")
    print("======================================")
    print("")

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=256,
        batch_size=64,
        gamma=0.99,
        tensorboard_log="./ppo_wheel_tensorboard_big_action",
    )

    try:
        model.learn(
            total_timesteps=300_000,
            callback=checkpoint_callback,
            reset_num_timesteps=True,
        )

    except KeyboardInterrupt:
        print("")
        print("Training interrupted by user.")
        print("Saving current big-action model before exit...")

    finally:
        model.save(MODEL_PATH)
        print(f"Saved big-action model to: {MODEL_PATH}")

        env.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()