#!/usr/bin/env python3
"""
SteerBot RL Training - Our Implementation
Trains PPO in live ROS2 loop with richer observations than baseline.
Observation: [error, error_velocity, arm_j1..j6, prev_action] = 9 dims
Action: wheel angle delta (degrees)
"""
import sys
import time
import numpy as np
import gymnasium as gym
import rclpy
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback

MODEL_PATH = "/home/students_steeringwheel/logs/our_ppo_model/final_model.zip"
ARM_JOINTS = ["joint1","joint2","joint3","joint4","joint5","joint6"]

class WheelHoldOursEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.max_action_deg = 5.0
        self.episode_length = 400
        self.step_count = 0
        self.wheel_now = None
        self.target_angle = None
        self.prev_error = 0.0
        self.prev_action = 0.0
        self.last_wheel_time = 0.0
        self.arm_pos = np.zeros(6)

        # 9-dim obs: error, error_vel, 6 arm joints, prev_action
        self.observation_space = gym.spaces.Box(
            low=np.array([-180.0, -500.0] + [-360.0]*6 + [-1.5], dtype=np.float32),
            high=np.array([180.0, 500.0] + [360.0]*6 + [1.5], dtype=np.float32),
        )
        self.action_space = gym.spaces.Box(
            low=np.array([-self.max_action_deg], dtype=np.float32),
            high=np.array([self.max_action_deg], dtype=np.float32),
        )

        self.node = rclpy.create_node("steerbot_ppo_trainer")
        self.pub = self.node.create_publisher(Float32, "/ai/wheel_hold_action", 10)
        self.node.create_subscription(Float32, "/wheel/position_from_ee", self.on_wheel, 10)
        self.node.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        print("SteerBot PPO Trainer started. obs=9dim, action=±1.5deg")
        # publish zero action immediately so grab node doesnt timeout
        for _ in range(5):
            self.pub.publish(Float32(data=0.0))
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def on_wheel(self, msg):
        self.wheel_now = float(msg.data)
        self.last_wheel_time = time.time()

    def on_joints(self, msg):
        for i, name in enumerate(ARM_JOINTS):
            if name in msg.name:
                idx = msg.name.index(name)
                self.arm_pos[i] = np.degrees(msg.position[idx])

    def wait_for_wheel(self):
        while rclpy.ok() and self.wheel_now is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def get_obs(self):
        error = self.target_angle - self.wheel_now
        error_vel = error - self.prev_error
        self.prev_error = error
        return np.array(
            [error, error_vel] + list(self.arm_pos) + [self.prev_action],
            dtype=np.float32
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self.wait_for_wheel()
        self.pub.publish(Float32(data=0.0))
        self.target_angle = self.wheel_now
        self.prev_error = 0.0
        self.prev_action = 0.0
        print(f"\nNew episode — target={self.target_angle:.2f} deg")
        return self.get_obs(), {}

    def step(self, action):
        self.step_count += 1
        action_deg = float(np.clip(action[0], -self.max_action_deg, self.max_action_deg))
        msg = Float32()
        msg.data = action_deg
        self.pub.publish(msg)
        self.prev_action = action_deg

        old_time = self.last_wheel_time
        deadline = time.time() + 1.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.last_wheel_time > old_time:
                break

        obs = self.get_obs()
        error = float(obs[0])
        error_vel = float(obs[1])

        # Reward: penalize error squared + small action penalty + velocity penalty
        reward = -0.1 * error**2 - 0.01 * abs(action_deg) - 0.005 * abs(error_vel)

        # detect stuck at physical limit
        self.stuck_counter = getattr(self, "stuck_counter", 0)
        if abs(error) < 0.1 and abs(action_deg) > 1.0:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        stuck = self.stuck_counter > 30
        terminated = abs(error) > 25.0 or stuck
        if stuck:
            print(f"STUCK detected at step {self.step_count} — ending episode")
        truncated = self.step_count >= self.episode_length

        if terminated:
            self.pub.publish(Float32(data=0.0))

        if self.step_count % 20 == 0 or terminated:
            print(f"step={self.step_count:04d} target={self.target_angle:.2f} "
                  f"wheel={self.wheel_now:.2f} err={error:.3f} act={action_deg:.3f} rew={reward:.3f}")

        return obs, reward, terminated, truncated, {}

    def close(self):
        self.pub.publish(Float32(data=0.0))
        self.node.destroy_node()

def main():
    import os
    os.makedirs("/home/students_steeringwheel/logs/our_ppo_model", exist_ok=True)
    rclpy.init()
    env = Monitor(WheelHoldOursEnv())
    checkpoint_cb = CheckpointCallback(
        save_freq=5000,
        save_path="/home/students_steeringwheel/logs/our_ppo_model/",
        name_prefix="steerbot_ppo",
    )
    model = PPO(
        "MlpPolicy", env, verbose=1,
        learning_rate=3e-4,
        n_steps=512,
        batch_size=64,
        gamma=0.99,
        tensorboard_log="/home/students_steeringwheel/logs/our_ppo_tensorboard/",
    )
    print("Starting SteerBot PPO training — 300k timesteps")
    try:
        model.learn(total_timesteps=50_000, callback=checkpoint_cb, reset_num_timesteps=True)
    except KeyboardInterrupt:
        print("Training interrupted — saving model...")
    finally:
        model.save(MODEL_PATH)
        print(f"Model saved to {MODEL_PATH}")
        env.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
