#!/usr/bin/env python3
import sys, time
import numpy as np
import torch
import torch.nn as nn
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

CHECKPOINT = "/home/students_steeringwheel/logs/wheel_hold_pid/2026-07-02_12-27-24/model_1499.pt"
ARM_JOINTS = ["joint1","joint2","joint3","joint4","joint5","joint6"]
TARGET_ANGLE_DEG = 93.0

KP_RANGE = (0.1, 2.0)
KI_RANGE = (0.001, 0.05)
KD_RANGE = (0.01, 0.5)
MAX_CORRECTION_DEG = 5.0
INTEGRAL_CLAMP = 50.0

def load_actor(checkpoint_path, device):
    ckpt = torch.load(checkpoint_path, map_location=device)
    actor = nn.Sequential(
        nn.Linear(15, 256), nn.ELU(),
        nn.Linear(256, 128), nn.ELU(),
        nn.Linear(128, 64), nn.ELU(),
        nn.Linear(64, 3)
    )
    actor_state = {k.replace('actor.', ''): v for k, v in ckpt['model_state_dict'].items() if k.startswith('actor.') and not k.startswith('actor_obs_normalizer')}
    actor.load_state_dict(actor_state)
    actor.eval()
    obs_mean = ckpt['model_state_dict']['actor_obs_normalizer._mean'].to(device)
    obs_std = ckpt['model_state_dict']['actor_obs_normalizer._std'].to(device)
    return actor.to(device), obs_mean, obs_std

def decode_gains(raw):
    a = np.clip(raw, -1.0, 1.0)
    a01 = (a + 1.0) * 0.5
    kp = KP_RANGE[0] + a01[0] * (KP_RANGE[1] - KP_RANGE[0])
    ki = KI_RANGE[0] + a01[1] * (KI_RANGE[1] - KI_RANGE[0])
    kd = KD_RANGE[0] + a01[2] * (KD_RANGE[1] - KD_RANGE[0])
    return kp, ki, kd

class PPOWheelPIDAgent(Node):
    def __init__(self):
        super().__init__("ppo_wheel_pid_agent")
        self.target_angle = TARGET_ANGLE_DEG
        self.wheel_now = TARGET_ANGLE_DEG
        self.wheel_vel = 0.0
        self.prev_wheel = None
        self.prev_time = None
        self.arm_pos = np.zeros(6)
        self.arm_vel = np.zeros(6)
        self.prev_arm_pos = None
        self.integral = 0.0
        self.prev_error_deg = 0.0
        self.max_abs_error = 0.0
        self.sum_abs_error = 0.0
        self.sample_count = 0
        self.test_start_time = time.time()
        self.test_duration = 60.0
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.actor, self.obs_mean, self.obs_std = load_actor(CHECKPOINT, self.device)
        self.get_logger().info(f"PID-gain policy loaded. Target={TARGET_ANGLE_DEG} deg.")
        self.create_subscription(Float32, "/wheel/position_from_ee", self.on_wheel, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.pub = self.create_publisher(Float32, "/ai/wheel_hold_action", 10)
        self.create_timer(0.05, self.loop)

    def on_wheel(self, msg):
        now = time.time()
        w = float(msg.data)
        if self.prev_wheel is not None:
            self.wheel_vel = (w - self.prev_wheel) / max(now - self.prev_time, 0.001)
        self.prev_wheel, self.prev_time, self.wheel_now = w, now, w
        self.get_logger().info(f"Wheel update: {w:.3f} deg", throttle_duration_sec=2.0)

    def on_joints(self, msg):
        for i, name in enumerate(ARM_JOINTS):
            if name in msg.name:
                idx = msg.name.index(name)
                p = np.degrees(msg.position[idx])
                if self.prev_arm_pos is not None:
                    self.arm_vel[i] = (p - self.prev_arm_pos[i]) * 0.1
                self.arm_pos[i] = p
        self.prev_arm_pos = self.arm_pos.copy()

    def loop(self):
        err_deg = self.target_angle - self.wheel_now
        while err_deg > 180: err_deg -= 360
        while err_deg < -180: err_deg += 360
        elapsed_now = time.time() - self.test_start_time
        if elapsed_now >= 5.0:
            self.max_abs_error = max(self.max_abs_error, abs(err_deg))
            self.sum_abs_error += abs(err_deg)
            self.sample_count += 1

        obs = np.clip(np.concatenate([
            np.radians(self.arm_pos),
            self.arm_vel,
            [np.radians(err_deg)],
            [np.radians(self.wheel_vel)],
            [self.integral / INTEGRAL_CLAMP]
        ]).astype(np.float32), -10, 10)
        obs_t = torch.tensor(obs, device=self.device).unsqueeze(0)
        obs_t = (obs_t - self.obs_mean) / self.obs_std
        with torch.no_grad():
            raw = self.actor(obs_t)[0].cpu().numpy()
        kp, ki, kd = decode_gains(raw)

        if abs(err_deg) > 15.0:
            self.integral = 0.0
        else:
            self.integral += err_deg * 0.05
            self.integral = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, self.integral))

        deriv_deg = (err_deg - self.prev_error_deg) / 0.05
        correction_deg = kp * err_deg + ki * self.integral + kd * deriv_deg
        correction_deg = max(-MAX_CORRECTION_DEG, min(MAX_CORRECTION_DEG, correction_deg))
        self.prev_error_deg = err_deg

        out = Float32()
        out.data = float(correction_deg)
        self.pub.publish(out)

        self.get_logger().info(
            f"target={self.target_angle:.2f} wheel={self.wheel_now:.2f} err={err_deg:.2f} "
            f"kp={kp:.3f} ki={ki:.4f} kd={kd:.3f} act={correction_deg:.3f}",
            throttle_duration_sec=1.0
        )

        if elapsed_now >= self.test_duration:
            mean_err = self.sum_abs_error / max(1, self.sample_count)
            self.get_logger().info("========== PID-GAIN HOLD TEST RESULT ==========")
            self.get_logger().info(f"Duration       : {self.test_duration:.1f} s")
            self.get_logger().info(f"Mean abs error : {mean_err:.3f} deg")
            self.get_logger().info(f"Max abs error  : {self.max_abs_error:.3f} deg")
            self.get_logger().info(f"Samples        : {self.sample_count}")
            self.get_logger().info("================================================")
            out.data = 0.0
            self.pub.publish(out)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PPOWheelPIDAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
