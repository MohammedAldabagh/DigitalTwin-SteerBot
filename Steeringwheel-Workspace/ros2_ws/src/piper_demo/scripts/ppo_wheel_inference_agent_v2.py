#!/usr/bin/env python3
import sys, time
import numpy as np
import torch
import torch.nn as nn
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

CHECKPOINT = "/home/students_steeringwheel/logs/wheel_hold_v2/2026-07-01_10-42-44/model_900.pt"
ARM_JOINTS = ["joint1","joint2","joint3","joint4","joint5","joint6"]
ACTION_SCALE_DEG = 1.5
TARGET_ANGLE_DEG = 90.0

def load_actor(checkpoint_path, device):
    ckpt = torch.load(checkpoint_path, map_location=device)
    actor = nn.Sequential(
        nn.Linear(15, 256), nn.ELU(),
        nn.Linear(256, 128), nn.ELU(),
        nn.Linear(128, 64), nn.ELU(),
        nn.Linear(64, 1)
    )
    actor_state = {k.replace('actor.', ''): v for k, v in ckpt['model_state_dict'].items() if k.startswith('actor.')}
    actor.load_state_dict(actor_state)
    actor.eval()
    return actor.to(device)

class PPOWheelInferenceAgentV2(Node):
    def __init__(self):
        super().__init__("ppo_wheel_inference_agent_v2")
        self.target_angle = TARGET_ANGLE_DEG
        self.wheel_now = TARGET_ANGLE_DEG
        self.wheel_vel = 0.0
        self.prev_wheel = None
        self.prev_time = None
        self.arm_pos = np.zeros(6)
        self.arm_vel = np.zeros(6)
        self.prev_arm_pos = None
        self.prev_action = 0.0
        self.max_abs_error = 0.0
        self.sum_abs_error = 0.0
        self.sample_count = 0
        self.test_start_time = time.time()
        self.test_duration = 60.0
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.actor = load_actor(CHECKPOINT, self.device)
        self.get_logger().info(f"Policy loaded. Target={TARGET_ANGLE_DEG} deg. Publishing immediately.")
        self.create_subscription(Float32, "/wheel/position", self.on_wheel, 10)
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
        err = self.target_angle - self.wheel_now
        while err > 180: err -= 360
        while err < -180: err += 360
        self.max_abs_error = max(self.max_abs_error, abs(err))
        self.sum_abs_error += abs(err)
        self.sample_count += 1
        obs = np.clip(np.concatenate([
            np.radians(self.arm_pos),
            self.arm_vel,
            [np.radians(err)],
            [np.radians(self.wheel_vel)],
            [self.prev_action]
        ]).astype(np.float32), -10, 10)
        with torch.no_grad():
            act = -float(self.actor(torch.tensor(obs, device=self.device).unsqueeze(0))[0,0].cpu()) * ACTION_SCALE_DEG
        out = Float32()
        out.data = act
        self.pub.publish(out)
        self.prev_action = act
        self.get_logger().info(
            f"target={self.target_angle:.2f} wheel={self.wheel_now:.2f} err={err:.2f} act={act:.3f}",
            throttle_duration_sec=1.0
        )
        if (time.time()-self.test_start_time) >= self.test_duration:
            mean_err = self.sum_abs_error / max(1, self.sample_count)
            self.get_logger().info(f"DONE mean={mean_err:.3f} max={self.max_abs_error:.3f}")
            out.data = 0.0
            self.pub.publish(out)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PPOWheelInferenceAgentV2()
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
