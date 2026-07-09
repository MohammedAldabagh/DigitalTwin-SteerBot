import sys
sys.path.insert(0, "/home/students_steeringwheel/isaac-lab/source/isaaclab")
sys.path.insert(0, "/home/students_steeringwheel/isaac-lab/source/isaaclab_assets")
sys.path.insert(0, "/home/students_steeringwheel/isaac-lab/source/isaaclab_tasks")
sys.path.insert(0, "/home/students_steeringwheel/isaac-lab/source/isaaclab_rl")
sys.path.insert(0, "/home/students_steeringwheel")

from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import os
import torch
from datetime import datetime

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
    RslRlVecEnvWrapper,
)
from rsl_rl.runners import OnPolicyRunner

from wheel_hold_task.wheel_hold_env_cfg import WheelHoldEnvCfg
from wheel_hold_task.wheel_hold_env import WheelHoldEnv

@configclass
class WheelHoldPPOCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 3000
    save_interval = 100
    experiment_name = "wheel_hold"
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_obs_normalization=True,
        critic_obs_normalization=True,
        actor_hidden_dims=[256, 128, 64],
        critic_hidden_dims=[256, 128, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )

env_cfg = WheelHoldEnvCfg()
env_cfg.scene.num_envs = 64

env = WheelHoldEnv(env_cfg)
env = RslRlVecEnvWrapper(env)

agent_cfg = WheelHoldPPOCfg()
agent_cfg.device = "cuda:0"

log_dir = os.path.join("logs", "wheel_hold", datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
os.makedirs(log_dir, exist_ok=True)

runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

env.close()
simulation_app.close()
