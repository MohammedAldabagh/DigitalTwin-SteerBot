from __future__ import annotations
import math
import isaaclab.sim as sim_utils
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
WHEEL_HOLD_USD_PATH = (
    "/home/students_steeringwheel/Steeringwheel-Workspace/isaac/scenes/"
    "g29_rotate_right_tilted27degrees.usd"
)
GRASP_JOINT_POS_DEG = {
    "joint1": 0.000754,
    "joint2": 152.283209,
    "joint3": -149.552996,
    "joint4": -0.003891,
    "joint5": 29.335842,
    "joint6": 90.002700,
}
@configclass
class WheelHoldEnvCfg(DirectRLEnvCfg):
    episode_length_s = 10.0
    decimation = 2
    action_space = 3
    observation_space = 15
    state_space = 0
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120,
        render_interval=decimation,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=256, env_spacing=2.5, replicate_physics=True, clone_in_fabric=True
    )
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/G29_root",
        spawn=sim_utils.UsdFileCfg(
            usd_path=WHEEL_HOLD_USD_PATH,
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "joint1": math.radians(GRASP_JOINT_POS_DEG["joint1"]),
                "joint2": math.radians(GRASP_JOINT_POS_DEG["joint2"]),
                "joint3": math.radians(GRASP_JOINT_POS_DEG["joint3"]),
                "joint4": math.radians(GRASP_JOINT_POS_DEG["joint4"]),
                "joint5": math.radians(GRASP_JOINT_POS_DEG["joint5"]),
                "joint6": math.radians(GRASP_JOINT_POS_DEG["joint6"]),
                "joint7": 0.0,
                "joint8": 0.0,
            },
            pos=(0.0, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0),
        ),
         actuators={
            "piper_arm": ImplicitActuatorCfg(
                joint_names_expr=["joint[1-6]"],
                effort_limit_sim=60.0,
                stiffness=400.0,
                damping=200.0,
            ),
            "piper_gripper": ImplicitActuatorCfg(
                joint_names_expr=["joint[7-8]"],
                effort_limit_sim=20.0,
                stiffness=2e3,
                damping=1e2,
            ),
        },
    )
    action_scale = 2.0
    joint_vel_scale = 0.1
    initial_wheel_offset_range_deg = (-10.0, 10.0)
    target_angle_range_deg = (1.0, 5.0)
    max_wheel_error_deg = 720.0
    reward_error_norm_deg = 7.0
    rew_scale_error = -3.0
    rew_scale_action_penalty = -0.01
    rew_scale_action_rate = -0.3 
    rew_scale_alive = 0.02
    rew_scale_joint_pose = 0.0
    rew_scale_terminated = -1.0
    grip_static_friction = 0.7
    grip_dynamic_friction = 1.2
    grip_restitution = 0.0
    grasp_settle_steps = 5
    grasp_timeout_steps = 300
    grasp_contact_force_threshold = 0.5
