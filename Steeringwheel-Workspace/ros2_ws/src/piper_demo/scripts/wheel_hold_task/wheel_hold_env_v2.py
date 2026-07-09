from __future__ import annotations

import math
import torch
from collections.abc import Sequence

from pxr import UsdGeom, UsdPhysics, PhysxSchema

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.envs import DirectRLEnv
from isaacsim.core.utils.stage import get_current_stage
from isaaclab.utils.math import sample_uniform

from .wheel_hold_env_cfg import WheelHoldEnvCfg, GRASP_JOINT_POS_DEG

WHEEL_BODY_PRIM_PATTERN = "/World/envs/*/G29_root/Steerbot_G29_steerwheel_position_27degrees"

WHEEL_AXIS_WORLD = torch.tensor([0.0, -math.sin(math.radians(27.0)), math.cos(math.radians(27.0))])


def _spawn_simple_ground_plane(prim_path: str, size: float = 50.0) -> None:
    stage = get_current_stage()
    plane_geom = UsdGeom.Plane.Define(stage, prim_path)
    plane_geom.CreateAxisAttr("Z")
    plane_geom.CreateWidthAttr(size)
    plane_geom.CreateLengthAttr(size)
    UsdPhysics.CollisionAPI.Apply(plane_geom.GetPrim())


class WheelHoldEnvV2(DirectRLEnv):
    cfg: WheelHoldEnvCfg

    def __init__(self, cfg: WheelHoldEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self._arm_dof_idx, _ = self.robot.find_joints(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self._gripper_dof_idx, _ = self.robot.find_joints(["joint7", "joint8"])
        self._link6_idx = self.robot.find_bodies(["link6"])[0][0]

        self.robot_dof_lower_limits = self.robot.data.soft_joint_pos_limits[0, :, 0].to(device=self.device)
        self.robot_dof_upper_limits = self.robot.data.soft_joint_pos_limits[0, :, 1].to(device=self.device)

        self.wheel_target_rad = torch.zeros(self.num_envs, device=self.device)
        self.wheel_angle_offset_rad = torch.zeros(self.num_envs, device=self.device)
        self.prev_action = torch.zeros((self.num_envs, 1), device=self.device)
        self.last_action = torch.zeros((self.num_envs, 1), device=self.device)
        self.a_cmd = torch.zeros(self.num_envs, device=self.device)

        self.grasp_confirmed = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.grasp_settle_counter = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.steps_since_reset = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.wheel_drive_released = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        self.wheel_center_w = torch.zeros((self.num_envs, 3), device=self.device)
        self.rim_radius = torch.ones(self.num_envs, device=self.device) * 2.9
        self.wheel_axis_w = WHEEL_AXIS_WORLD.to(device=self.device).unsqueeze(0).expand(self.num_envs, -1)

        self.action_scale_rad = torch.deg2rad(torch.tensor(self.cfg.action_scale, device=self.device))

        self._prev_wheel_angle_rad = torch.zeros(self.num_envs, device=self.device)
        self._prev_arm_pos = torch.zeros((self.num_envs, len(self._arm_dof_idx)), device=self.device)

        self._cached_wheel_angle = None
        self._cached_wheel_vel = None
        self._cached_arm_vel = None
        self._cache_step_id = -1

        ik_cfg = DifferentialIKControllerCfg(
            command_type="position",
            use_relative_mode=True,
            ik_method="dls",
        )
        self._ik_controller = DifferentialIKController(ik_cfg, self.num_envs, self.device)

        self._jacobi_body_idx = self._link6_idx
        all_joint_names = [j for j in self.robot.joint_names]
        arm_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self._jacobi_joint_ids = [all_joint_names.index(j) for j in arm_joint_names if j in all_joint_names]

        self.grasp_joint_targets = torch.deg2rad(
            torch.tensor(
                [GRASP_JOINT_POS_DEG[j] for j in ["joint1","joint2","joint3","joint4","joint5","joint6"]],
                device=self.device,
            )
        )

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)

        stage = get_current_stage()
        robot_prim_path_0 = self.cfg.robot.prim_path.replace(".*", "0")

        g29_root_prim = stage.GetPrimAtPath(robot_prim_path_0)
        if g29_root_prim.IsValid() and g29_root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            g29_root_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

        piper_desc_prim = stage.GetPrimAtPath(f"{robot_prim_path_0}/piper_description_01")
        if piper_desc_prim.IsValid() and piper_desc_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            piper_desc_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

        link_names = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "link7", "link8"]
        for link_name in link_names:
            link_prim = stage.GetPrimAtPath(f"{robot_prim_path_0}/piper_description_01/{link_name}")
            if link_prim.IsValid() and not link_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI.Apply(link_prim)

        env_root_path = robot_prim_path_0.split("/G29_root")[0]
        wheel_body_prim = stage.GetPrimAtPath(f"{env_root_path}/G29_root/Steerbot_G29_steerwheel_position_27degrees")
        if wheel_body_prim.IsValid() and not wheel_body_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI.Apply(wheel_body_prim)

        wheel_mesh_prim = stage.GetPrimAtPath(
            f"{env_root_path}/G29_root/Steerbot_G29_steerwheel_position_27degrees/"
            "Steerbot_G29_steerwheel_position_27degrees/mesh"
        )
        if wheel_mesh_prim.IsValid():
            mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(wheel_mesh_prim)
            mesh_collision_api.GetApproximationAttr().Set("convexHull")

        _spawn_simple_ground_plane(prim_path="/World/ground")

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])

        self._author_grip_friction_material()
        self.scene.articulations["robot"] = self.robot

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _author_grip_friction_material(self):
        stage = get_current_stage()
        robot_prim_path_0 = self.cfg.robot.prim_path.replace('.*', '0')
        material_path = "/World/PhysicsMaterials/GripMaterial"
        material_cfg = sim_utils.RigidBodyMaterialCfg(
            static_friction=self.cfg.grip_static_friction,
            dynamic_friction=self.cfg.grip_dynamic_friction,
            restitution=self.cfg.grip_restitution,
            friction_combine_mode="max",
            restitution_combine_mode="min",
        )
        material_cfg.func(material_path, material_cfg)
        for p in [
            f"{robot_prim_path_0}/piper_description_01/link7/collisions",
            f"{robot_prim_path_0}/piper_description_01/link8/collisions",
        ]:
            sim_utils.bind_physics_material(p, material_path)
        wheel_rim_path = (
            f"{robot_prim_path_0.split('/G29_root')[0]}/G29_root/"
            "Steerbot_G29_steerwheel_position_27degrees/"
            "Steerbot_G29_steerwheel_position_27degrees/mesh"
        )
        sim_utils.bind_physics_material(wheel_rim_path, material_path)

    def _get_wheel_angle_rad(self) -> torch.Tensor:
        if getattr(self, "_wheel_view", None) is None:
            self._wheel_view = self.sim.physics_sim_view.create_rigid_body_view(WHEEL_BODY_PRIM_PATTERN)
        transforms = self._wheel_view.get_transforms()
        quat = transforms[:, 3:7]
        w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
        raw_angle = 2.0 * torch.atan2(z, w)
        return raw_angle - self.wheel_angle_offset_rad

    def _refresh_step_cache(self):
        current_step_id = int(self.episode_length_buf.sum().item())
        if self._cache_step_id == current_step_id:
            return
        wheel_angle = self._get_wheel_angle_rad()
        raw_delta = wheel_angle - self._prev_wheel_angle_rad
        wrapped_delta = torch.atan2(torch.sin(raw_delta), torch.cos(raw_delta))
        wheel_vel = wrapped_delta / self.step_dt
        self._prev_wheel_angle_rad = wheel_angle.clone()
        arm_pos = self.robot.data.joint_pos[:, self._arm_dof_idx]
        arm_vel = (arm_pos - self._prev_arm_pos) / self.step_dt
        self._prev_arm_pos = arm_pos.clone()
        self._cached_wheel_angle = wheel_angle
        self._cached_wheel_vel = wheel_vel
        self._cached_arm_vel = arm_vel
        self._cache_step_id = current_step_id

    def _get_wheel_vel_rad_s(self):
        self._refresh_step_cache()
        return self._cached_wheel_vel

    def _get_arm_vel(self):
        self._refresh_step_cache()
        return self._cached_arm_vel

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.last_action = self.prev_action.clone()
        self.prev_action = actions.clamp(-1.0, 1.0)

    def _apply_action(self) -> None:
        gripper_closed = torch.tensor([0.035, -0.035], device=self.device).repeat(self.num_envs, 1)
        self.robot.set_joint_position_target(gripper_closed, joint_ids=self._gripper_dof_idx)

        confirmed = self.grasp_confirmed & self.wheel_drive_released
        if not torch.any(confirmed):
            return

        gripper_pos_w = self.robot.data.body_pos_w[:, self._link6_idx]
        radial = gripper_pos_w - self.wheel_center_w
        radial_norm = radial / (radial.norm(dim=-1, keepdim=True) + 1e-8)
        axis = self.wheel_axis_w
        tangent = torch.cross(axis, radial_norm, dim=-1)
        tangent = tangent / (tangent.norm(dim=-1, keepdim=True) + 1e-8)

        arc_delta_deg = self.prev_action[:, 0] * self.cfg.action_scale
        arc_delta_deg = torch.clamp(arc_delta_deg, -self.cfg.action_scale, self.cfg.action_scale)
        self.a_cmd += torch.deg2rad(arc_delta_deg)
        arc_delta = torch.deg2rad(arc_delta_deg) * self.rim_radius
        ee_delta = tangent * arc_delta.unsqueeze(-1)

        jacobians = self.robot.root_physx_view.get_jacobians()
        jacobian = jacobians[:, self._jacobi_body_idx, :, :]
        jacobian = jacobian[:, :, self._jacobi_joint_ids]

        ee_pos_curr = gripper_pos_w
        ee_quat_curr = self.robot.data.body_quat_w[:, self._link6_idx]

        self._ik_controller.set_command(ee_delta, ee_pos=gripper_pos_w, ee_quat=ee_quat_curr)
        joint_pos = self.robot.data.joint_pos[:, self._arm_dof_idx]
        delta_joint = self._ik_controller.compute(ee_pos_curr, ee_quat_curr, jacobian, joint_pos)

        new_targets = torch.clamp(
            joint_pos + delta_joint,
            self.robot_dof_lower_limits[self._arm_dof_idx],
            self.robot_dof_upper_limits[self._arm_dof_idx],
        )
        self.robot.set_joint_position_target(new_targets, joint_ids=self._arm_dof_idx)

    def _release_wheel_drive_if_grasped(self):
        to_release = self.grasp_confirmed & (~self.wheel_drive_released)
        if not torch.any(to_release):
            return
        env_ids = to_release.nonzero(as_tuple=True)[0]

        wheel_pos = self._wheel_view.get_transforms()[env_ids, :3]
        self.wheel_center_w[env_ids] = wheel_pos

        gripper_pos = self.robot.data.body_pos_w[env_ids, self._link6_idx]
        self.rim_radius[env_ids] = (gripper_pos - wheel_pos).norm(dim=-1)

        base_angle = self._get_wheel_angle_rad()[env_ids]
        delta_deg = sample_uniform(
            self.cfg.target_angle_range_deg[0], self.cfg.target_angle_range_deg[1], (len(env_ids),), self.device
        )
        self.wheel_target_rad[env_ids] = base_angle + torch.deg2rad(delta_deg)
        self.wheel_drive_released[to_release] = True

    def _update_grasp_confirmation(self):
        wheel_vel = self._get_wheel_vel_rad_s()
        arm_vel = self._get_arm_vel()
        arm_settled = torch.all(torch.abs(arm_vel) < 0.5, dim=-1)
        not_freefalling = torch.abs(wheel_vel) < 5.0
        settled_this_step = arm_settled & not_freefalling
        self.grasp_settle_counter = torch.where(
            settled_this_step, self.grasp_settle_counter + 1, torch.zeros_like(self.grasp_settle_counter)
        )
        self.grasp_confirmed = self.grasp_settle_counter >= self.cfg.grasp_settle_steps
        self._release_wheel_drive_if_grasped()

    def _get_observations(self) -> dict:
        wheel_pos = self._get_wheel_angle_rad()
        wheel_vel = self._get_wheel_vel_rad_s()
        wheel_err = self.wheel_target_rad - wheel_pos
        arm_pos = self.robot.data.joint_pos[:, self._arm_dof_idx]
        arm_vel = self._get_arm_vel() * self.cfg.joint_vel_scale
        obs = torch.cat((
            arm_pos,
            arm_vel,
            wheel_err.unsqueeze(-1),
            wheel_vel.unsqueeze(-1),
            self.prev_action,
        ), dim=-1)
        return {"policy": torch.clamp(obs, -10.0, 10.0)}

    def _get_rewards(self) -> torch.Tensor:
        wheel_pos = self._get_wheel_angle_rad()
        wheel_err = self.wheel_target_rad - wheel_pos
        err_norm = wheel_err / torch.deg2rad(torch.tensor(self.cfg.reward_error_norm_deg, device=self.device))
        action_penalty = torch.sum(self.prev_action**2, dim=-1)
        action_rate = torch.sum((self.prev_action - self.last_action) ** 2, dim=-1)
        arm_pos = self.robot.data.joint_pos[:, self._arm_dof_idx]
        joint_dist = torch.norm(arm_pos - self.grasp_joint_targets.unsqueeze(0), dim=-1)
        joint_pose_reward = -joint_dist
        alive_bonus = self.cfg.rew_scale_alive * torch.exp(-torch.square(err_norm))
        hold_reward = (
            self.cfg.rew_scale_error * torch.square(err_norm)
            + self.cfg.rew_scale_action_penalty * action_penalty
            + self.cfg.rew_scale_action_rate * action_rate
            + alive_bonus
        )
        hold_reward = torch.where(self.grasp_confirmed, hold_reward, torch.zeros_like(hold_reward))
        joint_pose_reward = torch.where(self.grasp_confirmed, torch.zeros_like(joint_pose_reward), joint_pose_reward)
        reward = joint_pose_reward * self.cfg.rew_scale_joint_pose + hold_reward
        terminated, _ = self._get_dones()
        reward = torch.where(terminated, reward + self.cfg.rew_scale_terminated, reward)
        self.extras["log"] = {
            "err_deg": torch.rad2deg(wheel_err).mean(),
            "action_penalty": action_penalty.mean(),
            "action_rate": action_rate.mean(),
            "grasp_confirmed_frac": self.grasp_confirmed.float().mean(),
            "joint_dist": joint_dist.mean(),
        }
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        current_step_id = int(self.episode_length_buf.sum().item())
        if getattr(self, "_dones_cache_step_id", -1) == current_step_id:
            return self._dones_cache
        self._update_grasp_confirmation()
        wheel_pos = self._get_wheel_angle_rad()
        wheel_err = torch.abs(self.wheel_target_rad - wheel_pos)
        max_err_rad = torch.deg2rad(torch.tensor(self.cfg.max_wheel_error_deg, device=self.device))
        error_blew_up = wheel_err > max_err_rad
        grasp_never_settled = (self.steps_since_reset > self.cfg.grasp_timeout_steps) & (~self.grasp_confirmed)
        terminated = error_blew_up | grasp_never_settled
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        self.steps_since_reset += 1
        self._dones_cache_step_id = current_step_id
        self._dones_cache = (terminated, time_out)
        return terminated, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        n = len(env_ids)
        joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        joint_vel = torch.zeros_like(joint_pos)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
        self.robot.set_joint_position_target(joint_pos[:, self._arm_dof_idx], joint_ids=self._arm_dof_idx, env_ids=env_ids)
        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]
        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        current_wheel_angle = self._get_wheel_angle_rad()[env_ids]
        self._prev_wheel_angle_rad[env_ids] = current_wheel_angle
        self._prev_arm_pos[env_ids] = joint_pos[:, self._arm_dof_idx]
        self.wheel_drive_released[env_ids] = False
        self.wheel_target_rad[env_ids] = current_wheel_angle
        self.grasp_confirmed[env_ids] = False
        self.grasp_settle_counter[env_ids] = 0
        self.steps_since_reset[env_ids] = 0
        self.prev_action[env_ids] = 0.0
        self.last_action[env_ids] = 0.0