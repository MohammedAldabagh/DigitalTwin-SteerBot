#!/usr/bin/env python3
"""
ncap_scenarios.py — NCAP test case runner for CARLA + G29 steering wheel.

Scenarios
---------
  CCRs       Car-to-Car Rear stationary  — ego at test speed → stopped target
  CCRm       Car-to-Car Rear moving      — ego faster than target
  CCRb       Car-to-Car Rear braking     — target decelerates at 6 m/s²
  VRU        Pedestrian crossing         — walker crosses ego path
  LaneChange Static obstacle in lane     — ego must steer around it

What it does
------------
  1. Finds the existing hero vehicle in CARLA (spawned by carla_g29_bus_drive.launch.py).
     If none found, spawns one at --spawn-index and drives it with Traffic Manager.
  2. Spawns the scenario target --initial-gap metres ahead of the ego.
  3. When ego closes to arm_distance: arms the scenario and starts 20 Hz recording.
  4. Records per tick:
       time_s, ego_speed_kmh, target_speed_kmh, rel_distance_m, ttc_s,
       steer_actual_rad/deg (from /wheel/steering_angle),
       steer_target_rad/deg (computed avoidance angle),
       lateral_offset_m, collision, verdict
  5. Publishes steer_target → /wheel/target_angle  (G29 haptic guidance)
     Also publishes /ncap/ttc, /ncap/steer_target_rad, /ncap/rel_distance_m, /ncap/verdict
  6. Writes CSV to <workspace>/isaac/streamdata/ncap_<scenario>_<timestamp>.csv

Usage (run alongside carla_g29_bus_drive.launch.py)
---------------------------------------------------
  python3 ncap_scenarios.py --scenario CCRs
  python3 ncap_scenarios.py --scenario VRU  --ego-speed 40
  python3 ncap_scenarios.py --scenario LaneChange --initial-gap 40
  python3 ncap_scenarios.py --scenario all          # runs all 5 in sequence

ROS2 topics
-----------
  Subscribed:
    /wheel/steering_angle  (std_msgs/Float32, rad)  — actual G29 input
  Published:
    /wheel/target_angle    (std_msgs/Float32, rad)  — avoidance guidance to G29
    /ncap/ttc              (std_msgs/Float32, s)
    /ncap/steer_target_rad (std_msgs/Float32, rad)
    /ncap/rel_distance_m   (std_msgs/Float32, m)
    /ncap/verdict          (std_msgs/String)
"""

import argparse
import csv
import math
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

import carla

# ── NCAP scenario defaults (Euro NCAP 2023 protocol) ─────────────────────────

SCENARIOS = ['CCRs', 'CCRm', 'CCRb', 'VRU', 'LaneChange']

PARAMS: dict[str, dict] = {
    'CCRs': dict(
        description='Car-to-Car Rear stationary: ego 50 km/h → stopped target',
        ego_speed_kmh=50.0,
        target_speed_kmh=0.0,
        initial_gap_m=40.0,
        arm_distance_m=12.0,
        lane_width_m=3.5,
        vehicle_half_w=1.1,
        wheelbase_m=3.0,
    ),
    'CCRm': dict(
        description='Car-to-Car Rear moving: ego 50 km/h → target 20 km/h',
        ego_speed_kmh=50.0,
        target_speed_kmh=20.0,
        initial_gap_m=40.0,
        arm_distance_m=12.0,
        lane_width_m=3.5,
        vehicle_half_w=1.1,
        wheelbase_m=3.0,
    ),
    'CCRb': dict(
        description='Car-to-Car Rear braking: target decelerates at 6 m/s²',
        ego_speed_kmh=50.0,
        target_speed_kmh=50.0,
        target_decel_ms2=6.0,
        initial_gap_m=40.0,
        arm_distance_m=12.0,
        lane_width_m=3.5,
        vehicle_half_w=1.1,
        wheelbase_m=3.0,
    ),
    'VRU': dict(
        description='VRU pedestrian: crosses at 5 km/h in front of ego at 40 km/h',
        ego_speed_kmh=40.0,
        ped_speed_ms=1.4,
        ped_offset_m=3.0,      # metres to right of ego path at spawn
        initial_gap_m=40.0,
        arm_distance_m=20.0,
        lane_width_m=3.5,
        vehicle_half_w=1.1,
        wheelbase_m=3.0,
    ),
    'LaneChange': dict(
        description='Lane change: static obstacle ahead, ego steers around it',
        ego_speed_kmh=50.0,
        target_speed_kmh=0.0,
        initial_gap_m=40.0,
        arm_distance_m=40.0,
        avoidance_lateral_m=3.5,
        lane_width_m=3.5,
        vehicle_half_w=1.1,
        wheelbase_m=3.0,
    ),
}

CSV_FIELDS = [
    'time_s', 'scenario',
    'ego_speed_kmh', 'target_speed_kmh',
    'rel_distance_m', 'ttc_s',
    'steer_actual_rad', 'steer_actual_deg',
    'steer_target_rad', 'steer_target_deg',
    'lateral_offset_m', 'collision', 'verdict',
]

TM_PORT = 8000
CARLA_HOST = 'localhost'
CARLA_PORT = 2000


def _workspace_root() -> Path:
    # scripts/ → package/ → src/ → ros2_ws/ → workspace/
    return Path(__file__).resolve().parent.parent.parent.parent.parent


def _speed_kmh(actor) -> float:
    v = actor.get_velocity()
    return math.hypot(v.x, v.y, v.z) * 3.6


# ── Main node ─────────────────────────────────────────────────────────────────

class NCAPRunner(Node):

    def __init__(self, scenario: str, ego_speed_override: float,
                 spawn_index: int, initial_gap_override: float):
        super().__init__('ncap_runner')

        if scenario not in PARAMS:
            raise ValueError(f'Unknown scenario: {scenario}')

        self._scenario = scenario
        self._p: dict = PARAMS[scenario].copy()
        if ego_speed_override > 0.0:
            self._p['ego_speed_kmh'] = ego_speed_override
        if initial_gap_override > 0.0:
            self._p['initial_gap_m'] = initial_gap_override

        self._spawn_index = spawn_index
        self._steer_actual = 0.0
        self._lock = threading.Lock()
        self._rows: list[dict] = []
        self._t0: float | None = None
        self._armed = False
        self._collision = False
        self._verdict = 'PENDING'
        self._finished = False
        self._ego_owned = False   # True if we spawned the ego ourselves

        # ROS2
        self.create_subscription(Float32, '/wheel/steering_angle', self._cb_steer, 10)
        self._pub_target = self.create_publisher(Float32, '/wheel/target_angle', 10)
        self._pub_ttc = self.create_publisher(Float32, '/ncap/ttc', 10)
        self._pub_steer_tgt = self.create_publisher(Float32, '/ncap/steer_target_rad', 10)
        self._pub_rel_dist = self.create_publisher(Float32, '/ncap/rel_distance_m', 10)
        self._pub_verdict = self.create_publisher(String, '/ncap/verdict', 10)

        # CARLA
        self._client = carla.Client(CARLA_HOST, CARLA_PORT)
        self._client.set_timeout(10.0)
        self._world = self._client.get_world()
        self._ego = None
        self._target = None
        self._col_sensor = None

        self._find_or_spawn_ego()
        self._spawn_target()
        self._attach_collision_sensor()

        self.create_timer(0.05, self._tick)  # 20 Hz

        self.get_logger().info(
            f'\n{"="*60}\n'
            f'  NCAP scenario : {scenario}\n'
            f'  {self._p["description"]}\n'
            f'  ego speed     : {self._p["ego_speed_kmh"]} km/h\n'
            f'  initial gap   : {self._p["initial_gap_m"]} m\n'
            f'  arm distance  : {self._p["arm_distance_m"]} m\n'
            f'{"="*60}'
        )

    # ── CARLA: ego ────────────────────────────────────────────────────────────

    def _find_or_spawn_ego(self):
        for actor in self._world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == 'hero':
                self._ego = actor
                self.get_logger().info(
                    f'Attached to existing hero: {actor.type_id} (id={actor.id})'
                )
                return

        self.get_logger().info('No hero vehicle found — spawning one with autopilot')
        sps = self._world.get_map().get_spawn_points()
        if not sps:
            raise RuntimeError('No spawn points in CARLA map')
        sp = sps[self._spawn_index % len(sps)]

        bp_lib = self._world.get_blueprint_library()
        try:
            bp = bp_lib.find('vehicle.volkswagen.t2_2021')
        except Exception:
            bp = bp_lib.find('vehicle.volkswagen.t2')
        bp.set_attribute('role_name', 'hero')

        self._ego = self._world.try_spawn_actor(bp, sp)
        if self._ego is None:
            raise RuntimeError('Failed to spawn ego vehicle')

        tm = self._client.get_trafficmanager(TM_PORT)
        tm.set_synchronous_mode(False)
        tm.ignore_lights_percentage(self._ego, 100)
        tm.ignore_signs_percentage(self._ego, 100)
        tm.auto_lane_change(self._ego, False)
        self._ego.set_autopilot(True, TM_PORT)
        tm.set_desired_speed(self._ego, self._p['ego_speed_kmh'])
        self._ego_owned = True
        self.get_logger().info(
            f'Ego spawned at ({sp.location.x:.1f}, {sp.location.y:.1f}) '
            f'@ {self._p["ego_speed_kmh"]} km/h autopilot'
        )

    # ── CARLA: target ─────────────────────────────────────────────────────────

    def _spawn_target(self):
        tf = self._ego.get_transform()
        yaw_rad = math.radians(tf.rotation.yaw)
        fwd = (math.cos(yaw_rad), math.sin(yaw_rad))
        gap = self._p['initial_gap_m']

        target_loc = carla.Location(
            x=tf.location.x + fwd[0] * gap,
            y=tf.location.y + fwd[1] * gap,
            z=tf.location.z,
        )

        if self._scenario == 'VRU':
            self._spawn_pedestrian(target_loc, tf.rotation, fwd)
        else:
            self._spawn_car(target_loc, tf.rotation)

    def _spawn_car(self, loc: carla.Location, rotation: carla.Rotation):
        bp_lib = self._world.get_blueprint_library()
        bps = bp_lib.filter('vehicle.tesla.model3')
        bp = bps[0] if bps else bp_lib.filter('vehicle.*')[0]
        bp.set_attribute('role_name', 'ncap_target')

        for z_offset in (0.0, 0.5, 1.0):
            spawn_loc = carla.Location(x=loc.x, y=loc.y, z=loc.z + z_offset)
            self._target = self._world.try_spawn_actor(
                bp, carla.Transform(spawn_loc, rotation)
            )
            if self._target is not None:
                break

        if self._target is None:
            raise RuntimeError('Failed to spawn target car')

        tgt_speed = self._p.get('target_speed_kmh', 0.0)
        if self._scenario == 'CCRm' and tgt_speed > 0.0:
            tm = self._client.get_trafficmanager(TM_PORT)
            tm.ignore_lights_percentage(self._target, 100)
            tm.ignore_signs_percentage(self._target, 100)
            tm.auto_lane_change(self._target, False)
            self._target.set_autopilot(True, TM_PORT)
            tm.set_desired_speed(self._target, tgt_speed)
        elif self._scenario == 'CCRb':
            # Drive at ego speed until armed, then brake
            tm = self._client.get_trafficmanager(TM_PORT)
            tm.ignore_lights_percentage(self._target, 100)
            tm.ignore_signs_percentage(self._target, 100)
            tm.auto_lane_change(self._target, False)
            self._target.set_autopilot(True, TM_PORT)
            tm.set_desired_speed(self._target, self._p['ego_speed_kmh'])

        self.get_logger().info(
            f'Target spawned: {self._target.type_id} '
            f'at ({loc.x:.1f}, {loc.y:.1f})'
        )

    def _spawn_pedestrian(self, loc: carla.Location, rotation: carla.Rotation,
                          fwd: tuple[float, float]):
        bp_lib = self._world.get_blueprint_library()
        peds = bp_lib.filter('walker.pedestrian.*')
        if not peds:
            self.get_logger().error('No pedestrian blueprints available')
            return

        ped_bp = peds[0]
        if ped_bp.has_attribute('is_invincible'):
            ped_bp.set_attribute('is_invincible', 'false')

        offset = self._p.get('ped_offset_m', 3.0)
        # Spawn to the right of the forward axis
        ped_loc = carla.Location(
            x=loc.x - fwd[1] * offset,   # right = -left = rotate fwd by -90°
            y=loc.y + fwd[0] * offset,
            z=loc.z + 0.5,
        )
        self._target = self._world.try_spawn_actor(
            ped_bp, carla.Transform(ped_loc, rotation)
        )
        if self._target is None:
            self.get_logger().warn('Pedestrian spawn failed — VRU test may be incomplete')
        else:
            self.get_logger().info(
                f'Pedestrian spawned at ({ped_loc.x:.1f}, {ped_loc.y:.1f})'
            )

    def _attach_collision_sensor(self):
        bp_lib = self._world.get_blueprint_library()
        col_bp = bp_lib.find('sensor.other.collision')
        self._col_sensor = self._world.spawn_actor(
            col_bp, carla.Transform(), attach_to=self._ego
        )
        self._col_sensor.listen(self._on_collision)

    def _on_collision(self, evt):
        if self._target and evt.other_actor.id == self._target.id:
            self._collision = True
            self.get_logger().warn(
                f'[{self._scenario}] COLLISION with {evt.other_actor.type_id}'
            )

    # ── Geometry helpers ──────────────────────────────────────────────────────

    def _rel_distance(self) -> float:
        if self._target is None:
            return 999.0
        el = self._ego.get_location()
        tl = self._target.get_location()
        return math.hypot(el.x - tl.x, el.y - tl.y)

    def _lateral_offset(self) -> float:
        """
        Signed lateral distance of target from ego forward axis.
        Positive = target is to the right of ego.
        """
        if self._target is None:
            return 0.0
        tf = self._ego.get_transform()
        yaw = math.radians(tf.rotation.yaw)
        dx = self._target.get_location().x - tf.location.x
        dy = self._target.get_location().y - tf.location.y
        # Project onto ego's right axis
        return -dx * math.sin(yaw) + dy * math.cos(yaw)

    def _ttc(self, rel_dist: float) -> float:
        ev = self._ego.get_velocity()
        ego_spd = math.hypot(ev.x, ev.y)
        if self._target and hasattr(self._target, 'get_velocity'):
            tv = self._target.get_velocity()
            tgt_spd = math.hypot(tv.x, tv.y)
        else:
            tgt_spd = 0.0
        closing = ego_spd - tgt_spd
        if closing <= 0.01:
            return 99.9
        return min(rel_dist / closing, 99.9)

    def _compute_avoidance_steer(self, rel_dist: float, lateral_offset: float) -> float:
        """
        Minimum steering angle (rad) needed to avoid the obstacle.

        Uses bicycle-model approximation:
            steer = atan(2 * L * d_lat / D²)
        where L = wheelbase, d_lat = remaining lateral clearance needed, D = rel_dist.

        Sign convention (ROS): positive = left turn.
        Steer away from the side the target is on.
        """
        L = self._p['wheelbase_m']
        half_w = self._p['vehicle_half_w']
        lane_w = self._p['lane_width_m']

        # Lateral gap still needed to fully clear the obstacle
        d_lat = (lane_w / 2.0 + half_w) - abs(lateral_offset)
        d_lat = max(0.1, d_lat)

        # Steer away from the target
        direction = -1.0 if lateral_offset >= 0.0 else 1.0

        if rel_dist < 0.5:
            return direction * math.radians(450.0)

        steer = direction * math.atan2(2.0 * L * d_lat, rel_dist ** 2)
        return max(-math.radians(450.0), min(math.radians(450.0), steer))

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _cb_steer(self, msg: Float32):
        with self._lock:
            self._steer_actual = msg.data

    # ── 20 Hz main loop ───────────────────────────────────────────────────────

    def _tick(self):
        if self._ego is None or self._finished:
            return
        try:
            self._do_tick()
        except Exception as exc:
            self.get_logger().warn(f'Tick error: {exc}')

    def _do_tick(self):
        rel_dist = self._rel_distance()
        ego_spd = _speed_kmh(self._ego)

        if self._target and hasattr(self._target, 'get_velocity'):
            tgt_spd = _speed_kmh(self._target)
        else:
            tgt_spd = 0.0

        ttc = self._ttc(rel_dist)
        lat_off = self._lateral_offset()
        steer_tgt = self._compute_avoidance_steer(rel_dist, lat_off)

        with self._lock:
            steer_actual = self._steer_actual

        arm_dist = self._p['arm_distance_m']
        if not self._armed and rel_dist <= arm_dist:
            self._armed = True
            self._t0 = time.time()
            self._arm_actions()
            self.get_logger().info(
                f'[{self._scenario}] ARMED  dist={rel_dist:.1f} m  '
                f'ego={ego_spd:.1f} km/h'
            )

        if not self._armed:
            return

        t = time.time() - self._t0

        # Publish guidance topics
        self._pub_target.publish(Float32(data=float(steer_tgt)))
        self._pub_ttc.publish(Float32(data=float(ttc)))
        self._pub_steer_tgt.publish(Float32(data=float(steer_tgt)))
        self._pub_rel_dist.publish(Float32(data=float(rel_dist)))

        self._update_verdict(rel_dist, ego_spd, lat_off, t)

        self._rows.append(dict(
            time_s=round(t, 3),
            scenario=self._scenario,
            ego_speed_kmh=round(ego_spd, 2),
            target_speed_kmh=round(tgt_spd, 2),
            rel_distance_m=round(rel_dist, 3),
            ttc_s=round(ttc, 3),
            steer_actual_rad=round(steer_actual, 4),
            steer_actual_deg=round(math.degrees(steer_actual), 2),
            steer_target_rad=round(steer_tgt, 4),
            steer_target_deg=round(math.degrees(steer_tgt), 2),
            lateral_offset_m=round(lat_off, 3),
            collision=int(self._collision),
            verdict=self._verdict,
        ))

        self.get_logger().info(
            f'[{self._scenario}]  t={t:.1f}s  dist={rel_dist:.1f}m  '
            f'ttc={ttc:.2f}s  spd={ego_spd:.1f}km/h  '
            f'steer_act={math.degrees(steer_actual):.1f}°  '
            f'steer_tgt={math.degrees(steer_tgt):.1f}°  '
            f'[{self._verdict}]',
            throttle_duration_sec=0.5,
        )

        if self._should_end(rel_dist, ego_spd, t):
            self._finish()

    def _arm_actions(self):
        """Scenario-specific actions triggered at arming distance."""
        if self._scenario == 'CCRb' and self._target is not None:
            self._target.set_autopilot(False)
            self._target.apply_control(
                carla.VehicleControl(throttle=0.0, brake=1.0)
            )
            self.get_logger().info('CCRb: target braking at 100%')

        elif self._scenario == 'VRU' and self._target is not None:
            tf = self._ego.get_transform()
            yaw = math.radians(tf.rotation.yaw)
            # Walk left across the road (from right side toward ego path)
            self._target.apply_control(carla.WalkerControl(
                direction=carla.Vector3D(math.sin(yaw), -math.cos(yaw), 0.0),
                speed=self._p['ped_speed_ms'],
            ))
            self.get_logger().info(
                f'VRU: pedestrian walking at {self._p["ped_speed_ms"]} m/s'
            )

    def _update_verdict(self, rel_dist: float, ego_spd: float,
                        lat_off: float, t: float):
        if self._collision:
            self._verdict = 'FAIL_COLLISION'
            return
        if self._scenario == 'LaneChange':
            avoidance = self._p.get('avoidance_lateral_m', 3.5)
            if abs(lat_off) >= avoidance * 0.8:
                self._verdict = 'PASS_AVOIDED'
                return
        if ego_spd < 2.0 and rel_dist > 2.0 and t > 2.0:
            self._verdict = 'PASS_STOPPED'
        elif rel_dist > 30.0 and t > 3.0:
            self._verdict = 'PASS_CLEARED'

    def _should_end(self, rel_dist: float, ego_spd: float, t: float) -> bool:
        if self._collision:
            return True
        if t > 30.0:
            return True
        if 'PASS' in self._verdict:
            return True
        return False

    def _finish(self):
        self._finished = True
        self.get_logger().info(
            f'[{self._scenario}] FINISHED  verdict={self._verdict}  '
            f'samples={len(self._rows)}'
        )
        self._pub_verdict.publish(String(data=self._verdict))
        path = self._write_csv()
        self.get_logger().info(f'CSV → {path}')

    def _write_csv(self) -> Path:
        csv_dir = _workspace_root() / 'isaac' / 'streamdata'
        csv_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = csv_dir / f'ncap_{self._scenario}_{ts}.csv'
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(self._rows)
        return path

    def destroy_node(self):
        if not self._finished and self._rows:
            self._write_csv()
        if self._col_sensor:
            try:
                self._col_sensor.destroy()
            except Exception:
                pass
        if self._target:
            try:
                self._target.destroy()
            except Exception:
                pass
        if self._ego_owned and self._ego:
            try:
                self._ego.set_autopilot(False)
                self._ego.destroy()
            except Exception:
                pass
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def run_one(scenario: str, ego_speed: float, spawn_index: int,
            initial_gap: float) -> None:
    rclpy.init()
    node = NCAPRunner(scenario, ego_speed, spawn_index, initial_gap)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='NCAP scenario runner for CARLA + G29 steering wheel'
    )
    parser.add_argument(
        '--scenario', choices=SCENARIOS + ['all'], default='CCRs',
        help='NCAP scenario name, or "all" to run every scenario in sequence'
    )
    parser.add_argument(
        '--ego-speed', type=float, default=0.0,
        help='Override ego speed km/h (0 = use NCAP default for the scenario)'
    )
    parser.add_argument(
        '--spawn-index', type=int, default=0,
        help='CARLA spawn point index used when no hero vehicle is found'
    )
    parser.add_argument(
        '--initial-gap', type=float, default=0.0,
        help='Override initial gap to target in metres (0 = use scenario default)'
    )
    args, ros_args = parser.parse_known_args()

    if args.scenario == 'all':
        for s in SCENARIOS:
            print(f'\n{"="*60}\nStarting NCAP scenario: {s}\n{"="*60}')
            subprocess.run([
                sys.executable, __file__,
                '--scenario', s,
                '--ego-speed', str(args.ego_speed),
                '--spawn-index', str(args.spawn_index),
                '--initial-gap', str(args.initial_gap),
            ] + ros_args, check=False)
            time.sleep(5.0)
    else:
        run_one(args.scenario, args.ego_speed, args.spawn_index, args.initial_gap)


if __name__ == '__main__':
    main()
