#!/usr/bin/env python3
"""
Scenario: VW T2 Bus driving forward at 15 km/h.
- Spawns VW T2 Bus and drives at 15 km/h via Traffic Manager
- Spectator camera follows the bus from behind/above
- Publishes position + orientation to ROS2

Topics published:
  /carla/vw_bus/pose      (geometry_msgs/PoseStamped)
  /carla/vw_bus/odom      (nav_msgs/Odometry)
  /carla/vw_bus/speed_kmh (std_msgs/Float32)

Usage:
  python3 vw_bus_scenario.py
  python3 vw_bus_scenario.py --spawn-index 5
  python3 vw_bus_scenario.py --camera-height 12 --camera-distance 8
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

import carla

TARGET_SPEED_KMH = 15.0
TM_PORT = 8000


def euler_to_quat(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class VWBusScenario(Node):

    def __init__(self, spawn_index: int = 0,
                 camera_height: float = 10.0,
                 camera_distance: float = 8.0,
                 camera_pitch: float = -35.0):
        super().__init__('vw_bus_scenario')

        self.pub_pose = self.create_publisher(PoseStamped, '/carla/vw_bus/pose', 10)
        self.pub_odom = self.create_publisher(Odometry, '/carla/vw_bus/odom', 10)
        self.pub_speed = self.create_publisher(Float32, '/carla/vw_bus/speed_kmh', 10)

        self._bus = None
        self._world = None
        self._spectator = None
        self._running = True

        self._cam_height = camera_height
        self._cam_dist = camera_distance
        self._cam_pitch = camera_pitch
        self._spawn_index = spawn_index

        self._setup()

        # Camera follow loop in a background thread (50 Hz)
        self._cam_thread = threading.Thread(target=self._camera_follow_loop, daemon=True)
        self._cam_thread.start()

        # ROS2 publish timer at 20 Hz
        self.create_timer(0.05, self._publish_state)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _setup(self):
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self._world = client.get_world()
        self._spectator = self._world.get_spectator()

        bp_lib = self._world.get_blueprint_library()

        try:
            bp = bp_lib.find('vehicle.volkswagen.t2_2021')
        except Exception:
            bp = bp_lib.find('vehicle.volkswagen.t2')
        bp.set_attribute('role_name', 'vw_bus')

        spawn_points = self._world.get_map().get_spawn_points()
        if not spawn_points:
            self.get_logger().error('No spawn points in map!')
            return

        idx = self._spawn_index % len(spawn_points)
        for offset in range(len(spawn_points)):
            sp = spawn_points[(idx + offset) % len(spawn_points)]
            self._bus = self._world.try_spawn_actor(bp, sp)
            if self._bus is not None:
                self.get_logger().info(
                    f'Spawned {self._bus.type_id} (id={self._bus.id}) at '
                    f'({sp.location.x:.1f}, {sp.location.y:.1f}) '
                    f'yaw={sp.rotation.yaw:.1f} deg'
                )
                break

        if self._bus is None:
            self.get_logger().error('Failed to spawn VW Bus!')
            return

        # Traffic Manager — exact 15 km/h, ignore lights, stay in lane
        tm = client.get_trafficmanager(TM_PORT)
        tm.set_synchronous_mode(False)
        tm.set_global_distance_to_leading_vehicle(2.5)
        tm.ignore_lights_percentage(self._bus, 100)
        tm.ignore_signs_percentage(self._bus, 100)
        tm.auto_lane_change(self._bus, False)

        self._bus.set_autopilot(True, TM_PORT)
        tm.set_desired_speed(self._bus, TARGET_SPEED_KMH)

        self.get_logger().info(
            f'VW Bus autopilot ON — target: {TARGET_SPEED_KMH} km/h'
        )

    # ------------------------------------------------------------------
    # Spectator camera — runs in background thread
    # ------------------------------------------------------------------

    def _camera_follow_loop(self):
        """
        Positions the CARLA spectator camera behind and above the VW Bus,
        looking slightly down at it. Updates at ~50 Hz.
        """
        while self._running:
            if self._bus is None or self._world is None:
                time.sleep(0.05)
                continue
            try:
                tf = self._bus.get_transform()

                # Forward vector of the bus (unit vector)
                yaw_rad = math.radians(tf.rotation.yaw)
                fwd_x = math.cos(yaw_rad)
                fwd_y = math.sin(yaw_rad)

                # Place camera behind the bus by cam_distance and above by cam_height
                cam_loc = carla.Location(
                    x=tf.location.x - fwd_x * self._cam_dist,
                    y=tf.location.y - fwd_y * self._cam_dist,
                    z=tf.location.z + self._cam_height,
                )
                cam_rot = carla.Rotation(
                    pitch=self._cam_pitch,
                    yaw=tf.rotation.yaw,
                    roll=0.0,
                )
                self._spectator.set_transform(carla.Transform(cam_loc, cam_rot))
            except Exception:
                pass
            time.sleep(0.02)

    # ------------------------------------------------------------------
    # ROS2 publisher
    # ------------------------------------------------------------------

    def _publish_state(self):
        if self._bus is None:
            return
        try:
            tf = self._bus.get_transform()
            vel = self._bus.get_velocity()
        except Exception as exc:
            self.get_logger().warn(f'Lost VW Bus: {exc}')
            self._bus = None
            return

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = euler_to_quat(
            tf.rotation.roll, tf.rotation.pitch, tf.rotation.yaw
        )

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'carla_world'
        pose.pose.position.x = tf.location.x
        pose.pose.position.y = tf.location.y
        pose.pose.position.z = tf.location.z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pub_pose.publish(pose)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'carla_world'
        odom.child_frame_id = 'vw_bus'
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = vel.x
        odom.twist.twist.linear.y = vel.y
        odom.twist.twist.linear.z = vel.z
        self.pub_odom.publish(odom)

        speed_kmh = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2) * 3.6
        self.pub_speed.publish(Float32(data=float(speed_kmh)))

        self.get_logger().info(
            f'Pos=({tf.location.x:.1f}, {tf.location.y:.1f}) '
            f'Yaw={tf.rotation.yaw:.1f} deg  '
            f'Speed={speed_kmh:.1f} km/h',
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._running = False
        if self._bus is not None:
            try:
                self._bus.set_autopilot(False)
                self._bus.destroy()
                self.get_logger().info('VW Bus removed from CARLA world')
            except Exception:
                pass
        super().destroy_node()


# ----------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='VW Bus 15 km/h scenario with camera follow')
    parser.add_argument('--spawn-index', type=int, default=0,
                        help='Spawn point index 0-154 (Town10HD)')
    parser.add_argument('--camera-height', type=float, default=10.0,
                        help='Camera height above bus in metres (default 10)')
    parser.add_argument('--camera-distance', type=float, default=8.0,
                        help='Camera distance behind bus in metres (default 8)')
    parser.add_argument('--camera-pitch', type=float, default=-35.0,
                        help='Camera pitch angle in degrees (default -35)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = VWBusScenario(
        spawn_index=args.spawn_index,
        camera_height=args.camera_height,
        camera_distance=args.camera_distance,
        camera_pitch=args.camera_pitch,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
