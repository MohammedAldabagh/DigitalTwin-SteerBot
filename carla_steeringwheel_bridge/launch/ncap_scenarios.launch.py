"""
ncap_scenarios.launch.py

Launches the full NCAP test pipeline:
  1. joy_node            — reads G29 hardware → /joy
  2. g29_steering_node   — /joy → /wheel/steering_angle (rad)
  3. vw_bus_spawner      — spawns VW T2 Bus in CARLA as role=hero (manual)
  4. carla_vehicle_bridge — steering + pedals → CARLA VehicleControl
  5. ncap_scenario_runner — spawns NCAP target, records steering angle metrics,
                            publishes /wheel/target_angle guidance to G29

Usage:
  ros2 launch carla_steeringwheel_bridge ncap_scenarios.launch.py scenario:=CCRs
  ros2 launch carla_steeringwheel_bridge ncap_scenarios.launch.py scenario:=VRU ego_speed:=40
  ros2 launch carla_steeringwheel_bridge ncap_scenarios.launch.py scenario:=LaneChange
  ros2 launch carla_steeringwheel_bridge ncap_scenarios.launch.py scenario:=CCRb initial_gap:=50

Available scenarios: CCRs  CCRm  CCRb  VRU  LaneChange

ROS2 topics published by ncap_scenario_runner:
  /wheel/target_angle    (Float32, rad)  — avoidance steering guidance to G29
  /ncap/ttc              (Float32, s)    — time to collision
  /ncap/steer_target_rad (Float32, rad)  — same as target_angle, for logging
  /ncap/rel_distance_m   (Float32, m)   — ego-to-target distance
  /ncap/verdict          (String)        — PASS_STOPPED / PASS_AVOIDED / FAIL_COLLISION

CSV output:
  <workspace>/isaac/streamdata/ncap_<scenario>_<timestamp>.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    carla_host = LaunchConfiguration('carla_host')
    carla_port = LaunchConfiguration('carla_port')
    joy_device = LaunchConfiguration('joy_device')
    spawn_index = LaunchConfiguration('spawn_index')
    scenario = LaunchConfiguration('scenario')
    ego_speed = LaunchConfiguration('ego_speed')
    initial_gap = LaunchConfiguration('initial_gap')

    return LaunchDescription([
        DeclareLaunchArgument('carla_host', default_value='localhost',
                              description='CARLA server hostname'),
        DeclareLaunchArgument('carla_port', default_value='2000',
                              description='CARLA server port'),
        DeclareLaunchArgument('joy_device', default_value='/dev/input/js0',
                              description='G29 joystick device path'),
        DeclareLaunchArgument('spawn_index', default_value='0',
                              description='CARLA spawn point index'),
        DeclareLaunchArgument('scenario', default_value='CCRs',
                              description='NCAP scenario: CCRs CCRm CCRb VRU LaneChange'),
        DeclareLaunchArgument('ego_speed', default_value='0.0',
                              description='Override ego speed km/h (0 = NCAP default)'),
        DeclareLaunchArgument('initial_gap', default_value='0.0',
                              description='Override initial gap to target m (0 = default)'),

        # 1. Physical G29 hardware → /joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device': joy_device}],
            output='screen',
        ),

        # 2. /joy → /wheel/steering_angle (Float32, rad)
        Node(
            package='g29_isaac_bridge',
            executable='g29_steering_node',
            name='g29_steering_node',
            output='screen',
        ),

        # 3. Spawn VW T2 Bus as hero (manual control — G29 drives it)
        Node(
            package='carla_steeringwheel_bridge',
            executable='vw_bus_spawner',
            name='vw_bus_spawner',
            parameters=[{
                'carla_host': carla_host,
                'carla_port': carla_port,
                'spawn_index': spawn_index,
            }],
            output='screen',
        ),

        # 4. /wheel/steering_angle + pedals → CARLA VehicleControl
        Node(
            package='carla_steeringwheel_bridge',
            executable='carla_vehicle_bridge',
            name='carla_vehicle_bridge',
            parameters=[{
                'carla_host': carla_host,
                'carla_port': carla_port,
                'ego_role_name': 'hero',
                'auto_throttle': 0.0,
            }],
            output='screen',
        ),

        # 5. NCAP scenario: spawn target, record, publish /wheel/target_angle
        Node(
            package='carla_steeringwheel_bridge',
            executable='ncap_scenario_runner',
            name='ncap_scenario_runner',
            arguments=[
                '--scenario', scenario,
                '--ego-speed', ego_speed,
                '--spawn-index', spawn_index,
                '--initial-gap', initial_gap,
            ],
            output='screen',
        ),
    ])
