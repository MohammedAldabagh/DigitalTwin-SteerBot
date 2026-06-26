"""
carla_g29_bus_drive.launch.py

Launches the full G29 → CARLA VW Bus pipeline:
  1. joy_node          — reads physical G29 hardware → /joy
  2. g29_steering_node — /joy → /wheel/steering_angle (radians)
  3. vw_bus_spawner    — spawns VW T2 Bus in CARLA as role=hero (no autopilot)
  4. carla_vehicle_bridge — /wheel/steering_angle + pedals → CARLA VehicleControl

Usage:
  ros2 launch carla_steeringwheel_bridge carla_g29_bus_drive.launch.py
  ros2 launch carla_steeringwheel_bridge carla_g29_bus_drive.launch.py auto_throttle:=0.4
  ros2 launch carla_steeringwheel_bridge carla_g29_bus_drive.launch.py auto_throttle:=0.0  # pedal only
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    carla_host = LaunchConfiguration('carla_host')
    carla_port = LaunchConfiguration('carla_port')
    auto_throttle = LaunchConfiguration('auto_throttle')
    joy_device = LaunchConfiguration('joy_device')
    spawn_index = LaunchConfiguration('spawn_index')

    return LaunchDescription([
        DeclareLaunchArgument('carla_host', default_value='localhost',
                              description='CARLA server hostname'),
        DeclareLaunchArgument('carla_port', default_value='2000',
                              description='CARLA server port'),
        DeclareLaunchArgument('auto_throttle', default_value='0.3',
                              description='Constant forward throttle [0-1]. '
                                          'Set 0.0 to use gas pedal only.'),
        DeclareLaunchArgument('joy_device', default_value='/dev/input/js0',
                              description='G29 joystick device path'),
        DeclareLaunchArgument('spawn_index', default_value='0',
                              description='Spawn point index for the VW Bus'),

        # 1. Physical G29 hardware → /joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device': joy_device}],
            output='screen',
        ),

        # 2. /joy → /wheel/steering_angle (Float32, radians)
        Node(
            package='g29_isaac_bridge',
            executable='g29_steering_node',
            name='g29_steering_node',
            output='screen',
        ),

        # 3. Spawn VW T2 Bus in CARLA with role_name='hero' (manual control)
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

        # 4. /wheel/steering_angle + /joy pedals → CARLA VehicleControl on the bus
        Node(
            package='carla_steeringwheel_bridge',
            executable='carla_vehicle_bridge',
            name='carla_vehicle_bridge',
            parameters=[{
                'carla_host': carla_host,
                'carla_port': carla_port,
                'ego_role_name': 'hero',
                'auto_throttle': auto_throttle,
            }],
            output='screen',
        ),
    ])
