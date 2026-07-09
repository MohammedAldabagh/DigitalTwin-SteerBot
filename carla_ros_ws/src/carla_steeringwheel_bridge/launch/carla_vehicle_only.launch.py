from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('carla_host', default_value='localhost',
                              description='Hostname of the CARLA server'),
        DeclareLaunchArgument('carla_port', default_value='2000',
                              description='Port of the CARLA server'),
        DeclareLaunchArgument('ego_role_name', default_value='hero',
                              description='role_name attribute of the ego vehicle in CARLA'),

        Node(
            package='carla_steeringwheel_bridge',
            executable='carla_vehicle_bridge',
            name='carla_vehicle_bridge',
            parameters=[{
                'carla_host': LaunchConfiguration('carla_host'),
                'carla_port': LaunchConfiguration('carla_port'),
                'ego_role_name': LaunchConfiguration('ego_role_name'),
            }],
            output='screen',
        ),
    ])
