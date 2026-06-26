from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    carla_host = LaunchConfiguration('carla_host')
    carla_port = LaunchConfiguration('carla_port')
    ego_role = LaunchConfiguration('ego_role_name')

    return LaunchDescription([
        DeclareLaunchArgument('carla_host', default_value='localhost',
                              description='Hostname of the CARLA server'),
        DeclareLaunchArgument('carla_port', default_value='2000',
                              description='Port of the CARLA server'),
        DeclareLaunchArgument('ego_role_name', default_value='hero',
                              description='role_name attribute of the ego vehicle in CARLA'),

        # G29 steering + pedals → CARLA vehicle control
        Node(
            package='carla_steeringwheel_bridge',
            executable='carla_vehicle_bridge',
            name='carla_vehicle_bridge',
            parameters=[{
                'carla_host': carla_host,
                'carla_port': carla_port,
                'ego_role_name': ego_role,
            }],
            output='screen',
        ),

        # CARLA sensors → ROS2 topics
        Node(
            package='carla_steeringwheel_bridge',
            executable='carla_sensor_bridge',
            name='carla_sensor_bridge',
            parameters=[{
                'carla_host': carla_host,
                'carla_port': carla_port,
                'ego_role_name': ego_role,
            }],
            output='screen',
        ),

        # CARLA vehicle pose → Piper MoveIt reference frame
        Node(
            package='carla_steeringwheel_bridge',
            executable='carla_piper_bridge',
            name='carla_piper_bridge',
            parameters=[{
                'carla_host': carla_host,
                'carla_port': carla_port,
                'ego_role_name': ego_role,
            }],
            output='screen',
        ),
    ])
