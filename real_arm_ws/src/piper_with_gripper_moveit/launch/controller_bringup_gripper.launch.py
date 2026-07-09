from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    fake_hardware = LaunchConfiguration("fake_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .to_moveit_configs()
    )

    pkg_share = get_package_share_directory("piper_with_gripper_moveit")
    controllers_yaml = PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])

    # Optional aber empfehlenswert (nur EINMAL im Gesamtsystem starten!)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"ignore_timestamp": True},
            moveit_config.robot_description,
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            controllers_yaml,
            # Achtung: das wirkt nur, wenn dein URDF/xacro dieses Param wirklich nutzt.
            # Wenn nicht: lieber über xacro-mappings im robot_description lösen.
            {"fake_hardware": fake_hardware},
        ],
    )

    # robuster spawner (timeout)
    spawner_common = [
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "120",
    ]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", *spawner_common],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", *spawner_common],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", *spawner_common],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    jsb_delayed = TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner])
    arm_and_gripper_delayed = TimerAction(
        period=4.0,
        actions=[arm_controller_spawner, gripper_controller_spawner],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fake_hardware",
                default_value="true",
                description="Use fake ros2_control hardware",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock (Isaac/Gazebo). Set false for fake sim without a simulator.",
            ),

            # rsp,  # <- aktivieren, wenn du ihn nicht schon im MoveIt-Launch startest
            ros2_control_node,
            jsb_delayed,
            arm_and_gripper_delayed,
        ]
    )
