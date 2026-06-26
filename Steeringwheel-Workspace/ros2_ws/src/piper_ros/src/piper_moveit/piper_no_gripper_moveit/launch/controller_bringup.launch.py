from launch import LaunchDescription
from launch.actions import (
    TimerAction,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    fake_hw = LaunchConfiguration("fake_hardware")

    # MoveIt-Config -> liefert robot_description
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_no_gripper_moveit")
        .to_moveit_configs()
    )

    pkg_share = get_package_share_directory("piper_no_gripper_moveit")

    controllers_yaml = PathJoinSubstitution(
        [pkg_share, "config", "ros2_controllers.yaml"]
    )

    # robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            controllers_yaml,
        ],
        output="screen",
    )

    # spawner joint_state_broadcaster
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # spawner arm_controller
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_arm_controller",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    jsb_delayed = TimerAction(period=1.0, actions=[jsb_spawner])
    arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[TimerAction(period=0.5, actions=[arm_spawner])],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fake_hardware", default_value=TextSubstitution(text="true")
            ),
            rsp,
            ros2_control_node,
            jsb_delayed,
            arm_after_jsb,
        ]
    )

