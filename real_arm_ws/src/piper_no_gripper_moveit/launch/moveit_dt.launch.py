from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt-Config laden
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_no_gripper_moveit")
        .to_moveit_configs()
    )

    pkg_share = get_package_share_directory("piper_no_gripper_moveit")
    rviz_config = PathJoinSubstitution([pkg_share, "config", "moveit.rviz"])

    # robot_state_publisher mit URDF in /robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # move_group – MoveIt-Config
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz mit KINEMATICS + PLANNING + TRAJECTORY
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            move_group,
            rviz,
        ]
    )

