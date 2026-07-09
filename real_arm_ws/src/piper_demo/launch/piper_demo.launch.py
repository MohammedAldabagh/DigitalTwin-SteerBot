from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_no_gripper_moveit")
        .to_moveit_configs()
    )

    demo_node = Node(
        package="piper_demo",
        executable="piper_demo_node",
        name="piper_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            {"move_group_name": "arm"},
        ],
    )

    return LaunchDescription([demo_node])

