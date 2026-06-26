from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="rotate",
        description="Mode: rotate | hold | rotate_only | ai_hold | servo_hold"
    )## servo_hold uses the wrist_servo joint to track /wheel/target_angle — arm body stays still.

    wheel_tf_frame_arg = DeclareLaunchArgument(
        "wheel_tf_frame",
        default_value="g29_joint_axis",
        description="Wheel frame name used by TF lookup"
    )## TF means Transform. The node will look up the wheel pose from TF using this frame name. It should be the same as the frame name used by the TF broadcaster that publishes the wheel pose. If using the optional static wheel TF, it should be the same as 'wheel.frame'.
    wheel_frame_arg = DeclareLaunchArgument(
        "wheel_frame",
        default_value="world",
        description="Parent frame for optional static wheel TF"
    )
    wheel_publish_static_tf_arg = DeclareLaunchArgument(
        "wheel_publish_static_tf",
        default_value="true",
        description="Publish static wheel TF from center/quaternion parameters"
    )

    wheel_center_x_arg = DeclareLaunchArgument("wheel_center_x", default_value="0.63854") 
    wheel_center_y_arg = DeclareLaunchArgument("wheel_center_y", default_value="0.0")
    wheel_center_z_arg = DeclareLaunchArgument("wheel_center_z", default_value="0.85729")
    start_angle_deg_arg = DeclareLaunchArgument("start_angle_deg", default_value="90.0") ## used 
    approach_offset_arg = DeclareLaunchArgument("approach_offset", default_value="0.10")##  
    rim_inset_arg = DeclareLaunchArgument("rim_inset", default_value="0.015")## 
    tcp_local_z_arg = DeclareLaunchArgument("tcp_local_z", default_value="0.0")##
    radius_arg = DeclareLaunchArgument("radius", default_value="0.13")
    rotate_deg_arg = DeclareLaunchArgument("rotate_deg", default_value="-55.0")
    rotate_steps_arg = DeclareLaunchArgument("rotate_steps", default_value="24")

    eef_step_arg = DeclareLaunchArgument("eef_step", default_value="0.01")
    min_fraction_arg = DeclareLaunchArgument("min_fraction", default_value="0.10")
    speed_fast_arg = DeclareLaunchArgument("speed_fast", default_value="0.2")
    speed_slow_arg = DeclareLaunchArgument("speed_slow", default_value="0.1")

    gripper_close_joint7_arg = DeclareLaunchArgument("gripper_close_joint7", default_value="0.0")
    gripper_close_joint8_arg = DeclareLaunchArgument("gripper_close_joint8", default_value="0.0")

    servo_joint_arg = DeclareLaunchArgument(
        "servo_joint", default_value="wrist_servo",
        description="Extra servo joint name for servo_hold mode")
    servo_scale_arg = DeclareLaunchArgument(
        "servo_scale", default_value="1.0",
        description="wheel_target_rad * servo_scale = servo_joint_rad (use <1.0 if gear ratio)")
    servo_max_step_arg = DeclareLaunchArgument(
        "servo_max_step_rad", default_value="0.15",
        description="Max servo joint delta per control tick (~8.6 deg)")
    servo_tol_arg = DeclareLaunchArgument(
        "servo_tol_rad", default_value="0.01",
        description="Servo dead-band below which no move is sent")

    enable_position_sensor_arg = DeclareLaunchArgument(
        "enable_position_sensor",
        default_value="true",
        description="Enable TF-based end-effector position sensor node"
    )
    sensor_parent_frame_arg = DeclareLaunchArgument(
        "sensor_parent_frame",
        default_value="world",
        description="Parent frame for position sensor TF lookup"
    )
    sensor_child_frame_arg = DeclareLaunchArgument(
        "sensor_child_frame",
        default_value="link6",
        description="Child frame for position sensor TF lookup (typically end-effector link)"
    )
    sensor_topic_arg = DeclareLaunchArgument(
        "sensor_topic",
        default_value="/piper/ee_pose",
        description="Output topic for detected pose"
    )
    sensor_rate_hz_arg = DeclareLaunchArgument(
        "sensor_rate_hz",
        default_value="30.0",
        description="Position sensor publish rate in Hz"
    )

    mode = LaunchConfiguration("mode")
    wheel_tf_frame = LaunchConfiguration("wheel_tf_frame")
    wheel_frame = LaunchConfiguration("wheel_frame")
    wheel_publish_static_tf = LaunchConfiguration("wheel_publish_static_tf")
    wheel_center_x = LaunchConfiguration("wheel_center_x")
    wheel_center_y = LaunchConfiguration("wheel_center_y")
    wheel_center_z = LaunchConfiguration("wheel_center_z")
    start_angle_deg = LaunchConfiguration("start_angle_deg")
    approach_offset = LaunchConfiguration("approach_offset")
    rim_inset = LaunchConfiguration("rim_inset")
    tcp_local_z = LaunchConfiguration("tcp_local_z")
    radius = LaunchConfiguration("radius")
    rotate_deg = LaunchConfiguration("rotate_deg")
    rotate_steps = LaunchConfiguration("rotate_steps")

    eef_step = LaunchConfiguration("eef_step")
    min_fraction = LaunchConfiguration("min_fraction")
    speed_fast = LaunchConfiguration("speed_fast")
    speed_slow = LaunchConfiguration("speed_slow")

    gripper_close_joint7 = LaunchConfiguration("gripper_close_joint7")
    gripper_close_joint8 = LaunchConfiguration("gripper_close_joint8")
    servo_joint        = LaunchConfiguration("servo_joint")
    servo_scale        = LaunchConfiguration("servo_scale")
    servo_max_step_rad = LaunchConfiguration("servo_max_step_rad")
    servo_tol_rad      = LaunchConfiguration("servo_tol_rad")
    enable_position_sensor = LaunchConfiguration("enable_position_sensor")
    sensor_parent_frame = LaunchConfiguration("sensor_parent_frame")
    sensor_child_frame = LaunchConfiguration("sensor_child_frame")
    sensor_topic = LaunchConfiguration("sensor_topic")
    sensor_rate_hz = LaunchConfiguration("sensor_rate_hz")

    return LaunchDescription([
        mode_arg,
        wheel_tf_frame_arg,
        wheel_frame_arg,
        wheel_publish_static_tf_arg,
        wheel_center_x_arg,
        wheel_center_y_arg,
        wheel_center_z_arg,
        start_angle_deg_arg,
        approach_offset_arg,
        rim_inset_arg,
        tcp_local_z_arg,
        radius_arg,
        rotate_deg_arg,
        rotate_steps_arg,
        eef_step_arg,
        min_fraction_arg,
        speed_fast_arg,
        speed_slow_arg,
        gripper_close_joint7_arg,
        gripper_close_joint8_arg,
        servo_joint_arg,
        servo_scale_arg,
        servo_max_step_arg,
        servo_tol_arg,
        enable_position_sensor_arg,
        sensor_parent_frame_arg,
        sensor_child_frame_arg,
        sensor_topic_arg,
        sensor_rate_hz_arg,
        Node(
            package="piper_demo",
            executable="piper_grab_rotate_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.trajectory_execution,
                {"use_sim_time": True},
                {"mode": mode},
                {"wheel.tf_frame": wheel_tf_frame},
                {"wheel.frame": wheel_frame},
                {"wheel.publish_static_tf": wheel_publish_static_tf},
                {"wheel.center_x": ParameterValue(wheel_center_x, value_type=float)},
                {"wheel.center_y": ParameterValue(wheel_center_y, value_type=float)},
                {"wheel.center_z": ParameterValue(wheel_center_z, value_type=float)},
                {"start_angle_deg": ParameterValue(start_angle_deg, value_type=float)},
                {"approach_offset": ParameterValue(approach_offset, value_type=float)},
                {"rim_inset": ParameterValue(rim_inset, value_type=float)},
                {"tcp_local_z": ParameterValue(tcp_local_z, value_type=float)},
                {"radius": ParameterValue(radius, value_type=float)},
                {"rotate_deg": ParameterValue(rotate_deg, value_type=float)},
                {"rotate_steps": ParameterValue(rotate_steps, value_type=int)},
                {"motion.eef_step": ParameterValue(eef_step, value_type=float)},
                {"motion.min_fraction": ParameterValue(min_fraction, value_type=float)},
                {"motion.fast": ParameterValue(speed_fast, value_type=float)},
                {"motion.slow": ParameterValue(speed_slow, value_type=float)},
                {"gripper.close_joint7": ParameterValue(gripper_close_joint7, value_type=float)},
                {"gripper.close_joint8": ParameterValue(gripper_close_joint8, value_type=float)},
                {"servo.joint":        servo_joint},
                {"servo.scale":        ParameterValue(servo_scale,        value_type=float)},
                {"servo.max_step_rad": ParameterValue(servo_max_step_rad, value_type=float)},
                {"servo.tol_rad":      ParameterValue(servo_tol_rad,      value_type=float)},
            ],
        )
        ,
        Node(
            package="piper_demo",
            executable="piper_position_sensor_node",
            output="screen",
            condition=IfCondition(enable_position_sensor),
            parameters=[
                {"use_sim_time": True},
                {"parent_frame": sensor_parent_frame},
                {"child_frame": sensor_child_frame},
                {"topic_name": sensor_topic},
                {"rate_hz": ParameterValue(sensor_rate_hz, value_type=float)},
            ],
        ),
    ])
