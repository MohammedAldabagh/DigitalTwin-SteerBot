#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <cmath>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <atomic>

class PiperGrabRotate
{
public:
  struct Wheel
  {
    std::string frame = "world";
    std::string tf_frame = "g29_joint_axis";
    bool publish_static_tf = true;
    tf2::Vector3 center{0.63854, 0.0, 0.85729};
    tf2::Quaternion q{0.369082, -0.369084, -0.603141, 0.603139};
  };

  struct Motion
  {
    double eef_step = 0.01;
    double jump_thresh = 0.0;
    double min_fraction = 0.10;

    double fast = 0.2;
    double slow = 0.1;

    double planning_time_s = 20.0;
    int planning_attempts = 20;
  };

  struct Gripper
  {
    std::string group = "gripper";
    std::map<std::string,double> open{{"joint7", 0.035}, {"joint8",-0.035}};
    std::map<std::string,double> close{{"joint7", 0.0},  {"joint8", 0.0}};
  };

  struct Servo
  {
    std::string joint = "wrist_servo";
    double scale = 1.0;
    double max_step_rad = 0.15;
    double tol_rad = 0.01;
  };

  struct Config
  {
    std::string arm_group = "arm";
    std::string ee_link_override;

    Wheel wheel;

    double radius = 0.16;
    double start_angle_deg = 90.0;

    double rotate_deg = -55.0;
    int rotate_steps = 12;

    double approach_offset = 0.10;
    double rim_inset = 0.0;
    double tcp_local_z = 0.0;

    Motion motion;
    Gripper gripper;
    Servo  servo;
  };

  PiperGrabRotate(rclcpp::Node::SharedPtr node, Config cfg);

  bool run();
  bool runHold();
  bool runRotateOnly();
  bool runAIHold();
  bool runServoHold();

private:
  struct WheelState
  {
    tf2::Vector3 c;
    tf2::Quaternion q;
    tf2::Vector3 n;
  };

  WheelState wheelFromTf() const;

  tf2::Vector3 rimPoint(const WheelState& ws, double angle_rad) const;
  void rimFrame(const WheelState& ws, const tf2::Vector3& contact,
                tf2::Vector3& r_out, tf2::Vector3& t_out) const;

  geometry_msgs::msg::PoseStamped makeApproachPose(const WheelState& ws,
                                                   const geometry_msgs::msg::PoseStamped& seed,
                                                   double angle_rad) const;

  geometry_msgs::msg::PoseStamped makeGraspPose(const WheelState& ws,
                                                const geometry_msgs::msg::PoseStamped& approach) const;

  geometry_msgs::msg::Quaternion makeGraspOrientation(const WheelState& ws,
                                                      const tf2::Vector3& contact) const;

  void applyTcpLocalZ(geometry_msgs::msg::PoseStamped& p) const;

  void setSpeed(double scale);
  bool moveToPose(const geometry_msgs::msg::PoseStamped& pose);
  bool moveToJoints(const std::map<std::string, double>& joints);
  void moveGripper(const std::map<std::string, double>& target);

  bool rotateArcCartesian(const WheelState& ws,
                          const geometry_msgs::msg::PoseStamped& grasp_pose,
                          double start_angle_rad);

  bool execTraj(const moveit_msgs::msg::RobotTrajectory& traj, const char* tag);

  bool cartesianTo(const geometry_msgs::msg::Pose& target,
                   const char* tag,
                   double eef_step = -1.0,
                   double jump_thresh = -1.0,
                   double min_fraction = -1.0);

  double angleOnWheel(const WheelState& ws,
                      const geometry_msgs::msg::PoseStamped& tcp) const;

  bool nudgeJoint(const std::string& joint_name,
                  double delta_rad,
                  double speed_scale,
                  bool clamp=true);

  bool holdWheelAngle(const WheelState& ws,
                      const geometry_msgs::msg::PoseStamped& grasp_ref,
                      double wheel_grasp_rad,
                      double a_grasp_rad,
                      double wheel_target_rad);

  bool aiHoldWheelAngle(const WheelState& ws,
                        const geometry_msgs::msg::PoseStamped& grasp_ref,
                        double a_grasp_rad,
                        double wheel_target_deg);

  bool servoHoldLoop(int servo_idx, double servo_zero_rad);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  Config cfg_;

  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_;

  std::string planning_frame_;
  std::string ee_link_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_sub_;
  std::atomic<double> wheel_pos_rad_{0.0};
  std::atomic<bool> wheel_pos_valid_{false};
  std::string wheel_joint_name_ = "RevoluteJoint";

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wheel_position_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_position_pub_;
  std::atomic<double> wheel_position_deg_{0.0};
  std::atomic<bool> wheel_position_valid_{false};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ai_action_sub_;
  std::atomic<double> ai_action_deg_{0.0};
  std::atomic<bool> ai_action_valid_{false};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wheel_target_sub_;
  std::atomic<double> wheel_target_rad_{0.0};
  std::atomic<bool>   wheel_target_valid_{false};
};