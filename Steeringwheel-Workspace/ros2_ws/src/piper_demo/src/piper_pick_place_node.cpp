#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <thread>
#include <map>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

static bool plan_and_execute_pose(moveit::planning_interface::MoveGroupInterface& arm,
                                 const geometry_msgs::msg::PoseStamped& pose,
                                 const std::string& ee_link,
                                 rclcpp::Logger logger)
{
  arm.setStartStateToCurrentState();
  arm.clearPoseTargets();
  arm.setPoseTarget(pose, ee_link);

  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto res = arm.plan(plan);
  if (res == moveit::core::MoveItErrorCode::SUCCESS)
  {
    auto exec_res = arm.execute(plan);
    if (exec_res != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(logger, "Execution failed.");
      return false;
    }
    return true;
  }

  RCLCPP_WARN(logger, "Planning failed.");
  return false;
}

static bool plan_and_execute_joints(moveit::planning_interface::MoveGroupInterface& arm,
                                   const std::map<std::string, double>& joints,
                                   rclcpp::Logger logger)
{
  arm.setStartStateToCurrentState();
  arm.clearPoseTargets();
  arm.setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto res = arm.plan(plan);
  if (res == moveit::core::MoveItErrorCode::SUCCESS)
  {
    auto exec_res = arm.execute(plan);
    if (exec_res != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(logger, "Execution failed.");
      return false;
    }
    return true;
  }

  RCLCPP_WARN(logger, "Planning failed.");
  return false;
}

static void move_gripper(moveit::planning_interface::MoveGroupInterface& gripper,
                         const std::map<std::string, double>& target,
                         rclcpp::Logger logger)
{
  gripper.setStartStateToCurrentState();
  gripper.setJointValueTarget(target);
  (void)gripper.move();
  RCLCPP_INFO(logger, "Gripper command sent.");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("piper_pick_place_node");

  std::string arm_group = node->declare_parameter<std::string>("arm_group", "arm");
  std::string gripper_group = node->declare_parameter<std::string>("gripper_group", "gripper");

  RCLCPP_INFO(node->get_logger(), "Arm group: %s", arm_group.c_str());
  RCLCPP_INFO(node->get_logger(), "Gripper group: %s", gripper_group.c_str());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  std::this_thread::sleep_for(2s);

  moveit::planning_interface::MoveGroupInterface arm(node, arm_group);
  moveit::planning_interface::MoveGroupInterface gripper(node, gripper_group);

  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(10);

  const std::string planning_frame = arm.getPlanningFrame();
  const std::string ee_link = arm.getEndEffectorLink();

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", planning_frame.c_str());
  RCLCPP_INFO(node->get_logger(), "EE link: %s", ee_link.c_str());

  
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);

  

  const double can_x = 0.45;     
  const double can_y = 0.00;    
  const double can_z = 0.35;
  
  
  
  // const double can_x = 0.75228;
  //const double can_y = -0.15218;
  //const double can_z = 0.80589;

  const double z_offset = 0.10;
  const double forward_offset_local_z = 0.075;

  const double place_dx = 0.20;
  const double place_dy = 0.20;

  const double place_down_extra = 0.00;

  std::map<std::string, double> joints_home;
  joints_home["joint1"] = 0.0;
  joints_home["joint2"] = 0.0;
  joints_home["joint3"] = 0.0;
  joints_home["joint4"] = 0.0;
  joints_home["joint5"] = 0.0;
  joints_home["joint6"] = 0.0;

  std::map<std::string, double> gripper_open;
  gripper_open["joint7"] = 0.035;
  gripper_open["joint8"] = -0.035;

  std::map<std::string, double> gripper_close;
  gripper_close["joint7"] = 0.0;
  gripper_close["joint8"] = 0.0;

  auto current_pose = arm.getCurrentPose(ee_link);

  geometry_msgs::msg::PoseStamped pick_hover;
  pick_hover.header.frame_id = planning_frame;
  pick_hover.pose = current_pose.pose;
  pick_hover.pose.position.x = can_x;
  pick_hover.pose.position.y = can_y;
  pick_hover.pose.position.z = can_z + z_offset;

  geometry_msgs::msg::PoseStamped pick_grasp = pick_hover;
  pick_grasp.pose.position.z = can_z;

  {
    tf2::Quaternion q;
    tf2::fromMsg(pick_hover.pose.orientation, q);
    tf2::Matrix3x3 R(q);

    tf2::Vector3 offset_local(0.0, 0.0, forward_offset_local_z);
    tf2::Vector3 offset_world = R * offset_local;

    auto apply = [&](geometry_msgs::msg::PoseStamped& p)
    {
      p.pose.position.x += offset_world.x();
      p.pose.position.y += offset_world.y();
      p.pose.position.z += offset_world.z();
    };

    apply(pick_hover);
    apply(pick_grasp);

    RCLCPP_INFO(node->get_logger(),
                "Applied local-Z offset %.3f => world dx=%.3f dy=%.3f dz=%.3f",
                forward_offset_local_z, offset_world.x(), offset_world.y(), offset_world.z());
  }

  geometry_msgs::msg::PoseStamped place_hover = pick_hover;
  place_hover.pose.position.x = can_x + place_dx;
  place_hover.pose.position.y = can_y + place_dy;

  geometry_msgs::msg::PoseStamped place_down = place_hover;
  place_down.pose.position.z = can_z + place_down_extra;

  RCLCPP_INFO(node->get_logger(), "A) Gripper öffnen (Start)");
  move_gripper(gripper, gripper_open, node->get_logger());
  std::this_thread::sleep_for(500ms);

  RCLCPP_INFO(node->get_logger(), "1) Hover über Dose");
  if (!plan_and_execute_pose(arm, pick_hover, ee_link, node->get_logger())) goto shutdown;

  RCLCPP_INFO(node->get_logger(), "2) Runterfahren (Pick)");
  if (!plan_and_execute_pose(arm, pick_grasp, ee_link, node->get_logger())) goto shutdown;

  RCLCPP_INFO(node->get_logger(), "3) Greifer schließen (Greifen)");
  move_gripper(gripper, gripper_close, node->get_logger());
  std::this_thread::sleep_for(600ms);
/*
  RCLCPP_INFO(node->get_logger(), "4) Senkrecht hoch (zurück zum Hover)");
  if (!plan_and_execute_pose(arm, pick_hover, ee_link, node->get_logger())) goto shutdown;

  RCLCPP_INFO(node->get_logger(), "5) Über Ablageposition fahren (Hover)");
  if (!plan_and_execute_pose(arm, place_hover, ee_link, node->get_logger())) goto shutdown;

  RCLCPP_INFO(node->get_logger(), "6) Langsam runter (Place)");
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);
  if (!plan_and_execute_pose(arm, place_down, ee_link, node->get_logger())) goto shutdown;
  arm.setMaxVelocityScalingFactor(1.0);
  arm.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(node->get_logger(), "7) Loslassen (Gripper öffnen)");
  move_gripper(gripper, gripper_open, node->get_logger());
  std::this_thread::sleep_for(600ms);

  RCLCPP_INFO(node->get_logger(), "8) Hochfahren (Place Hover)");
  if (!plan_and_execute_pose(arm, place_hover, ee_link, node->get_logger())) goto shutdown;

  RCLCPP_INFO(node->get_logger(), "9) Gripper schließen (leer)");
  move_gripper(gripper, gripper_close, node->get_logger());
  std::this_thread::sleep_for(400ms);

  RCLCPP_INFO(node->get_logger(), "10) Zurück in Home");
  if (!plan_and_execute_joints(arm, joints_home, node->get_logger())) */

  RCLCPP_INFO(node->get_logger(), "Pick abgeschlossen.");
  goto shutdown;

shutdown:
  exec.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}