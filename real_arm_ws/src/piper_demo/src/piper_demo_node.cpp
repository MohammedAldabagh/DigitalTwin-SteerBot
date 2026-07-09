#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <thread>
#include <map>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("piper_demo");

  std::string group_name = node->declare_parameter<std::string>("move_group_name", "arm");
  RCLCPP_INFO(node->get_logger(), "Using MoveIt group: %s", group_name.c_str());

  std::this_thread::sleep_for(1s);

  moveit::planning_interface::MoveGroupInterface move_group(node, group_name);
  move_group.setPlanningTime(5.0);

  // Ziel 1: zero
  std::map<std::string, double> joints_zero;
  joints_zero["joint1"] = 0.0;
  joints_zero["joint2"] = 0.0;
  joints_zero["joint3"] = 0.0;
  joints_zero["joint4"] = 0.0;
  joints_zero["joint5"] = 0.0;
  joints_zero["joint6"] = 0.0;

  // Ziel 2: Pose (Grad -> Rad)
  std::map<std::string, double> joints_pose;
  joints_pose["joint1"] = 0.0698;
  joints_pose["joint2"] = 0.8727;
  joints_pose["joint3"] = -1.2741;
  joints_pose["joint4"] = -0.0524;
  joints_pose["joint5"] = 1.0472;
  joints_pose["joint6"] = 0.0698;

  bool to_pose = true;
  rclcpp::Rate rate(0.1);

  while (rclcpp::ok())
  {
    const auto &goal = to_pose ? joints_pose : joints_zero;
    RCLCPP_INFO(node->get_logger(), "Planning to %s", to_pose ? "RVIZ POSE" : "ZERO");

    move_group.setJointValueTarget(goal);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Executing...");
      auto exec_result = move_group.execute(plan);
      if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_WARN(node->get_logger(), "Execution failed.");
      }
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Planning failed for this goal.");
    }

    to_pose = !to_pose;
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

