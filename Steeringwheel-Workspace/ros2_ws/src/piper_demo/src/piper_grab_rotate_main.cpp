#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "piper_demo/piper_grab_rotate.hpp"

#include <thread>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("piper_grab_rotate_node");

  // Mode-Parameter (rotate | hold | rotate_only | ai_hold)
  const std::string mode = node->declare_parameter<std::string>("mode", "rotate");

  // Runtime tuning for distant wheel grab without recompiling.
  PiperGrabRotate::Config cfg;
  cfg.arm_group = node->declare_parameter<std::string>("arm_group", cfg.arm_group);
  cfg.ee_link_override = node->declare_parameter<std::string>("ee_link_override", cfg.ee_link_override);

  cfg.wheel.frame = node->declare_parameter<std::string>("wheel.frame", cfg.wheel.frame);
  cfg.wheel.tf_frame = node->declare_parameter<std::string>("wheel.tf_frame", cfg.wheel.tf_frame);
  cfg.wheel.publish_static_tf = node->declare_parameter<bool>("wheel.publish_static_tf", cfg.wheel.publish_static_tf);
  cfg.wheel.center.setX(node->declare_parameter<double>("wheel.center_x", cfg.wheel.center.x()));
  cfg.wheel.center.setY(node->declare_parameter<double>("wheel.center_y", cfg.wheel.center.y()));
  cfg.wheel.center.setZ(node->declare_parameter<double>("wheel.center_z", cfg.wheel.center.z()));
  cfg.wheel.q.setX(node->declare_parameter<double>("wheel.q_x", cfg.wheel.q.x()));
  cfg.wheel.q.setY(node->declare_parameter<double>("wheel.q_y", cfg.wheel.q.y()));
  cfg.wheel.q.setZ(node->declare_parameter<double>("wheel.q_z", cfg.wheel.q.z()));
  cfg.wheel.q.setW(node->declare_parameter<double>("wheel.q_w", cfg.wheel.q.w()));

  cfg.radius = node->declare_parameter<double>("radius", cfg.radius);
  cfg.start_angle_deg = node->declare_parameter<double>("start_angle_deg", cfg.start_angle_deg);
  cfg.rotate_deg = node->declare_parameter<double>("rotate_deg", cfg.rotate_deg);
  cfg.rotate_steps = node->declare_parameter<int>("rotate_steps", cfg.rotate_steps);
  cfg.approach_offset = node->declare_parameter<double>("approach_offset", cfg.approach_offset);
  cfg.rim_inset = node->declare_parameter<double>("rim_inset", cfg.rim_inset);
  cfg.tcp_local_z = node->declare_parameter<double>("tcp_local_z", cfg.tcp_local_z);

  cfg.motion.eef_step = node->declare_parameter<double>("motion.eef_step", cfg.motion.eef_step);
  cfg.motion.jump_thresh = node->declare_parameter<double>("motion.jump_thresh", cfg.motion.jump_thresh);
  cfg.motion.min_fraction = node->declare_parameter<double>("motion.min_fraction", cfg.motion.min_fraction);
  cfg.motion.fast = node->declare_parameter<double>("motion.fast", cfg.motion.fast);
  cfg.motion.slow = node->declare_parameter<double>("motion.slow", cfg.motion.slow);
  cfg.motion.planning_time_s = node->declare_parameter<double>("motion.planning_time_s", cfg.motion.planning_time_s);
  cfg.motion.planning_attempts = node->declare_parameter<int>("motion.planning_attempts", cfg.motion.planning_attempts);

  cfg.gripper.group = node->declare_parameter<std::string>("gripper.group", cfg.gripper.group);
  cfg.gripper.open["joint7"] = node->declare_parameter<double>("gripper.open_joint7", cfg.gripper.open["joint7"]);
  cfg.gripper.open["joint8"] = node->declare_parameter<double>("gripper.open_joint8", cfg.gripper.open["joint8"]);
  cfg.gripper.close["joint7"] = node->declare_parameter<double>("gripper.close_joint7", cfg.gripper.close["joint7"]);
  cfg.gripper.close["joint8"] = node->declare_parameter<double>("gripper.close_joint8", cfg.gripper.close["joint8"]);

  cfg.servo.joint        = node->declare_parameter<std::string>("servo.joint",        cfg.servo.joint);
  cfg.servo.scale        = node->declare_parameter<double>     ("servo.scale",        cfg.servo.scale);
  cfg.servo.max_step_rad = node->declare_parameter<double>     ("servo.max_step_rad", cfg.servo.max_step_rad);
  cfg.servo.tol_rad      = node->declare_parameter<double>     ("servo.tol_rad",      cfg.servo.tol_rad);

  RCLCPP_INFO(
    node->get_logger(),
    "Params: mode=%s rotate_deg=%.1f start_angle_deg=%.1f approach_offset=%.3f radius=%.3f",
    mode.c_str(), cfg.rotate_deg, cfg.start_angle_deg, cfg.approach_offset, cfg.radius);

  RCLCPP_INFO(
    node->get_logger(),
    "Wheel TF: publish_static_tf=%s frame=%s tf_frame=%s center=(%.4f, %.4f, %.4f)",
    cfg.wheel.publish_static_tf ? "true" : "false",
    cfg.wheel.frame.c_str(),
    cfg.wheel.tf_frame.c_str(),
    cfg.wheel.center.x(), cfg.wheel.center.y(), cfg.wheel.center.z());

  // Executor for TF, MoveIt, ROS callbacks, etc.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // Wait until TF / MoveIt are safely available
  std::this_thread::sleep_for(2s);

  // App + Config
  PiperGrabRotate app(node, cfg);

  // Mode selection
  bool ok = false;

  if (mode == "rotate")
  {
    RCLCPP_INFO(node->get_logger(), "Mode = ROTATE");
    ok = app.run();
  }
  else if (mode == "hold")
  {
    RCLCPP_INFO(node->get_logger(), "Mode = HOLD (Ctrl+C to stop)");
    ok = app.runHold();
  }
  else if (mode == "rotate_only")
  {
    RCLCPP_INFO(node->get_logger(), "Mode = ROTATE_ONLY (no grip)");
    ok = app.runRotateOnly();
  }
  else if (mode == "ai_hold")
  {
    RCLCPP_INFO(node->get_logger(), "Mode = AI_HOLD");
    ok = app.runAIHold();
  }
  else if (mode == "servo_hold")
  {
    RCLCPP_INFO(node->get_logger(),
      "Mode = SERVO_HOLD  joint=%s  scale=%.3f",
      cfg.servo.joint.c_str(), cfg.servo.scale);
    ok = app.runServoHold();
  }
  else
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Unknown mode '%s'. Use: rotate | hold | rotate_only | ai_hold | servo_hold",
      mode.c_str());

    ok = false;
  }

  exec.cancel();

  if (spinner.joinable())
    spinner.join();

  rclcpp::shutdown();

  return ok ? 0 : 1;
}