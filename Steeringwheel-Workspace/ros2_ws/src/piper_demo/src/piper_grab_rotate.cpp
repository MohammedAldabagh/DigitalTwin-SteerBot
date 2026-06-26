#include "piper_demo/piper_grab_rotate.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

static tf2::Quaternion normalizedQuat(tf2::Quaternion q)
{
  if (q.length2() < 1e-12) return tf2::Quaternion(0,0,0,1);
  q.normalize();
  return q;
}

static geometry_msgs::msg::Quaternion rotateQuatAroundAxisWorld(
    const geometry_msgs::msg::Quaternion& q_in,
    const tf2::Vector3& axis_world,
    double angle_rad)
{
  tf2::Quaternion q0;
  tf2::fromMsg(q_in, q0);
  q0 = normalizedQuat(q0);

  tf2::Vector3 a = axis_world;
  if (a.length2() < 1e-12) a = tf2::Vector3(0, 0, 1);
  a.normalize();

  tf2::Quaternion qrot;
  qrot.setRotation(a, angle_rad);
  qrot = normalizedQuat(qrot);

  tf2::Quaternion q = qrot * q0;
  q = normalizedQuat(q);
  return tf2::toMsg(q);
}

static geometry_msgs::msg::TransformStamped makeStaticTf(
  const rclcpp::Time& stamp, const std::string& parent, const std::string& child,
  const tf2::Vector3& t, const tf2::Quaternion& q_in)
{
  geometry_msgs::msg::TransformStamped st;
  st.header.stamp = stamp;
  st.header.frame_id = parent;
  st.child_frame_id  = child;

  st.transform.translation.x = t.x();
  st.transform.translation.y = t.y();
  st.transform.translation.z = t.z();

  tf2::Quaternion q = normalizedQuat(q_in);
  st.transform.rotation = tf2::toMsg(q);
  return st;
}

static double wrapToPi(double a)
{
  while (a >  M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

static double shortestAngDist(double from, double to)
{
  return wrapToPi(to - from);
}

PiperGrabRotate::PiperGrabRotate(rclcpp::Node::SharedPtr node, Config cfg)
: node_(std::move(node)),
  logger_(node_->get_logger()),
  cfg_(std::move(cfg)),
  arm_(node_, cfg_.arm_group),
  gripper_(node_, cfg_.gripper.group)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  arm_.setPlanningTime(cfg_.motion.planning_time_s);
  arm_.setNumPlanningAttempts(cfg_.motion.planning_attempts);

  if (!cfg_.ee_link_override.empty())
    arm_.setEndEffectorLink(cfg_.ee_link_override);

  planning_frame_ = arm_.getPlanningFrame();
  ee_link_ = arm_.getEndEffectorLink();

  cfg_.wheel.q = normalizedQuat(cfg_.wheel.q);

  if (cfg_.wheel.publish_static_tf && !cfg_.wheel.tf_frame.empty())
  {
    static_tf_broadcaster_->sendTransform(
      makeStaticTf(node_->get_clock()->now(),
                   cfg_.wheel.frame,
                   cfg_.wheel.tf_frame,
                   cfg_.wheel.center,
                   cfg_.wheel.q));

    RCLCPP_INFO(logger_, "Published static TF: %s -> %s",
                cfg_.wheel.frame.c_str(), cfg_.wheel.tf_frame.c_str());
  }
  else
  {
    RCLCPP_INFO(logger_, "Static wheel TF disabled; expecting live TF for frame '%s'.",
                cfg_.wheel.tf_frame.c_str());
  }

  wheel_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/wheel_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      for (size_t i = 0; i < msg->name.size(); ++i)
      {
        if (msg->name[i] == wheel_joint_name_ && i < msg->position.size())
        {
          wheel_pos_rad_.store(msg->position[i]);
          wheel_pos_valid_.store(true);
          return;
        }
      }
    });

  RCLCPP_INFO(logger_, "Subscribed to /wheel_states, joint='%s'",
              wheel_joint_name_.c_str());

  wheel_position_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/wheel/position_from_ee", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32::SharedPtr msg)
    {
      wheel_position_deg_.store(static_cast<double>(msg->data));
      wheel_position_valid_.store(true);
    });

  wheel_position_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
    "/wheel/position_from_ee", rclcpp::QoS(10));

  RCLCPP_INFO(logger_, "Subscribed to /wheel/position");

  RCLCPP_INFO(logger_, "Planning frame: %s", planning_frame_.c_str());
  RCLCPP_INFO(logger_, "EE link: %s", ee_link_.c_str());
  RCLCPP_INFO(logger_, "Wheel TF frame: '%s'", cfg_.wheel.tf_frame.c_str());

  ai_action_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/ai/wheel_hold_action",
    rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32::SharedPtr msg)
    {
      ai_action_deg_.store(static_cast<double>(msg->data));
      ai_action_valid_.store(true);
    });

  RCLCPP_INFO(logger_, "Subscribed to /ai/wheel_hold_action");

  wheel_target_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/wheel/target_angle", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32::SharedPtr msg)
    {
      wheel_target_rad_.store(static_cast<double>(msg->data));
      wheel_target_valid_.store(true);
    });

  RCLCPP_INFO(logger_, "Subscribed to /wheel/target_angle");
}

double PiperGrabRotate::angleOnWheel(const WheelState& ws,
                                     const geometry_msgs::msg::PoseStamped& tcp) const
{
  tf2::Vector3 p(tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z);

  tf2::Vector3 v = p - ws.c;
  v = v - ws.n * v.dot(ws.n);
  if (v.length2() < 1e-12) return 0.0;

  tf2::Vector3 v_local = tf2::quatRotate(ws.q.inverse(), v);
  return std::atan2(v_local.y(), v_local.x());
}

PiperGrabRotate::WheelState PiperGrabRotate::wheelFromTf() const
{
  const auto timeout = tf2::durationFromSec(1.0);

  if (!tf_buffer_->canTransform(planning_frame_, cfg_.wheel.tf_frame, tf2::TimePointZero, timeout))
  {
    throw tf2::LookupException(
      "TF not available: " + planning_frame_ + " <- " + cfg_.wheel.tf_frame);
  }

  auto T = tf_buffer_->lookupTransform(planning_frame_, cfg_.wheel.tf_frame, tf2::TimePointZero);

  tf2::Transform tf;
  tf2::fromMsg(T.transform, tf);

  WheelState ws;
  ws.c = tf.getOrigin();
  ws.q = normalizedQuat(tf.getRotation());

  ws.n = tf2::quatRotate(ws.q, tf2::Vector3(0,0,1));
  if (ws.n.length2() < 1e-12) ws.n = tf2::Vector3(0,0,1);
  ws.n.normalize();
  return ws;
}

tf2::Vector3 PiperGrabRotate::rimPoint(const WheelState& ws, double angle_rad) const
{
  const tf2::Vector3 local(cfg_.radius * std::cos(angle_rad),
                           cfg_.radius * std::sin(angle_rad),
                           0.0);
  return tf2::quatRotate(ws.q, local) + ws.c;
}

void PiperGrabRotate::rimFrame(const WheelState& ws, const tf2::Vector3& contact,
                               tf2::Vector3& r_out, tf2::Vector3& t_out) const
{
  tf2::Vector3 r = contact - ws.c;

  r = r - ws.n * r.dot(ws.n);
  if (r.length2() < 1e-12) r = tf2::Vector3(1, 0, 0);
  r.normalize();

  tf2::Vector3 t = ws.n.cross(r);
  if (t.length2() < 1e-12) t = tf2::Vector3(0, 1, 0);
  t.normalize();

  r_out = r;
  t_out = t;
}

geometry_msgs::msg::Quaternion PiperGrabRotate::makeGraspOrientation(const WheelState& ws, const tf2::Vector3& contact) const
{
  tf2::Vector3 r, t;
  rimFrame(ws, contact, r, t);

  tf2::Vector3 z_axis = (-ws.n);
  z_axis.normalize();

  tf2::Vector3 x_axis = t;
  x_axis.normalize();

  tf2::Vector3 y_axis = z_axis.cross(x_axis);
  if (y_axis.length2() < 1e-12) y_axis = tf2::Vector3(0,1,0);
  y_axis.normalize();

  x_axis = y_axis.cross(z_axis);
  x_axis.normalize();

  tf2::Matrix3x3 R(
    x_axis.x(), y_axis.x(), z_axis.x(),
    x_axis.y(), y_axis.y(), z_axis.y(),
    x_axis.z(), y_axis.z(), z_axis.z()
  );

  tf2::Quaternion q;
  R.getRotation(q);
  return tf2::toMsg(normalizedQuat(q));
}

void PiperGrabRotate::applyTcpLocalZ(geometry_msgs::msg::PoseStamped& p) const
{
  const double dz = cfg_.tcp_local_z;
  if (std::abs(dz) < 1e-9) return;

  tf2::Quaternion q;
  tf2::fromMsg(p.pose.orientation, q);
  q = normalizedQuat(q);

  const tf2::Vector3 z_axis = tf2::quatRotate(q, tf2::Vector3(0, 0, 1));

  p.pose.position.x += dz * z_axis.x();
  p.pose.position.y += dz * z_axis.y();
  p.pose.position.z += dz * z_axis.z();
}

geometry_msgs::msg::PoseStamped PiperGrabRotate::makeApproachPose(
  const WheelState& ws,
  const geometry_msgs::msg::PoseStamped& seed,
  double angle_rad) const
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = planning_frame_;
  out.pose = seed.pose;

  const tf2::Vector3 contact = rimPoint(ws, angle_rad);

  out.pose.position.x = contact.x() + cfg_.approach_offset * ws.n.x();
  out.pose.position.y = contact.y() + cfg_.approach_offset * ws.n.y();
  out.pose.position.z = contact.z() + cfg_.approach_offset * ws.n.z();

  out.pose.orientation = makeGraspOrientation(ws, contact);
  applyTcpLocalZ(out);
  return out;
}

geometry_msgs::msg::PoseStamped PiperGrabRotate::makeGraspPose(
  const WheelState& ws,
  const geometry_msgs::msg::PoseStamped& approach) const
{
  geometry_msgs::msg::PoseStamped out = approach;

  out.pose.position.x -= cfg_.approach_offset * ws.n.x();
  out.pose.position.y -= cfg_.approach_offset * ws.n.y();
  out.pose.position.z -= cfg_.approach_offset * ws.n.z();

  out.pose.position.x -= cfg_.rim_inset * ws.n.x();
  out.pose.position.y -= cfg_.rim_inset * ws.n.y();
  out.pose.position.z -= cfg_.rim_inset * ws.n.z();

  tf2::Vector3 contact(out.pose.position.x, out.pose.position.y, out.pose.position.z);
  out.pose.orientation = makeGraspOrientation(ws, contact);
  applyTcpLocalZ(out);
  return out;
}

void PiperGrabRotate::setSpeed(double scale)
{
  arm_.setMaxVelocityScalingFactor(scale);
  arm_.setMaxAccelerationScalingFactor(scale);
}

bool PiperGrabRotate::moveToPose(const geometry_msgs::msg::PoseStamped& pose)
{
  arm_.setStartStateToCurrentState();
  arm_.clearPoseTargets();
  arm_.setPoseTarget(pose, ee_link_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(logger_, "Planning failed.");
    return false;
  }

  if (arm_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(logger_, "Execution failed.");
    return false;
  }
  return true;
}

bool PiperGrabRotate::moveToJoints(const std::map<std::string, double>& joints)
{
  arm_.setStartStateToCurrentState();
  arm_.clearPoseTargets();
  arm_.setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(logger_, "Joint planning failed.");
    return false;
  }

  if (arm_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(logger_, "Joint execution failed.");
    return false;
  }
  return true;
}

void PiperGrabRotate::moveGripper(const std::map<std::string, double>& target)
{
  gripper_.setStartStateToCurrentState();
  gripper_.setJointValueTarget(target);
  auto code = gripper_.move();

  if (code != moveit::core::MoveItErrorCode::SUCCESS)
    RCLCPP_WARN(logger_, "Gripper move failed.");
  else
    RCLCPP_INFO(logger_, "Gripper move ok.");
}

bool PiperGrabRotate::rotateArcCartesian(
  const WheelState& ws,
  const geometry_msgs::msg::PoseStamped& grasp_pose,
  double start_angle_rad)
{
  const int steps = std::max(24, cfg_.rotate_steps);
  const double total_rad = cfg_.rotate_deg * M_PI / 180.0;

  geometry_msgs::msg::Pose p_start = arm_.getCurrentPose(ee_link_).pose;
  const geometry_msgs::msg::Pose p_ref = grasp_pose.pose;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(steps + 1);
  waypoints.push_back(p_start);

  for (int i = 1; i <= steps; ++i)
  {
    const double s = double(i) / double(steps);
    const double a = start_angle_rad + total_rad * s;

    geometry_msgs::msg::Pose p = p_start;
    const tf2::Vector3 rim = rimPoint(ws, a);

    p.position.x = rim.x() - cfg_.rim_inset * ws.n.x();
    p.position.y = rim.y() - cfg_.rim_inset * ws.n.y();
    p.position.z = rim.z() - cfg_.rim_inset * ws.n.z();

    p.orientation = rotateQuatAroundAxisWorld(p_ref.orientation, ws.n, total_rad * s);

    waypoints.push_back(p);
  }

  moveit_msgs::msg::RobotTrajectory traj;
  arm_.setStartStateToCurrentState();

  const double fraction = arm_.computeCartesianPath(
    waypoints, cfg_.motion.eef_step, cfg_.motion.jump_thresh, traj, true);

  if (fraction >= cfg_.motion.min_fraction)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = traj;
    return arm_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  RCLCPP_WARN(logger_,
              "Arc cartesian fraction too small: %.3f (min=%.3f). Falling back to segmented pose planning.",
              fraction, cfg_.motion.min_fraction);

  const int fallback_steps = std::max(8, std::min(steps, 24));
  for (int i = 1; i <= fallback_steps; ++i)
  {
    const double s = static_cast<double>(i) / static_cast<double>(fallback_steps);
    const double a = start_angle_rad + total_rad * s;
    const tf2::Vector3 rim = rimPoint(ws, a);

    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = planning_frame_;
    target.pose.position.x = rim.x() - cfg_.rim_inset * ws.n.x();
    target.pose.position.y = rim.y() - cfg_.rim_inset * ws.n.y();
    target.pose.position.z = rim.z() - cfg_.rim_inset * ws.n.z();
    target.pose.orientation = rotateQuatAroundAxisWorld(p_ref.orientation, ws.n, total_rad * s);

    if (!cartesianTo(target.pose, "ArcFallbackSeg", cfg_.motion.eef_step, cfg_.motion.jump_thresh, 0.2)
        && !moveToPose(target))
    {
      RCLCPP_WARN(logger_, "Fallback arc segment %d/%d failed.", i, fallback_steps);
      return false;
    }
  }

  RCLCPP_INFO(logger_, "Fallback segmented arc finished (%d segments).", fallback_steps);
  return true;
}

bool PiperGrabRotate::execTraj(const moveit_msgs::msg::RobotTrajectory& traj, const char* tag)
{
  if (!tag) tag = "Traj";

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj;

  if (arm_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(logger_, "%s execute failed.", tag);
    return false;
  }
  return true;
}

bool PiperGrabRotate::cartesianTo(
  const geometry_msgs::msg::Pose& target,
  const char* tag,
  double eef_step,
  double jump_thresh,
  double min_fraction)
{
  if (!tag) tag = "Cartesian";

  if (eef_step < 0.0)      eef_step      = cfg_.motion.eef_step;
  if (jump_thresh < 0.0)   jump_thresh   = cfg_.motion.jump_thresh;
  if (min_fraction < 0.0)  min_fraction  = cfg_.motion.min_fraction;

  const auto start = arm_.getCurrentPose(ee_link_).pose;

  const double dx = target.position.x - start.position.x;
  const double dy = target.position.y - start.position.y;
  const double dz = target.position.z - start.position.z;
  const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

  std::vector<geometry_msgs::msg::Pose> wps;
  wps.reserve(2);
  wps.push_back(start);
  wps.push_back(target);

  moveit_msgs::msg::RobotTrajectory traj;
  arm_.setStartStateToCurrentState();

  const double frac = arm_.computeCartesianPath(wps, eef_step, jump_thresh, traj, true);

  RCLCPP_INFO(logger_, "%s cartesian fraction: %.3f (dist=%.3f, step=%.4f, jump=%.3f, min=%.3f)",
              tag, frac, dist, eef_step, jump_thresh, min_fraction);

  if (frac < min_fraction)
  {
    RCLCPP_WARN(logger_, "%s cartesian path too small (%.3f < %.3f).",
                tag, frac, min_fraction);
    return false;
  }

  return execTraj(traj, tag);
}

bool PiperGrabRotate::nudgeJoint(
  const std::string& joint_name,
  double delta_rad,
  double speed_scale,
  bool clamp)
{
  const auto names = arm_.getJointNames();
  auto vals = arm_.getCurrentJointValues();

  int idx = -1;
  for (size_t i = 0; i < names.size(); ++i)
  {
    if (names[i] == joint_name)
    {
      idx = static_cast<int>(i);
      break;
    }
  }

  if (idx < 0 || idx >= static_cast<int>(vals.size()))
  {
    RCLCPP_WARN(logger_, "nudgeJoint: joint '%s' not found in group '%s'",
                joint_name.c_str(), cfg_.arm_group.c_str());
    return false;
  }

  double target = vals[idx] + delta_rad;

  if (clamp)
  {
    const auto state = arm_.getCurrentState();
    if (state)
    {
      const auto* jm = state->getRobotModel()->getJointModel(joint_name);
      if (jm && !jm->getVariableBounds().empty())
      {
        const auto& b = jm->getVariableBounds()[0];
        if (b.position_bounded_)
        {
          target = std::min(std::max(target, b.min_position_), b.max_position_);
        }
      }
    }
  }

  vals[idx] = target;

  setSpeed(speed_scale);
  arm_.setStartStateToCurrentState();
  arm_.clearPoseTargets();
  arm_.setJointValueTarget(vals);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(logger_, "nudgeJoint: planning failed for %s (delta=%.3f rad)",
                joint_name.c_str(), delta_rad);
    return false;
  }

  if (arm_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(logger_, "nudgeJoint: execution failed for %s", joint_name.c_str());
    return false;
  }

  RCLCPP_INFO(logger_, "nudgeJoint: %s += %.2f deg",
              joint_name.c_str(), delta_rad * 180.0 / M_PI);
  return true;
}

bool PiperGrabRotate::holdWheelAngle(
  const WheelState& ws,
  const geometry_msgs::msg::PoseStamped& grasp_ref,
  double wheel_grasp_rad,
  double a_grasp_rad,
  double wheel_target_rad)
{
  const double tol_rad = 0.5 * M_PI / 180.0;
  const double rate_hz = 20.0;
  const double s = +1.0;

  rclcpp::Rate rate(rate_hz);
  setSpeed(cfg_.motion.slow);

  RCLCPP_INFO(logger_,
    "Hold: wheel_grasp=%.3f, a_grasp=%.3f, target=%.3f",
    wheel_grasp_rad, a_grasp_rad, wheel_target_rad);

  while (rclcpp::ok())
  {
    if (!wheel_pos_valid_.load())
    {
      rate.sleep();
      continue;
    }

    const double wheel_now = wheel_pos_rad_.load();
    const double err = shortestAngDist(wheel_now, wheel_target_rad);

    if (std::abs(err) > tol_rad)
    {
      const double a_cmd = a_grasp_rad + s * err;

      const tf2::Vector3 rim = rimPoint(ws, a_cmd);

      geometry_msgs::msg::Pose target = arm_.getCurrentPose(ee_link_).pose;

      target.position.x = rim.x() - cfg_.rim_inset * ws.n.x();
      target.position.y = rim.y() - cfg_.rim_inset * ws.n.y();
      target.position.z = rim.z() - cfg_.rim_inset * ws.n.z();

      target.orientation =
        rotateQuatAroundAxisWorld(grasp_ref.pose.orientation, ws.n, (a_cmd - a_grasp_rad));

      (void)cartesianTo(target, "HoldJoint",
                        cfg_.motion.eef_step, cfg_.motion.jump_thresh, 0.2);

      RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 500,
        "Hold: now=%.3f tgt=%.3f err=%.3f a_cmd=%.3f",
        wheel_now, wheel_target_rad, err, a_cmd);
    }

    rate.sleep();
  }

  return true;
}

bool PiperGrabRotate::aiHoldWheelAngle(
  const WheelState& ws,
  const geometry_msgs::msg::PoseStamped& grasp_ref,
  double a_grasp_rad,
  double wheel_target_deg)
{
  const double rate_hz = 10.0;
  const double max_action_deg = 5.0;
  const double max_total_cmd_deg = 250.0;

  double a_cmd = a_grasp_rad;

  rclcpp::Rate rate(rate_hz);
  setSpeed(cfg_.motion.slow);

  RCLCPP_INFO(logger_, "AI_HOLD: using external Python agent actions");
  RCLCPP_INFO(logger_, "AI_HOLD: target wheel angle = %.3f deg", wheel_target_deg);


  while (rclcpp::ok())
  {
    if (!wheel_position_valid_.load())
    {
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
                           "AI_HOLD: waiting for /wheel/position ...");
      rate.sleep();
      continue;
    }

    if (!ai_action_valid_.load())
    {
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
                           "AI_HOLD: waiting for /ai/wheel_hold_action ...");
      rate.sleep();
      continue;
    }

    const double wheel_now_deg = wheel_position_deg_.load();
    const double err_deg = wheel_target_deg - wheel_now_deg;

    double agent_action_deg = ai_action_deg_.load();
    agent_action_deg = std::clamp(agent_action_deg, -max_action_deg, max_action_deg);

    if (std::abs(agent_action_deg) <= 0.001)
    {
      a_cmd = a_grasp_rad;
    }
    if (std::abs(agent_action_deg) > 0.001)
    {
      const double agent_action_rad = agent_action_deg * M_PI / 180.0;

      a_cmd = a_cmd + agent_action_rad;

      const double max_total_cmd_rad = max_total_cmd_deg * M_PI / 180.0;
      a_cmd = std::clamp(
        a_cmd,
        a_grasp_rad - max_total_cmd_rad,
        a_grasp_rad + max_total_cmd_rad
      );

      const tf2::Vector3 rim = rimPoint(ws, a_cmd);

      geometry_msgs::msg::Pose target = arm_.getCurrentPose(ee_link_).pose;

      target.position.x = rim.x() - cfg_.rim_inset * ws.n.x();
      target.position.y = rim.y() - cfg_.rim_inset * ws.n.y();
      target.position.z = rim.z() - cfg_.rim_inset * ws.n.z();

      target.orientation =
        rotateQuatAroundAxisWorld(
          grasp_ref.pose.orientation,
          ws.n,
          a_cmd - a_grasp_rad
        );

      cartesianTo(target, "AIHoldStep",
                  cfg_.motion.eef_step,
                  cfg_.motion.jump_thresh,
                  0.2);
      
      auto ee_now = arm_.getCurrentPose(ee_link_);
      double computed_angle_rad = angleOnWheel(ws, ee_now);
      std_msgs::msg::Float32 fb_msg;
      fb_msg.data = static_cast<float>(computed_angle_rad * 180.0 / M_PI);
      wheel_position_pub_->publish(fb_msg);

     
    }

    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 500,
      "AI_HOLD: now=%.3f target=%.3f err=%.3f agent_action=%.3f a_cmd=%.3f",
      wheel_now_deg,
      wheel_target_deg,
      err_deg,
      agent_action_deg,
      (a_cmd - a_grasp_rad) * 180.0 / M_PI);

    rate.sleep();
  }

  return true;
}

bool PiperGrabRotate::runHold()
{
  WheelState ws;
  try
  {
    ws = wheelFromTf();
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_ERROR(logger_, "wheelFromTf failed: %s", e.what());
    return false;
  }

  const double a0 = cfg_.start_angle_deg * M_PI / 180.0;
  auto seed = arm_.getCurrentPose(ee_link_);

  auto approach = makeApproachPose(ws, seed, a0);
  auto grasp = makeGraspPose(ws, approach);

  setSpeed(cfg_.motion.fast);
  RCLCPP_INFO(logger_, "A) Gripper open");
  moveGripper(cfg_.gripper.open);
  std::this_thread::sleep_for(400ms);

  RCLCPP_INFO(logger_, "1) Move to approach");
  if (!moveToPose(approach))
    return false;

  RCLCPP_INFO(logger_, "2) Move to grasp (slow, linear)");
  setSpeed(cfg_.motion.slow);
  if (!cartesianTo(grasp.pose, "Grasp"))
    return false;

  RCLCPP_INFO(logger_, "3) Close gripper");
  moveGripper(cfg_.gripper.close);
  std::this_thread::sleep_for(600ms);

  {
    rclcpp::Time t0 = node_->get_clock()->now();
    rclcpp::Rate r(200.0);
    while (rclcpp::ok() && !wheel_pos_valid_.load())
    {
      if ((node_->get_clock()->now() - t0).seconds() > 2.0)
      {
        RCLCPP_WARN(logger_, "No /wheel_states received within 2s. Using target=0.0 rad.");
        break;
      }
      r.sleep();
    }
  }

  auto grasp_ref = arm_.getCurrentPose(ee_link_);

  const double wheel_grasp = wheel_pos_valid_.load() ? wheel_pos_rad_.load() : 0.0;
  const double wheel_target = wheel_grasp;
  const double a_grasp = a0;

  RCLCPP_INFO(logger_, "Hold target wheel joint = %.3f rad (%.1f deg)",
              wheel_target, wheel_target * 180.0 / M_PI);

  return holdWheelAngle(ws, grasp_ref, wheel_grasp, a_grasp, wheel_target);
}

bool PiperGrabRotate::runAIHold()
{
  WheelState ws;
  try
  {
    ws = wheelFromTf();
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_ERROR(logger_, "wheelFromTf failed: %s", e.what());
    return false;
  }

  const double a0 = cfg_.start_angle_deg * M_PI / 180.0;
  auto seed = arm_.getCurrentPose(ee_link_);

  auto approach = makeApproachPose(ws, seed, a0);
  auto grasp = makeGraspPose(ws, approach);

  setSpeed(cfg_.motion.fast);
  RCLCPP_INFO(logger_, "AI_HOLD A) Gripper open");
  moveGripper(cfg_.gripper.open);
  std::this_thread::sleep_for(400ms);

  RCLCPP_INFO(logger_, "AI_HOLD 1) Move to approach");
  if (!moveToPose(approach))
    return false;

  RCLCPP_INFO(logger_, "AI_HOLD 2) Move to grasp");
  setSpeed(cfg_.motion.slow);
  if (!cartesianTo(grasp.pose, "AIGrasp"))
    return false;

  RCLCPP_INFO(logger_, "AI_HOLD 3) Close gripper");
  moveGripper(cfg_.gripper.close);
  std::this_thread::sleep_for(1000ms);

  RCLCPP_INFO(logger_, "AI_HOLD 4) Publishing initial wheel position");

  {
    auto ee_seed = arm_.getCurrentPose(ee_link_);
    double seed_angle_rad = angleOnWheel(ws, ee_seed);
    std_msgs::msg::Float32 seed_msg;
    seed_msg.data = static_cast<float>(seed_angle_rad * 180.0 / M_PI);
    wheel_position_pub_->publish(seed_msg);
    wheel_position_deg_.store(seed_msg.data);
    wheel_position_valid_.store(true);
  }
  

  const double wheel_target_deg = 25.0;

  RCLCPP_INFO(logger_,
              "AI_HOLD target = %.3f deg (holding at center)",
              wheel_target_deg);

  RCLCPP_INFO(logger_, "AI_HOLD 5) Waiting for /ai/wheel_hold_action");

  {
    rclcpp::Time t0 = node_->get_clock()->now();
    rclcpp::Rate r(100.0);

    while (rclcpp::ok() && !ai_action_valid_.load())
    {
      if ((node_->get_clock()->now() - t0).seconds() > 30.0)
      {
        RCLCPP_ERROR(logger_,
          "AI_HOLD: No /ai/wheel_hold_action received within 30s.");
        return false;
      }
      r.sleep();
    }
  }

  auto grasp_ref = arm_.getCurrentPose(ee_link_);
  const double a_grasp = a0;

  RCLCPP_INFO(logger_, "AI_HOLD is gripping wheel and following Python agent actions.");

  return aiHoldWheelAngle(ws, grasp_ref, a_grasp, wheel_target_deg);
}

bool PiperGrabRotate::run()
{
  WheelState ws;
  try
  {
    ws = wheelFromTf();
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_ERROR(logger_, "wheelFromTf failed: %s", e.what());
    return false;
  }

  const double a0 = cfg_.start_angle_deg * M_PI / 180.0;
  auto seed = arm_.getCurrentPose(ee_link_);

  RCLCPP_INFO(logger_, "Wheel(planning): c=(%.4f %.4f %.4f) n=(%.4f %.4f %.4f)",
              ws.c.x(), ws.c.y(), ws.c.z(),
              ws.n.x(), ws.n.y(), ws.n.z());

  auto approach = makeApproachPose(ws, seed, a0);
  auto grasp = makeGraspPose(ws, approach);

  RCLCPP_INFO(logger_, "Approach target: x=%.3f y=%.3f z=%.3f q=(%.4f %.4f %.4f %.4f) (frame=%s)",
              approach.pose.position.x, approach.pose.position.y, approach.pose.position.z,
              approach.pose.orientation.x, approach.pose.orientation.y, approach.pose.orientation.z, approach.pose.orientation.w,
              approach.header.frame_id.c_str());

  RCLCPP_INFO(logger_, "Grasp target:    x=%.3f y=%.3f z=%.3f q=(%.4f %.4f %.4f %.4f) (frame=%s)",
              grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z,
              grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z, grasp.pose.orientation.w,
              grasp.header.frame_id.c_str());

  setSpeed(cfg_.motion.fast);
  RCLCPP_INFO(logger_, "A) Gripper open");
  moveGripper(cfg_.gripper.open);
  std::this_thread::sleep_for(400ms);

  RCLCPP_INFO(logger_, "1) Move to approach");
  if (!moveToPose(approach))
    return false;

  RCLCPP_INFO(logger_, "2) Move to grasp (slow, linear)");
  setSpeed(cfg_.motion.slow);

  if (!cartesianTo(grasp.pose, "Grasp"))
    return false;

  auto grasp_now = arm_.getCurrentPose(ee_link_);
  const double a_start = angleOnWheel(ws, grasp_now);

  RCLCPP_INFO(logger_, "a_start (from grasp_now) = %.3f rad (%.1f deg)",
              a_start, a_start * 180.0 / M_PI);

  RCLCPP_INFO(logger_, "3) Close gripper");
  moveGripper(cfg_.gripper.close);
  std::this_thread::sleep_for(600ms);

  RCLCPP_INFO(logger_, "4) Rotate along wheel plane");
  setSpeed(cfg_.motion.slow);

  if (!rotateArcCartesian(ws, grasp_now, a_start))
    return false;

  setSpeed(cfg_.motion.fast);
  RCLCPP_INFO(logger_, "5) Open gripper (release)");
  moveGripper(cfg_.gripper.open);
  std::this_thread::sleep_for(400ms);

  if (!nudgeJoint("joint4", -2.0 * M_PI / 180.0, cfg_.motion.slow))
    return false;

  RCLCPP_INFO(logger_, "6) Retract along TCP local -Z (linear)");
  auto retract = arm_.getCurrentPose(ee_link_);

  {
    tf2::Quaternion q;
    tf2::fromMsg(retract.pose.orientation, q);
    q = normalizedQuat(q);
    tf2::Vector3 z = tf2::quatRotate(q, tf2::Vector3(0, 0, 1));
    retract.pose.position.x -= 0.10 * z.x();
    retract.pose.position.y -= 0.10 * z.y();
    retract.pose.position.z -= 0.10 * z.z();
  }

  if (!cartesianTo(retract.pose, "Retract"))
    return false;

  RCLCPP_INFO(logger_, "7) Return to approach");
  if (!moveToPose(approach))
    return false;

  RCLCPP_INFO(logger_, "Grab+Rotate finished.");
  return true;
}

bool PiperGrabRotate::runRotateOnly()
{
  WheelState ws;
  try
  {
    ws = wheelFromTf();
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_ERROR(logger_, "wheelFromTf failed: %s", e.what());
    return false;
  }

  const double a0 = cfg_.start_angle_deg * M_PI / 180.0;
  auto seed = arm_.getCurrentPose(ee_link_);

  auto approach = makeApproachPose(ws, seed, a0);
  auto grasp = makeGraspPose(ws, approach);

  setSpeed(cfg_.motion.fast);
  RCLCPP_INFO(logger_, "1) Move to approach");
  if (!moveToPose(approach))
    return false;

  RCLCPP_INFO(logger_, "2) Move to contact point (slow, no grip)");
  setSpeed(cfg_.motion.slow);
  if (!cartesianTo(grasp.pose, "Contact"))
    return false;

  auto contact_now = arm_.getCurrentPose(ee_link_);
  const double a_start = angleOnWheel(ws, contact_now);

  RCLCPP_INFO(logger_, "3) Rotate along wheel arc (open gripper)");
  setSpeed(cfg_.motion.slow);
  if (!rotateArcCartesian(ws, contact_now, a_start))
    return false;

  setSpeed(cfg_.motion.fast);
  RCLCPP_INFO(logger_, "4) Retract");
  auto retract = arm_.getCurrentPose(ee_link_);

  {
    tf2::Quaternion q;
    tf2::fromMsg(retract.pose.orientation, q);
    q = normalizedQuat(q);
    tf2::Vector3 z = tf2::quatRotate(q, tf2::Vector3(0, 0, 1));
    retract.pose.position.x -= 0.10 * z.x();
    retract.pose.position.y -= 0.10 * z.y();
    retract.pose.position.z -= 0.10 * z.z();
  }

  if (!cartesianTo(retract.pose, "Retract"))
    return false;

  RCLCPP_INFO(logger_, "5) Return to approach");
  if (!moveToPose(approach))
    return false;

  RCLCPP_INFO(logger_, "RotateOnly finished.");
  return true;
}

bool PiperGrabRotate::runServoHold()
{
  WheelState ws;
  try
  {
    ws = wheelFromTf();
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_ERROR(logger_, "wheelFromTf failed: %s", e.what());
    return false;
  }

  const double a0 = cfg_.start_angle_deg * M_PI / 180.0;
  auto seed = arm_.getCurrentPose(ee_link_);

  auto approach = makeApproachPose(ws, seed, a0);
  auto grasp    = makeGraspPose(ws, approach);

  setSpeed(cfg_.motion.fast);
  RCLCPP_INFO(logger_, "SERVO_HOLD A) Gripper open");
  moveGripper(cfg_.gripper.open);
  std::this_thread::sleep_for(400ms);

  RCLCPP_INFO(logger_, "SERVO_HOLD 1) Move to approach");
  if (!moveToPose(approach))
    return false;

  RCLCPP_INFO(logger_, "SERVO_HOLD 2) Move to grasp (slow, linear)");
  setSpeed(cfg_.motion.slow);
  if (!cartesianTo(grasp.pose, "ServoGrasp"))
    return false;

  RCLCPP_INFO(logger_, "SERVO_HOLD 3) Close gripper");
  moveGripper(cfg_.gripper.close);
  std::this_thread::sleep_for(600ms);

  const auto joint_names = arm_.getJointNames();
  int servo_idx = -1;
  for (int i = 0; i < static_cast<int>(joint_names.size()); ++i)
  {
    if (joint_names[i] == cfg_.servo.joint)
    {
      servo_idx = i;
      break;
    }
  }

  if (servo_idx < 0)
  {
    RCLCPP_ERROR(logger_,
      "SERVO_HOLD: joint '%s' not found in arm group '%s'.",
      cfg_.servo.joint.c_str(), cfg_.arm_group.c_str());
    return false;
  }

  RCLCPP_INFO(logger_, "SERVO_HOLD 4) Waiting for /wheel/target_angle ...");
  {
    rclcpp::Time t0 = node_->get_clock()->now();
    rclcpp::Rate r(100.0);
    while (rclcpp::ok() && !wheel_target_valid_.load())
    {
      if ((node_->get_clock()->now() - t0).seconds() > 30.0)
      {
        RCLCPP_ERROR(logger_, "SERVO_HOLD: no /wheel/target_angle received within 30s.");
        return false;
      }
      r.sleep();
    }
  }

  const auto grasp_vals = arm_.getCurrentJointValues();
  const double servo_zero_rad = grasp_vals[servo_idx];

  RCLCPP_INFO(logger_,
    "SERVO_HOLD 5) Tracking — joint[%d]='%s'  zero=%.3f rad (%.1f deg)  scale=%.3f",
    servo_idx, cfg_.servo.joint.c_str(),
    servo_zero_rad, servo_zero_rad * 180.0 / M_PI,
    cfg_.servo.scale);

  return servoHoldLoop(servo_idx, servo_zero_rad);
}

bool PiperGrabRotate::servoHoldLoop(int servo_idx, double servo_zero_rad)
{
  arm_.setPlanningTime(2.0);
  arm_.setNumPlanningAttempts(3);
  setSpeed(cfg_.motion.slow);

  rclcpp::Rate rate(20.0);

  while (rclcpp::ok())
  {
    if (!wheel_target_valid_.load())
    {
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
                           "SERVO_HOLD: waiting for /wheel/target_angle ...");
      rate.sleep();
      continue;
    }

    const double wheel_target  = wheel_target_rad_.load();
    const double servo_target  = servo_zero_rad + cfg_.servo.scale * wheel_target;

    auto vals = arm_.getCurrentJointValues();
    const double servo_now = vals[servo_idx];
    const double err       = servo_target - servo_now;

    if (std::abs(err) > cfg_.servo.tol_rad)
    {
      const double delta  = std::clamp(err, -cfg_.servo.max_step_rad, cfg_.servo.max_step_rad);
      vals[servo_idx]     = servo_now + delta;

      arm_.setStartStateToCurrentState();
      arm_.setJointValueTarget(vals);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        arm_.execute(plan);
      else
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
                             "SERVO_HOLD: planning failed for servo step");
    }

    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 500,
      "SERVO_HOLD: wheel_tgt=%.1f°  servo_tgt=%.1f°  servo_now=%.1f°  err=%.2f°",
      wheel_target  * 180.0 / M_PI,
      servo_target  * 180.0 / M_PI,
      servo_now     * 180.0 / M_PI,
      err           * 180.0 / M_PI);

    rate.sleep();
  }

  return true;
}