#include <chrono>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class PiperPositionSensorNode : public rclcpp::Node
{
public:
  PiperPositionSensorNode()
  : Node("piper_position_sensor")
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "world");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "link6");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/piper/ee_pose");
    const double rate_hz = this->declare_parameter<double>("rate_hz", 30.0);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, 20);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PiperPositionSensorNode::publishPose, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Position sensor started: %s -> %s, topic=%s, rate=%.1f Hz",
      parent_frame_.c_str(), child_frame_.c_str(), topic_name_.c_str(), rate_hz);
  }

private:
  void publishPose()
  {
    try
    {
      const auto tf = tf_buffer_.lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = tf.header;
      pose_msg.pose.position.x = tf.transform.translation.x;
      pose_msg.pose.position.y = tf.transform.translation.y;
      pose_msg.pose.position.z = tf.transform.translation.z;
      pose_msg.pose.orientation = tf.transform.rotation;

      pose_pub_->publish(pose_msg);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "TF lookup failed (%s -> %s): %s",
        parent_frame_.c_str(),
        child_frame_.c_str(),
        ex.what());
    }
  }

  std::string parent_frame_;
  std::string child_frame_;
  std::string topic_name_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PiperPositionSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
