#include <cctype>
#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <libmotioncapture/motioncapture.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
std::string sanitize_topic_token(const std::string & name)
{
  std::string token;
  token.reserve(name.size());

  for (const auto ch : name) {
    const auto uch = static_cast<unsigned char>(ch);
    if (std::isalnum(uch) || ch == '_') {
      token.push_back(static_cast<char>(ch));
    } else {
      token.push_back('_');
    }
  }

  while (!token.empty() && token.front() == '_') {
    token.erase(token.begin());
  }
  while (!token.empty() && token.back() == '_') {
    token.pop_back();
  }

  if (token.empty()) {
    token = "rigid_body";
  }

  return token;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("mocap_pose_node");

  const auto mocap_type = node->declare_parameter<std::string>("mocap_type", "nokov");
  const auto hostname = node->declare_parameter<std::string>("hostname", "localhost");
  const auto frame_id = node->declare_parameter<std::string>("frame_id", "world");

  std::map<std::string, std::string> cfg;
  cfg["hostname"] = hostname;

  std::unique_ptr<libmotioncapture::MotionCapture> mocap;
  try {
    mocap.reset(libmotioncapture::MotionCapture::connect(mocap_type, cfg));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      node->get_logger(), "Failed to connect motion capture type '%s' at '%s': %s",
      mocap_type.c_str(), hostname.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  if (!mocap->supportsRigidBodyTracking()) {
    RCLCPP_FATAL(
      node->get_logger(), "Motion capture type '%s' does not support rigid bodies",
      mocap_type.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(), "Publishing mocap poses from type '%s' at '%s' with frame_id '%s'",
    mocap_type.c_str(), hostname.c_str(), frame_id.c_str());

  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers;

  while (rclcpp::ok()) {
    mocap->waitForNextFrame();

    const auto stamp = node->now();
    const auto & rigid_bodies = mocap->rigidBodies();

    for (const auto & item : rigid_bodies) {
      const auto & rigid_body = item.second;
      const auto topic_name = "/mocap/" + sanitize_topic_token(rigid_body.name()) + "/pose";

      auto publisher_it = publishers.find(topic_name);
      if (publisher_it == publishers.end()) {
        publisher_it = publishers
          .emplace(
          topic_name,
          node->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic_name, rclcpp::SensorDataQoS()))
          .first;
        RCLCPP_INFO(
          node->get_logger(), "Publishing rigid body '%s' on '%s'",
          rigid_body.name().c_str(), topic_name.c_str());
      }

      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id;

      const auto & position = rigid_body.position();
      msg.pose.position.x = position.x();
      msg.pose.position.y = position.y();
      msg.pose.position.z = position.z();

      const auto & rotation = rigid_body.rotation();
      msg.pose.orientation.w = rotation.w();
      msg.pose.orientation.x = rotation.x();
      msg.pose.orientation.y = rotation.y();
      msg.pose.orientation.z = rotation.z();

      publisher_it->second->publish(msg);
    }
  }

  rclcpp::shutdown();
  return 0;
}
