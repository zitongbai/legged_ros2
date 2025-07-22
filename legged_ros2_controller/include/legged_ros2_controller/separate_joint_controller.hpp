/**
 * @file separate_joint_controller.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "legged_ros2_controller/legged_ros2_controller.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace legged
{

class SeparateJointController : public LeggedController
{
public:

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  using JointStateMsgSharedPtr = std::shared_ptr<sensor_msgs::msg::JointState>;
  realtime_tools::RealtimeBuffer<JointStateMsgSharedPtr> joint_cmd_buffer_;
  JointStateMsgSharedPtr joint_cmd_msg_;
  
  std::vector<double> kp_;
  std::vector<double> kd_;

};


} // namespace legged

