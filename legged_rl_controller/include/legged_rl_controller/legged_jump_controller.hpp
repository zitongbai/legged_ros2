/**
 * @file legged_jump_controller.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "legged_rl_controller/legged_rl_controller.hpp"
#include "legged_rl_controller/legged_jump_articulation.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace legged{

class LeggedJumpController : public LeggedRLController{

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  controller_interface::CallbackReturn configure_parameters_() override;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr jump_cmd_sub_;
  using BoolMsgSharedPtr = std::shared_ptr<std_msgs::msg::Bool>;
  using JumpCmdBuffer = realtime_tools::RealtimeBuffer<BoolMsgSharedPtr>;
  std::shared_ptr<JumpCmdBuffer> jump_cmd_buffer_;
  BoolMsgSharedPtr jump_cmd_msg_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr jump_dist_sub_;
  using Float32MsgSharedPtr = std::shared_ptr<std_msgs::msg::Float32>;
  using JumpDistBuffer = realtime_tools::RealtimeBuffer<Float32MsgSharedPtr>;
  std::shared_ptr<JumpDistBuffer> jump_dist_buffer_;
  Float32MsgSharedPtr jump_dist_msg_;

};

}

