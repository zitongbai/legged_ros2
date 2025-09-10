/**
 * @file legged_rl_controller.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "legged_ros2_controller/legged_ros2_controller.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "yaml-cpp/yaml.h"
#include <Eigen/Geometry>
#include <deque>

#include "legged_rl_controller/legged_rl_controller_parameters.hpp"
#include "legged_rl_controller/isaaclab/envs/manager_based_rl_env.h"
#include "legged_rl_controller/legged_articulation.hpp"
#include "legged_rl_controller/isaaclab/assets/articulation/articulation.h"

namespace legged
{

class LeggedRLController : public LeggedController
{
public:

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  using TwistMsgSharedPtr = std::shared_ptr<geometry_msgs::msg::Twist>;
  using CmdBuffer = realtime_tools::RealtimeBuffer<TwistMsgSharedPtr>;
  std::shared_ptr<CmdBuffer> cmd_vel_buffer_;
  TwistMsgSharedPtr cmd_vel_msg_;

  std::shared_ptr<legged_rl_controller::ParamListener> param_listener_;
  legged_rl_controller::Params params_;
  void update_parameters_();
  virtual controller_interface::CallbackReturn configure_parameters_();

  // Fall detection
  bool detect_fall_();
  
  std::shared_ptr<isaaclab::Articulation> robot_;
  isaaclab::ManagerBasedRLEnvCfg env_cfg_;
  std::unique_ptr<isaaclab::ManagerBasedRLEnv> env_;
  std::string rl_policy_path_;
};



} // namespace legged

