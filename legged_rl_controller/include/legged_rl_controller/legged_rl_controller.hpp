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
#include <torch/script.h>
#include <Eigen/Geometry>

#include "legged_rl_controller/legged_rl_controller_parameters.hpp"

namespace legged
{

using ObsFunc = std::function<const torch::Tensor&()>;

struct ObsTerm{
  std::string name;
  ObsFunc func;
  int obs_num;
  std::vector<double> clip; // clip[0] is the lower bound, clip[1] is the upper bound
  double scale;
  // todo: history length
};

struct ActionTerm{
  std::vector<double> clip_min;
  std::vector<double> clip_max;
  std::vector<double> scale;
  std::vector<double> kp;
  std::vector<double> kd;
  std::vector<double> default_joint_pos;
};

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

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  using TwistMsgSharedPtr = std::shared_ptr<geometry_msgs::msg::Twist>;
  realtime_tools::RealtimeBuffer<TwistMsgSharedPtr> cmd_vel_buffer_;
  TwistMsgSharedPtr cmd_vel_msg_;

  std::shared_ptr<legged_rl_controller::ParamListener> param_listener_;
  legged_rl_controller::Params params_;
  void update_parameters_();
  controller_interface::CallbackReturn configure_parameters_();

  // RL policy network
  std::string rl_policy_path_;
  std::shared_ptr<torch::jit::script::Module> policy_net_;

  // RL observation
  std::vector<ObsTerm> obs_terms_;

  int find_obs_func_(const std::string& name, ObsFunc& func);
  
  const torch::Tensor& obs_base_ang_vel_();
  torch::Tensor base_ang_vel_tensor_;

  const torch::Tensor& obs_projected_gravity_();
  torch::Tensor projected_gravity_tensor_;

  const torch::Tensor& obs_velocity_commands_();
  torch::Tensor velocity_commands_tensor_;

  const torch::Tensor& obs_joint_pos_();
  torch::Tensor joint_pos_tensor_;

  const torch::Tensor& obs_joint_vel_();
  torch::Tensor joint_vel_tensor_;

  const torch::Tensor& obs_actions_();
  torch::Tensor actions_tensor_;

  int obs_num_; // number of observations
  torch::Tensor obs_tensor_; // tensor to hold all observations

  void update_observations_();

  // TODO: consider obs history length

  // RL action
  ActionTerm action_term_;

};



} // namespace legged

