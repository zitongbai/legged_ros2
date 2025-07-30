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
#include <deque>

#include "legged_rl_controller/legged_rl_controller_parameters.hpp"

namespace legged
{

template<typename T>
class HistoryBuffer{
public:
  HistoryBuffer(size_t buffer_size) : buffer_size_(buffer_size) {}

  void push(const T& value) {
    if (buffer_size_ == 0) {
      return;
    }
    if (buffer_.size() >= buffer_size_) {
      buffer_.pop_front(); // delete the oldest element
    }
    buffer_.push_back(value);
  }

  std::vector<T> get_all() const {
    return std::vector<T>(buffer_.begin(), buffer_.end());
  }

  size_t history_length() const {
    return buffer_size_;
  }

private:
  std::deque<T> buffer_;
  size_t buffer_size_;

};

struct ObsTerm{
  std::string name;
  std::function<const torch::Tensor&(ObsTerm*)> func;
  int obs_num;
  double scale;
  std::vector<double> clip; // clip[0] is the lower bound, clip[1] is the upper bound
  torch::Tensor obs_tensor; // tensor to hold the observation
  HistoryBuffer<torch::Tensor> history_buffer; // for history length

  size_t history_length() const {
    return history_buffer.history_length();
  }
};

using ObsFunc = std::function<const torch::Tensor&(ObsTerm*)>;

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
  
  const torch::Tensor& obs_base_ang_vel_(ObsTerm* obs_term);

  const torch::Tensor& obs_projected_gravity_(ObsTerm* obs_term);

  const torch::Tensor& obs_velocity_commands_(ObsTerm* obs_term);

  const torch::Tensor& obs_joint_pos_(ObsTerm* obs_term);

  const torch::Tensor& obs_joint_vel_(ObsTerm* obs_term);

  const torch::Tensor& obs_actions_(ObsTerm* obs_term);

  int obs_num_; // number of observations
  torch::Tensor obs_tensor_; // tensor to hold all observations

  void update_observations_();

  // TODO: consider obs history length

  // RL action
  ActionTerm action_term_;
  torch::Tensor action_tensor_;

};



} // namespace legged

