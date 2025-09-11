/**
 * @file random_traj_controller.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief This controller is used to collect data for actuator network training
 * @version 0.1
 * @date 2025-09-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "legged_ros2_controller/legged_ros2_controller.hpp"
#include "legged_ros2_controller/static_controller.hpp"

#include <random>

namespace legged{

class RandomTrajController : public LeggedController
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

private:
  std::shared_ptr<static_controller::ParamListener> param_listener_;
  static_controller::Params params_;
  void update_parameters_();

  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<double> default_pos_;
  std::vector<CosineCurve> joint_curves_;

  std::vector<std::vector<double>> targetPos_;
  int targetIdx_;
  int targetNum_;
  
  std::random_device rd_;
  std::mt19937 gen_;
  std::normal_distribution<double> normalDist_;
  double noiseVariance_ = 0.1;
  double noiseScale_ = 0.1;

};


}





