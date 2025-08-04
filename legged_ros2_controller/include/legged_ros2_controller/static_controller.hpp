/**
 * @file static_controller.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "legged_ros2_controller/legged_ros2_controller.hpp"

#include "legged_ros2_controller/static_controller_parameters.hpp"

#include <cmath>

class CosineCurve{

public:
  CosineCurve(){};

  double getPos(double t){
    if (t < t0_){
      return x0_;
    } else if (t > t1_){
      return x1_;
    } else {
      double t_normalized = (t - t0_) / (t1_ - t0_);
      double w = 0.5 - 0.5 * cos(M_PI * t_normalized);
      return x0_ * (1 - w) + x1_ * w;
    }
  };

  double getVel(double t){
    if (t < t0_ || t > t1_){
      return 0.0;
    } else {
      double t_normalized = (t - t0_) / (t1_ - t0_);
      return 0.5 * M_PI * (x1_ - x0_) * sin(M_PI * t_normalized) / (t1_ - t0_);
    }
  };
  
  void reset(double x0, double x1, double t0, double t1){
    x0_ = x0;
    x1_ = x1;
    t0_ = t0;
    t1_ = t1;
  };

  double getStartTime(){
    return t0_;
  };

  double getEndTime(){
    return t1_;
  };

private:
  double x0_;
  double x1_;
  double t0_;
  double t1_;
};

namespace legged
{
class StaticController : public LeggedController
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
};



}

