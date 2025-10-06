/**
 * @file mujoco_ros2_control.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <cmath>
#include <algorithm>
#include <limits>

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "legged_ros2_control/legged_ros2_control.hpp"
#include "legged_ros2_control/mujoco/mujoco_system_interface.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace legged {

class ElasticBand{
public:
  ElasticBand(
    double stiffness=200, double damping=100
  ) : stiffness_(stiffness), damping_(damping), enable_(true) {}
  ~ElasticBand() = default;

  void compute_force(const std::vector<double> &x, const std::vector<double> &dx){
    // 输入合法性检查
    if(x.size() < 3 || dx.size() < 3){
      // 输入不合法，清零并返回
      force_[0] = force_[1] = force_[2] = 0.0;
      return;
    }

    if(!enable_){
      force_[0] = force_[1] = force_[2] = 0.0;
      return;
    }

    std::vector<double> vec(3);
    for(int i=0; i<3; ++i){
      vec[i] = hang_point_[i] - x[i];
    }
    double dist = std::sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);

    // 若未超过弹性长度或距离极小（避免数值问题），清零并返回
    const double eps = std::numeric_limits<double>::epsilon() * 1e3;
    if(dist <= length_ || dist <= eps){
      force_[0] = force_[1] = force_[2] = 0.0;
      return;
    }

    std::vector<double> dir(3);
    for(int i=0; i<3; ++i){
      dir[i] = vec[i] / dist;
    }

    double v_proj = dx[0]*dir[0] + dx[1]*dir[1] + dx[2]*dir[2];
    double f_mag = stiffness_ * (dist - length_) - damping_ * v_proj;
    for(int i=0; i<3; ++i){
      force_[i] = f_mag * dir[i];
    }
  }

  void adjust_length(double delta){
    length_ += delta;
    if(length_ < 0.0) length_ = 0.0;
  }

  bool is_enabled() const { return enable_; }
  void set_enabled(bool enable) { enable_ = enable; }

  const std::vector<double>& force() const { return force_; }
  std::vector<double>& force() { return force_; }

  double length() const { return length_; }

private:
  double stiffness_;
  double damping_;

  std::vector<double> hang_point_ = {0, 0, 3}; // fixed point of the elastic band
  double length_ = 0.0;
  bool enable_ = true;
  std::vector<double> force_ = {0, 0, 0};
};

class MujocoRos2Control : public LeggedRos2Control {

public:
  MujocoRos2Control(rclcpp::Node::SharedPtr node, mjModel *mujoco_model, mjData *mujoco_data) 
      : LeggedRos2Control(node), mj_model_(mujoco_model), mj_data_(mujoco_data) {
  
    RCLCPP_INFO(node_->get_logger(), "Creating MujocoRos2Control...");

    elastic_band_enable_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "elastic_band/enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          bool enable = msg->data;
          elastic_band_.set_enabled(enable);
          RCLCPP_INFO(node_->get_logger(), "Elastic band is %s.", elastic_band_.is_enabled() ? "enabled" : "disabled");
        });

    elastic_band_adjust_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "elastic_band/adjust_length", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          double delta = msg->data;
          elastic_band_.adjust_length(delta);
          RCLCPP_INFO(node_->get_logger(), "Elastic band length is adjusted to: %.3f", elastic_band_.length());
        });

    RCLCPP_INFO(node_->get_logger(), "MujocoRos2Control created.");
  }

  void update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:

  void import_components_(std::vector<hardware_interface::HardwareInfo> &hardware_info, 
                        std::unique_ptr<hardware_interface::ResourceManager> &resource_manager) override;

  std::shared_ptr<pluginlib::ClassLoader<MujocoSystemInterfaceBase>> system_interface_loader_;


  mjModel *mj_model_;
  mjData *mj_data_;
  
  ElasticBand elastic_band_ = ElasticBand();
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> elastic_band_enable_sub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> elastic_band_adjust_sub_;
};

}