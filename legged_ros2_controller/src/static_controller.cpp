/**
 * @file static_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "legged_ros2_controller/static_controller.hpp"

namespace legged
{

controller_interface::CallbackReturn StaticController::on_init() {

  if (LeggedController::on_init() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    param_listener_ = std::make_shared<static_controller::ParamListener>(get_node());
  }catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during StaticController's on_init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StaticController::on_configure(const rclcpp_lifecycle::State & previous_state) {
  
  update_parameters_();

  imu_names_.clear(); // Static controller does not use IMU sensors

  kp_.clear();
  kd_.clear();
  default_pos_.clear();

  joint_names_ = params_.joint_names;
  for(const auto & jnt_name : joint_names_){
    const auto & jnt_cfg = params_.joint_cfg.joint_names_map.at(jnt_name);
    kp_.push_back(jnt_cfg.kp);
    kd_.push_back(jnt_cfg.kd);
    default_pos_.push_back(jnt_cfg.default_pos);
  }

  joint_curves_.resize(joint_names_.size());

  if (LeggedController::on_configure(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StaticController::on_activate(const rclcpp_lifecycle::State & previous_state) {

  if (LeggedController::on_activate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  auto init_joint_pos = joint_interface_->get_joint_position();
  double current_time = get_node()->now().seconds();
  for(size_t i=0; i<joint_names_.size(); ++i){
    joint_curves_[i].reset(init_joint_pos[i], default_pos_[i], current_time, current_time + params_.traj_time);
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type StaticController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {

  std::vector<double> joint_pos_target(joint_names_.size());
  std::vector<double> joint_vel_target(joint_names_.size());
  double current_time = time.seconds();
  for(size_t i=0; i<joint_names_.size(); ++i){
    joint_pos_target[i] = joint_curves_[i].getPos(current_time);
    joint_vel_target[i] = joint_curves_[i].getVel(current_time);
  }

  std::vector<double> ff(joint_names_.size(), 0.0);

  try{
    joint_interface_->set_joint_command(
      joint_pos_target, 
      joint_vel_target, 
      ff, 
      kp_, 
      kd_
    );
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in set_joint_command: %s", e.what());
    return controller_interface::return_type::ERROR;
  }
  
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn StaticController::on_deactivate(const rclcpp_lifecycle::State & previous_state){

  if (LeggedController::on_deactivate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_interface_->set_joint_command(
    std::vector<double>(default_pos_.size(), 0.0), 
    std::vector<double>(default_pos_.size(), 0.0), 
    std::vector<double>(default_pos_.size(), 0.0), 
    std::vector<double>(default_pos_.size(), 0.0), 
    std::vector<double>(default_pos_.size(), 0.0)
  );

  return controller_interface::CallbackReturn::SUCCESS;
}

void StaticController::update_parameters_(){
  if (!param_listener_->is_old(params_)) {
    return;
  }
  params_ = param_listener_->get_params();
}

} // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  legged::StaticController, controller_interface::ControllerInterface)

