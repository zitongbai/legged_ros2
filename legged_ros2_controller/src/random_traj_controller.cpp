/**
 * @file random_traj_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include "legged_ros2_controller/random_traj_controller.hpp" 

namespace legged {

controller_interface::CallbackReturn RandomTrajController::on_init() {

  if (LeggedController::on_init() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    param_listener_ = std::make_shared<static_controller::ParamListener>(get_node());
  }catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during RandomTrajController's on_init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RandomTrajController::on_configure(const rclcpp_lifecycle::State & previous_state) {
  
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

  targetNum_ = 4;
  targetPos_.resize(targetNum_);
  targetPos_.at(0) = {0.0, 1.36, -2.40, 0.0, 1.36, -2.40,
                      -0.2, 1.36, -2.40, 0.2, 1.36, -2.40}; // ready
  targetPos_.at(1) = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                      0.0, 0.67, -1.3, 0.0, 0.67, -1.3};  // stand
  targetPos_.at(2) = {-0.35, 1.36, -2.40, 0.35, 1.36, -2.40,
                      -0.5, 1.36, -2.40, 0.5, 1.36, -2.40}; // down
  targetPos_.at(3) = {-0.2, 0.5, -1.0, 0.2, 0.5, -1.0,
                      -0.4, 0.8, -1.0, 0.4, 0.8, -1.0};  // stand

  // random noise generator
  gen_ = std::mt19937(rd_());
  noiseVariance_ = 0.5;
  normalDist_ = std::normal_distribution<double>(0.0, noiseVariance_);
  noiseScale_ = 0.5;

  if (LeggedController::on_configure(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RandomTrajController::on_activate(const rclcpp_lifecycle::State & previous_state) {

  if (LeggedController::on_activate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  auto init_joint_pos = joint_interface_->get_joint_position();
  double current_time = get_node()->now().seconds();
  targetIdx_ = 0;
  for(size_t i=0; i<joint_names_.size(); ++i){
    joint_curves_[i].reset(init_joint_pos[i], targetPos_[targetIdx_][i], current_time, current_time + params_.traj_time);
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RandomTrajController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {

  auto current_joint_pos = joint_interface_->get_joint_position();

  // check if need to update target
  double current_time = time.seconds();
  if (current_time > joint_curves_[0].getEndTime()){
    targetIdx_ = (targetIdx_ + 1) % targetNum_;
    for(size_t i=0; i<joint_names_.size(); ++i){
      double noise = noiseScale_ * normalDist_(gen_);
      joint_curves_[i].reset(
        current_joint_pos[i],
        targetPos_[targetIdx_][i] + noise, 
        current_time, 
        current_time + params_.traj_time
      );
    }
  }

  std::vector<double> joint_pos_target(joint_names_.size());
  std::vector<double> joint_vel_target(joint_names_.size());
  // double current_time = time.seconds();
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

controller_interface::CallbackReturn RandomTrajController::on_deactivate(const rclcpp_lifecycle::State & previous_state){

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

void RandomTrajController::update_parameters_(){
  if (!param_listener_->is_old(params_)) {
    return;
  }
  params_ = param_listener_->get_params();
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  legged::RandomTrajController, controller_interface::ControllerInterface)


