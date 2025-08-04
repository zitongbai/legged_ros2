/**
 * @file legged_ros2_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "legged_ros2_controller/legged_ros2_controller.hpp"


#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>


using config_type = controller_interface::interface_configuration_type;

namespace legged{

controller_interface::CallbackReturn LeggedController::on_init(){
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/) {
  
  if(joint_names_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "Joint names are empty, please get them in on_init() method.");
      return CallbackReturn::ERROR;
  }

  if(imu_names_.empty()){
      RCLCPP_WARN(get_node()->get_logger(), "No IMU sensors configured.");
  }

  joint_interface_ = std::make_unique<JointInterface>(joint_names_);
  for(const auto & imu: imu_names_){
    imu_interfaces_.emplace_back(std::make_unique<semantic_components::IMUSensor>(imu));
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LeggedController::command_interface_configuration() const{
  controller_interface::InterfaceConfiguration conf;
  conf.type = config_type::INDIVIDUAL;
  conf.names = joint_interface_->get_command_interface_names();
  return conf;
}

controller_interface::InterfaceConfiguration LeggedController::state_interface_configuration() const{
  controller_interface::InterfaceConfiguration conf;
  conf.type = config_type::INDIVIDUAL;
  auto joint_state_if_names = joint_interface_->get_state_interface_names();
  std::vector<std::string> imu_state_if_names;
  for (const auto & imu_if : imu_interfaces_) {
    auto imu_if_names = imu_if->get_state_interface_names();
    imu_state_if_names.insert(imu_state_if_names.end(), imu_if_names.begin(), imu_if_names.end());
  }
  conf.names.reserve(joint_state_if_names.size() + imu_state_if_names.size());
  conf.names.insert(conf.names.end(), joint_state_if_names.begin(), joint_state_if_names.end());
  conf.names.insert(conf.names.end(), imu_state_if_names.begin(), imu_state_if_names.end());
  return conf;
}

controller_interface::CallbackReturn LeggedController::on_activate(const rclcpp_lifecycle::State &){
  bool success = true;
  success &= joint_interface_->assign_loaned_state_interfaces(state_interfaces_);
  for(auto & imu_if : imu_interfaces_){
    success &= imu_if->assign_loaned_state_interfaces(state_interfaces_);
  }
  
  success &= joint_interface_->assign_loaned_command_interfaces(command_interfaces_);

  return success ? controller_interface::CallbackReturn::SUCCESS : controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn LeggedController::on_deactivate(const rclcpp_lifecycle::State &){
  joint_interface_->release_interfaces();
  for(auto & imu_if : imu_interfaces_){
    imu_if->release_interfaces();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeggedController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

  return controller_interface::return_type::OK;
}


} // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  legged::LeggedController, controller_interface::ControllerInterface)
