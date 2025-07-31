/**
 * @file legged_system_interface.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "legged_ros2_control/legged_system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace legged
{

CallbackReturn LeggedSystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.LeggedSystemInterface"));

  // Resize vectors based on the number of joints
  joint_data_.resize(info.joints.size());
  for (size_t i = 0; i < info.joints.size(); ++i) {
    joint_data_[i].name = info.joints[i].name;
    
    auto it = std::find_if(
      info.joints[i].command_interfaces.begin(),
      info.joints[i].command_interfaces.end(),
      [](const hardware_interface::InterfaceInfo & interface) {
        return interface.name == hardware_interface::HW_IF_EFFORT;
      });
    if (it != info.joints[i].command_interfaces.end()) {
      joint_data_[i].tau_range_[0] = std::stod(it->min);
      joint_data_[i].tau_range_[1] = std::stod(it->max);
    } else {
      joint_data_[i].tau_range_[0] = -std::numeric_limits<double>::infinity();
      joint_data_[i].tau_range_[1] = std::numeric_limits<double>::infinity();
    }

    RCLCPP_INFO(*logger_, "Joint %zu: %s with torque range [%f, %f]",
                i, joint_data_[i].name.c_str(), joint_data_[i].tau_range_[0], joint_data_[i].tau_range_[1]);
  }

  imu_data_.resize(info.sensors.size());
  for (size_t i=0; i<info.sensors.size(); i++){
    imu_data_[i].name = info.sensors[i].name;
    RCLCPP_INFO(*logger_, "IMU %zu: %s", i, imu_data_[i].name.c_str());
  }

  if(!build_joint_data_()){
    RCLCPP_ERROR_STREAM(*logger_, "Failed to build joint data.");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;

}

std::vector<hardware_interface::StateInterface> LeggedSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // joint position, velocity, and effort state interfaces
  for (size_t i = 0; i < joint_data_.size(); ++i) {
    state_interfaces.emplace_back(joint_data_[i].name, hardware_interface::HW_IF_POSITION, &joint_data_[i].pos_);
    state_interfaces.emplace_back(joint_data_[i].name, hardware_interface::HW_IF_VELOCITY, &joint_data_[i].vel_);
    state_interfaces.emplace_back(joint_data_[i].name, hardware_interface::HW_IF_EFFORT, &joint_data_[i].tau_);
  }
  
  // IMU interface
  for(size_t i=0; i<imu_data_.size(); i++){
    state_interfaces.emplace_back(imu_data_[i].name, "orientation.x", &imu_data_[i].quat_[0]);
    state_interfaces.emplace_back(imu_data_[i].name, "orientation.y", &imu_data_[i].quat_[1]);
    state_interfaces.emplace_back(imu_data_[i].name, "orientation.z", &imu_data_[i].quat_[2]);
    state_interfaces.emplace_back(imu_data_[i].name, "orientation.w", &imu_data_[i].quat_[3]);
    state_interfaces.emplace_back(imu_data_[i].name, "angular_velocity.x", &imu_data_[i].ang_vel_[0]);
    state_interfaces.emplace_back(imu_data_[i].name, "angular_velocity.y", &imu_data_[i].ang_vel_[1]);
    state_interfaces.emplace_back(imu_data_[i].name, "angular_velocity.z", &imu_data_[i].ang_vel_[2]);
    state_interfaces.emplace_back(imu_data_[i].name, "linear_acceleration.x", &imu_data_[i].lin_acc_[0]);
    state_interfaces.emplace_back(imu_data_[i].name, "linear_acceleration.y", &imu_data_[i].lin_acc_[1]);
    state_interfaces.emplace_back(imu_data_[i].name, "linear_acceleration.z", &imu_data_[i].lin_acc_[2]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LeggedSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // joint position, velocity, and effort command interfaces
  for (size_t i = 0; i < joint_data_.size(); ++i) {
    command_interfaces.emplace_back(joint_data_[i].name, hardware_interface::HW_IF_POSITION, &joint_data_[i].pos_cmd_);
    command_interfaces.emplace_back(joint_data_[i].name, hardware_interface::HW_IF_VELOCITY, &joint_data_[i].vel_cmd_);
    command_interfaces.emplace_back(joint_data_[i].name, hardware_interface::HW_IF_EFFORT, &joint_data_[i].ff_cmd_);
    command_interfaces.emplace_back(joint_data_[i].name, hardware_interface::HW_IF_KP, &joint_data_[i].kp_);
    command_interfaces.emplace_back(joint_data_[i].name, hardware_interface::HW_IF_KD, &joint_data_[i].kd_);
  }

  return command_interfaces;
}


}  // namespace legged
