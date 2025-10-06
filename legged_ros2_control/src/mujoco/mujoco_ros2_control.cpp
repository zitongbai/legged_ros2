/**
 * @file mujoco_ros2_control.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "legged_ros2_control/mujoco/mujoco_ros2_control.hpp"

namespace legged {

void MujocoRos2Control::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  if(elastic_band_.is_enabled()){
    std::vector<double> pos(3);
    std::vector<double> vel(3);
    for(int i=0; i<3; ++i){
      pos[i] = mj_data_->qpos[i];
      vel[i] = mj_data_->qvel[i];
    }
    elastic_band_.compute_force(pos, vel);
    const auto & force = elastic_band_.force();
    // apply the force to the body
    for(int i=0; i<3; ++i){
      mj_data_->qfrc_applied[i] = force[i];
    }
  } else {
    // clear the applied force
    for(int i=0; i<3; ++i){
      mj_data_->qfrc_applied[i] = 0.0;
    }
  }

  mj_step1(mj_model_, mj_data_);

  LeggedRos2Control::update(time, period);

  mj_step2(mj_model_, mj_data_);
}

void MujocoRos2Control::import_components_(std::vector<hardware_interface::HardwareInfo> &hardware_info, 
                        std::unique_ptr<hardware_interface::ResourceManager> &resource_manager)
{
  // Create the system interface loader
  try{
    system_interface_loader_.reset(new pluginlib::ClassLoader<MujocoSystemInterfaceBase>(
      "legged_ros2_control", "legged::MujocoSystemInterfaceBase"));
  }catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to create hardware interface loader:  " << ex.what());
    return;
  }

  // Import components according to the hardware info
  for(const auto & hw_info: hardware_info){
    std::string hw_class_type = hw_info.hardware_class_type;
    MujocoSystemInterfaceBase::UniquePtr system_interface;
    try{
      system_interface = MujocoSystemInterfaceBase::UniquePtr(
        system_interface_loader_->createUnmanagedInstance(hw_class_type));
    }catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to create system interface for " << hw_class_type << ": " << ex.what());
      continue;
    }

    system_interface->init_sim(mj_model_, mj_data_);

    resource_manager->import_component(std::move(system_interface), hw_info);

    rclcpp_lifecycle::State state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, 
      hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager->set_component_state(hw_info.name, state);
  }
}

}

