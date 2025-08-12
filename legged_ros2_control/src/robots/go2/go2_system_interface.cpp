/**
 * @file go2_system_interface.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "legged_ros2_control/robots/unitree/robots/go2/go2_system_interface.hpp"

namespace legged {

const std::string Go2SystemInterface::TOPIC_LOWCMD = "rt/lowcmd";
const std::string Go2SystemInterface::TOPIC_LOWSTATE = "rt/lowstate";

CallbackReturn Go2SystemInterface::on_init(const hardware_interface::HardwareInfo & info){
  if (LeggedSystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if(imu_data_.size() != 1){
    RCLCPP_ERROR(*logger_, "Go2SystemInterface only supports one IMU sensor");
    return CallbackReturn::ERROR;
  }

  network_interface_ = info_.hardware_parameters["network_interface"];

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.Go2SystemInterface"));

  return CallbackReturn::SUCCESS;
}

CallbackReturn Go2SystemInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

  RCLCPP_INFO(*logger_, "Configuring Go2SystemInterface...");

  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

  RCLCPP_INFO(*logger_, "Go2SystemInterface configured with network interface: %s", network_interface_.c_str());

  lowcmd_publisher_ = std::make_unique<go2::LowCmdPublisher>(TOPIC_LOWCMD);
  lowstate_subscriber_ = std::make_unique<go2::LowStateSubscriber>(TOPIC_LOWSTATE);

  RCLCPP_INFO(*logger_, "Go2SystemInterface waiting for connection...");
  lowstate_subscriber_->wait_for_connection();
  RCLCPP_INFO(*logger_, "Go2SystemInterface connected.");

  RCLCPP_INFO(*logger_, "Go2SystemInterface configured successfully.");
  return CallbackReturn::SUCCESS;
}

bool Go2SystemInterface::build_joint_data_(){
  for(size_t i=0; i<joint_data_.size(); i++){
    const auto & jnt_name = joint_data_[i].name;
    auto it = go2::joint_index_map.find(jnt_name);
    if(it == go2::joint_index_map.end()){
      RCLCPP_ERROR(*logger_, "Joint name %s not found in Go2 joint map", jnt_name.c_str());
      return false;
    }
    joint_data_[i].adr = static_cast<int>(it->second);
  }
  return true;
}

return_type Go2SystemInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  
  lowstate_subscriber_->update();

  // Read joint data
  for (size_t i=0; i < joint_data_.size(); ++i) {
    joint_data_[i].pos_ = lowstate_subscriber_->msg_.motor_state()[joint_data_[i].adr].q();
    joint_data_[i].vel_ = lowstate_subscriber_->msg_.motor_state()[joint_data_[i].adr].dq();
    joint_data_[i].tau_ = lowstate_subscriber_->msg_.motor_state()[joint_data_[i].adr].tau_est();
  }

  // Read IMU data
  // Only use one IMU here
  // Unitree SDK2 quaternion: w x y z
  // ImuData quaternion: x y z w
  imu_data_[0].quat_[0] = lowstate_subscriber_->msg_.imu_state().quaternion()[1]; // x
  imu_data_[0].quat_[1] = lowstate_subscriber_->msg_.imu_state().quaternion()[2]; // y
  imu_data_[0].quat_[2] = lowstate_subscriber_->msg_.imu_state().quaternion()[3]; // z
  imu_data_[0].quat_[3] = lowstate_subscriber_->msg_.imu_state().quaternion()[0]; // w
  imu_data_[0].ang_vel_[0] = lowstate_subscriber_->msg_.imu_state().gyroscope()[0]; // angular velocity x
  imu_data_[0].ang_vel_[1] = lowstate_subscriber_->msg_.imu_state().gyroscope()[1]; // angular velocity y
  imu_data_[0].ang_vel_[2] = lowstate_subscriber_->msg_.imu_state().gyroscope()[2]; // angular velocity z
  imu_data_[0].lin_acc_[0] = lowstate_subscriber_->msg_.imu_state().accelerometer()[0]; // linear acceleration x
  imu_data_[0].lin_acc_[1] = lowstate_subscriber_->msg_.imu_state().accelerometer()[1]; // linear acceleration y
  imu_data_[0].lin_acc_[2] = lowstate_subscriber_->msg_.imu_state().accelerometer()[2]; // linear acceleration z

  return return_type::OK;
}

return_type Go2SystemInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

  // Prepare low_cmd_

  for(size_t i=0; i < joint_data_.size(); ++i) {
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].q() = joint_data_[i].pos_cmd_;
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].dq() = joint_data_[i].vel_cmd_;
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].tau() = joint_data_[i].ff_cmd_;
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].kp() = joint_data_[i].kp_;
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].kd() = joint_data_[i].kd_;
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].mode() = (0x01);   // motor switch to servo (PMSM) mode
  }

  lowcmd_publisher_->unlockAndPublish();

  return return_type::OK;
}

} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  legged::Go2SystemInterface, legged::LeggedSystemInterface)


  