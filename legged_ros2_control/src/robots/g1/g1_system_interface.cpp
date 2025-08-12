/**
 * @file g1_system_interface.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include "legged_ros2_control/robots/unitree/robots/g1/g1_system_interface.hpp"

namespace legged {

CallbackReturn G1SystemInterface::on_init(const hardware_interface::HardwareInfo & info){
  if (LeggedSystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if(imu_data_.size() != 1){
    RCLCPP_ERROR(*logger_, "G1SystemInterface only supports one IMU sensor");
    return CallbackReturn::ERROR;
  }

  network_interface_ = info_.hardware_parameters["network_interface"];

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.G1SystemInterface"));

  return CallbackReturn::SUCCESS;
}


CallbackReturn G1SystemInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

  RCLCPP_INFO(*logger_, "Configuring G1SystemInterface...");

  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

  RCLCPP_INFO(*logger_, "G1SystemInterface configured with network interface: %s", network_interface_.c_str());

  // -------------------------------------- TODO --------------------------------------
  // RCLCPP_INFO(*logger_, "Trying to shutdown motion control-related service...");
  // try {
  //   // try to shutdown motion control-related service
  //   msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
  //   std::cout << "111111111111111" << std::endl;
  //   msc_->SetTimeout(5.0f);
  //   std::cout << "22222222222222222" << std::endl;
  //   msc_->Init();
  //   std::cout << "333333333333333" << std::endl;
  //   std::string form, name;
  //   while (msc_->CheckMode(form, name), !name.empty()) {
  //     std::cout << "44444444444444" << std::endl;
  //     if (msc_->ReleaseMode())
  //       RCLCPP_WARN(*logger_, "Failed to switch to Release Mode");
  //     sleep(5);
  //   }
  // } catch (const std::exception& e) {
  //   RCLCPP_ERROR(*logger_, "Error in motion switcher: %s", e.what());
  //   return CallbackReturn::ERROR;
  // }
  // RCLCPP_INFO(*logger_, "Motion control-related service shutdown successfully");

  lowstate_subscriber_ = std::make_shared<g1::LowStateSubscriber>();
  lowcmd_publisher_ = std::make_unique<g1::LowCmdPublisher>();

  RCLCPP_INFO(*logger_, "G1SystemInterface waiting for connection to G1 robot...");
  lowstate_subscriber_->wait_for_connection();
  RCLCPP_INFO(*logger_, "G1SystemInterface connected to G1 robot");

  RCLCPP_INFO(*logger_, "G1SystemInterface initialized successfully");

  return CallbackReturn::SUCCESS;
}


bool G1SystemInterface::build_joint_data_(){
  for(size_t i=0; i<joint_data_.size(); i++){
    const auto & jnt_name = joint_data_[i].name;
    auto it = g1_joint_index_map.find(jnt_name);
    if (it == g1_joint_index_map.end()) {
      RCLCPP_ERROR(*logger_, "Joint %s not found in G1 joint index map", jnt_name.c_str());
      return false;
    }
    joint_data_[i].adr = it->second;
  }
  return true;
}


return_type G1SystemInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

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

  if(mode_machine_ != lowstate_subscriber_->msg_.mode_machine()){
    if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(lowstate_subscriber_->msg_.mode_machine()) << std::endl;
    mode_machine_ = lowstate_subscriber_->msg_.mode_machine();
  }

  return return_type::OK;
}


return_type G1SystemInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

  // Prepare low_cmd_

  lowcmd_publisher_->msg_.mode_machine() = mode_machine_;

  for(size_t i=0; i < joint_data_.size(); ++i) {
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].q() = joint_data_[i].pos_cmd_;
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].dq() = joint_data_[i].vel_cmd_;
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].tau() = joint_data_[i].ff_cmd_;
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].kp() = joint_data_[i].kp_;
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].kd() = joint_data_[i].kd_;
    lowcmd_publisher_->msg_.motor_cmd()[joint_data_[i].adr].mode(1); // 1:Enable, 0:Disable
  }

  lowcmd_publisher_->unlockAndPublish();

  // // --- Period statistics ---
  // static size_t call_count = 0;
  // static double period_sum = 0.0;
  // static double max_period = 0.0;
  // double current_period = period.seconds();
  // call_count++;
  // period_sum += current_period;
  // if (current_period > max_period) {
  //   max_period = current_period;
  // }
  // if (call_count == 100) {
  //   double avg_period = period_sum / 100.0;
  //   RCLCPP_INFO(*logger_, "Average write period over 100 calls: %.6f s, Max period: %.6f s", avg_period, max_period);
  //   call_count = 0;
  //   period_sum = 0.0;
  //   max_period = 0.0;
  // }

  return return_type::OK;
}


} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  legged::G1SystemInterface, legged::LeggedSystemInterface)

