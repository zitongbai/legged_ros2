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


#include "legged_ros2_control/robots/g1/g1_system_interface.hpp"


namespace legged {

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
};

const std::string G1SystemInterface::HG_CMD_TOPIC = "rt/lowcmd";
const std::string G1SystemInterface::HG_STATE_TOPIC = "rt/lowstate";

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
  RCLCPP_INFO(*logger_, "Trying to shutdown motion control-related service...");

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

  RCLCPP_INFO(*logger_, "Motion control-related service shutdown successfully");

  // create publisher and subscriber for Unitree SDK2
  // create publisher
  lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
  lowcmd_publisher_->InitChannel();
  // create subscriber
  low_state_ = LowState_();
  lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
  lowstate_subscriber_->InitChannel(std::bind(&G1SystemInterface::lowstate_handler_, this, std::placeholders::_1), 1);

  // Wait for connection to G1 robot, allow Ctrl+C to exit quickly
  while(!connected_ && rclcpp::ok()){
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(*logger_, "Waiting for connection to G1 robot...");
  }
  if (!rclcpp::ok()) {
    RCLCPP_WARN(*logger_, "Shutdown requested, exiting configuration early.");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(*logger_, "G1SystemInterface initialized successfully");

  return CallbackReturn::SUCCESS;
}


void G1SystemInterface::lowstate_handler_(const void * msg){
  low_state_ = *(const LowState_ *)msg;
  if (low_state_.crc() != Crc32Core((uint32_t *)&low_state_, (sizeof(LowState_) >> 2) - 1)) {
    std::cout << "[ERROR] CRC Error, " << 
      "received: " << low_state_.crc() <<
      ", calculated: " << Crc32Core((uint32_t *)&low_state_, (sizeof(LowState_) >> 2) - 1) << std::endl;
    return;
  }
  // TODO: make data thread safe
  connected_ = true;
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
  // check if low_state_ is available
  // if (low_state_.crc() != Crc32Core((uint32_t *)&low_state_, (sizeof(LowState_) >> 2) - 1)) {
  //   // RCLCPP_WARN(*logger_, "LowState CRC check failed, waiting for valid data...");
  //   return return_type::OK;
  // }

  // Read joint data
  for (size_t i=0; i < joint_data_.size(); ++i) {
    joint_data_[i].pos_ = low_state_.motor_state()[joint_data_[i].adr].q();
    joint_data_[i].vel_ = low_state_.motor_state()[joint_data_[i].adr].dq();
    joint_data_[i].tau_ = low_state_.motor_state()[joint_data_[i].adr].tau_est();
  }

  // Read IMU data
  // Only use one IMU here
  // Unitree SDK2 quaternion: w x y z
  // ImuData quaternion: x y z w
  imu_data_[0].quat_[0] = low_state_.imu_state().quaternion()[1]; // x
  imu_data_[0].quat_[1] = low_state_.imu_state().quaternion()[2]; // y
  imu_data_[0].quat_[2] = low_state_.imu_state().quaternion()[3]; // z
  imu_data_[0].quat_[3] = low_state_.imu_state().quaternion()[0]; // w
  imu_data_[0].ang_vel_[0] = low_state_.imu_state().gyroscope()[0]; // angular velocity x
  imu_data_[0].ang_vel_[1] = low_state_.imu_state().gyroscope()[1]; // angular velocity y
  imu_data_[0].ang_vel_[2] = low_state_.imu_state().gyroscope()[2]; // angular velocity z
  imu_data_[0].lin_acc_[0] = low_state_.imu_state().accelerometer()[0]; // linear acceleration x
  imu_data_[0].lin_acc_[1] = low_state_.imu_state().accelerometer()[1]; // linear acceleration y
  imu_data_[0].lin_acc_[2] = low_state_.imu_state().accelerometer()[2]; // linear acceleration z

  if(mode_machine_ != low_state_.mode_machine()){
    if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state_.mode_machine()) << std::endl;
    mode_machine_ = low_state_.mode_machine();
  }

  return return_type::OK;
}


return_type G1SystemInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

  // Prepare low_cmd_
  low_cmd_ = LowCmd_();
  low_cmd_.mode_pr() = static_cast<uint8_t>(Mode::PR);
  low_cmd_.mode_machine() = mode_machine_;

  for(size_t i=0; i < joint_data_.size(); ++i) {
    low_cmd_.motor_cmd()[joint_data_[i].adr].q() = joint_data_[i].pos_cmd_;
    low_cmd_.motor_cmd()[joint_data_[i].adr].dq() = joint_data_[i].vel_cmd_;
    low_cmd_.motor_cmd()[joint_data_[i].adr].tau() = joint_data_[i].ff_cmd_;
    low_cmd_.motor_cmd()[joint_data_[i].adr].kp() = joint_data_[i].kp_;
    low_cmd_.motor_cmd()[joint_data_[i].adr].kd() = joint_data_[i].kd_;
  }

  low_cmd_.crc() = Crc32Core((uint32_t *)&low_cmd_, (sizeof(low_cmd_) >> 2) - 1);

  lowcmd_publisher_->Write(low_cmd_);

  return return_type::OK;
}


} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  legged::G1SystemInterface, legged::LeggedSystemInterface)

