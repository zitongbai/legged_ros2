/**
 * @file mujoco_system_interface.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "legged_ros2_control/mujoco/mujoco_system_interface.hpp"

namespace legged
{

CallbackReturn MujocoSystemInterface::on_init(const hardware_interface::HardwareInfo & info){
  if (LeggedSystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.MujocoSystemInterface"));

  return CallbackReturn::SUCCESS;
}

  
return_type MujocoSystemInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // check if the model is still valid
  if (mj_model_ == nullptr || mj_data_ == nullptr) {
    RCLCPP_ERROR_STREAM(*logger_, "Mujoco model or data is not valid.");
    return return_type::ERROR;
  }

  // Read joint data
  for(size_t i=0; i<joint_data_.size(); i++){
    joint_data_[i].pos_ = mj_data_->qpos[mj_extra_joint_data_[i].mj_pos_adr];
    joint_data_[i].vel_ = mj_data_->qvel[mj_extra_joint_data_[i].mj_vel_adr];
    joint_data_[i].tau_ = mj_data_->qfrc_applied[mj_extra_joint_data_[i].mj_vel_adr];
  }

  /**
   *  Read IMU data
   *    quaternion convention for ImuData: x, y, z, w
   *    quaternion convention for mujoco: w, x, y, z
   *
   * The sensor configuration in Mujoco XML might look like this:
        <framequat name="imu_quat" objtype="site" objname="imu" />
        <gyro name="imu_gyro" site="imu" />
        <accelerometer name="imu_acc" site="imu" />
   */
  for(size_t i=0; i<imu_data_.size(); i++){
    imu_data_[i].quat_[0] = mj_data_->sensordata[10*i + 1]; // x
    imu_data_[i].quat_[1] = mj_data_->sensordata[10*i + 2]; // y
    imu_data_[i].quat_[2] = mj_data_->sensordata[10*i + 3]; // z
    imu_data_[i].quat_[3] = mj_data_->sensordata[10*i + 0]; // w
    imu_data_[i].ang_vel_[0] = mj_data_->sensordata[10*i + 4]; // angular velocity x
    imu_data_[i].ang_vel_[1] = mj_data_->sensordata[10*i + 5]; // angular velocity y
    imu_data_[i].ang_vel_[2] = mj_data_->sensordata[10*i + 6]; // angular velocity z
    imu_data_[i].lin_acc_[0] = mj_data_->sensordata[10*i + 7]; // linear acceleration x
    imu_data_[i].lin_acc_[1] = mj_data_->sensordata[10*i + 8]; // linear acceleration y
    imu_data_[i].lin_acc_[2] = mj_data_->sensordata[10*i + 9]; // linear acceleration z
  }

  // TODO: handle this in other places
  // // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  // for(size_t i = 0; i < joint_data_.size(); ++i) {
  //   joint_data_[i].pos_cmd_ = joint_data_[i].pos_;
  //   joint_data_[i].vel_cmd_ = joint_data_[i].vel_;
  //   joint_data_[i].ff_cmd_ = 0.0;
  //   joint_data_[i].kp_ = 0.0;
  //   joint_data_[i].kd_ = 0.0;
  // }

  return return_type::OK;
}

return_type MujocoSystemInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // check if the model is still valid
  if (mj_model_ == nullptr || mj_data_ == nullptr) {
    RCLCPP_ERROR_STREAM(*logger_, "Mujoco model or data is not valid.");
    return return_type::ERROR;
  }
  // const double max_torque = 50;  // TODO: get from config or parameter
  // Write joint commands
  for(size_t i=0; i<joint_data_.size(); i++){
    double torque_target = joint_data_[i].ff_cmd_ + 
                      joint_data_[i].kp_ * (joint_data_[i].pos_cmd_ - joint_data_[i].pos_) +
                      joint_data_[i].kd_ * (joint_data_[i].vel_cmd_ - joint_data_[i].vel_);
    torque_target = std::clamp(torque_target, joint_data_[i].tau_range_[0], joint_data_[i].tau_range_[1]);
    mj_data_->qfrc_applied[mj_extra_joint_data_[i].mj_vel_adr] = torque_target;
  }

  // std::cout << "pos: "<< joint_data_[0].pos_ << ", vel: " << joint_data_[0].vel_ <<
  //   ", pos_cmd: " << joint_data_[0].pos_cmd_ << ", vel_cmd: " << joint_data_[0].vel_cmd_ <<
  //   ", ff_cmd: " << joint_data_[0].ff_cmd_ <<
  //   ", torque_target: " << mj_data_->qfrc_applied[mj_extra_joint_data_[0].mj_vel_adr] << 
  //   " kp: " << joint_data_[0].kp_ << ", kd: " << joint_data_[0].kd_ << std::endl;

  return return_type::OK;
}


void MujocoSystemInterface::init_sim(mjModel *mujoco_model, mjData *mujoco_data){
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;
  const char * name = mj_id2name(mj_model_, mjtObj::mjOBJ_KEY, 0); // Initialize keyframe names
  if (name != nullptr){
    mj_resetDataKeyframe(mj_model_, mj_data_, 0); // Reset to the first keyframe
  } else {
    mj_resetData(mj_model_, mj_data_);
  }
}


bool MujocoSystemInterface::build_joint_data_(){

  mj_extra_joint_data_.resize(joint_data_.size());

  for(size_t i=0; i<joint_data_.size(); i++){
    auto jnt_name = joint_data_[i].name;
    int mj_jnt_id = mj_name2id(mj_model_, mjtObj::mjOBJ_JOINT, jnt_name.c_str());
    if(mj_jnt_id < 0){
      RCLCPP_ERROR_STREAM(
        *logger_,
        "Joint name '" << jnt_name << "' not found in Mujoco model.");
      return false;
    }

    mj_extra_joint_data_[i].mj_pos_adr = mj_model_->jnt_qposadr[mj_jnt_id];
    mj_extra_joint_data_[i].mj_vel_adr = mj_model_->jnt_dofadr[mj_jnt_id];

    int mj_ctrl_id = mj_name2id(mj_model_, mjtObj::mjOBJ_ACTUATOR, jnt_name.c_str());
    if(mj_ctrl_id < 0){
      RCLCPP_ERROR_STREAM(
        *logger_,
        "Joint name '" << jnt_name << "' not found in Mujoco model as an actuator.");
      return false;
    }

    mj_extra_joint_data_[i].mj_ctrl_adr = mj_ctrl_id;

    if (mj_model_->jnt_actfrclimited[mj_jnt_id]){
      joint_data_[i].tau_range_[0] = mj_model_->jnt_actfrcrange[2 * mj_jnt_id + 0];
      joint_data_[i].tau_range_[1] = mj_model_->jnt_actfrcrange[2 * mj_jnt_id + 1];
    } else if (mj_model_->actuator_ctrllimited[mj_ctrl_id]) {
      joint_data_[i].tau_range_[0] = mj_model_->actuator_ctrlrange[2 * mj_ctrl_id + 0];
      joint_data_[i].tau_range_[1] = mj_model_->actuator_ctrlrange[2 * mj_ctrl_id + 1];
    } else {
      joint_data_[i].tau_range_[0] = -std::numeric_limits<double>::infinity();
      joint_data_[i].tau_range_[1] = std::numeric_limits<double>::infinity();
    }

    // debug print:
    RCLCPP_INFO_STREAM(
      *logger_,
      "Joint name '" << jnt_name << "' has Mujoco position address: " << mj_extra_joint_data_[i].mj_pos_adr <<
      ", velocity address: " << mj_extra_joint_data_[i].mj_vel_adr <<
      ", control address: " << mj_extra_joint_data_[i].mj_ctrl_adr <<
      ", torque range: [" << joint_data_[i].tau_range_[0] << ", " << joint_data_[i].tau_range_[1] << "]"
    );
  }

  // TODO: joint limits

  return true;
}

} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  legged::MujocoSystemInterface, legged::MujocoSystemInterfaceBase)