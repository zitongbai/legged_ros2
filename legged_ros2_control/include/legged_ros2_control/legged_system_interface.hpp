/**
 * @file legged_system_interface.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief ROS2 Control hardware interface for legged robots
 * @version 0.1
 * @date 2025-07-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "legged_ros2_control/visibility_control.h"

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "sensor_msgs/msg/imu.hpp"

using hardware_interface::return_type;

namespace hardware_interface{
/// Constant defining Kp interface
constexpr char HW_IF_KP[] = "kp";
/// Constant defining Kd interface
constexpr char HW_IF_KD[] = "kd";
/// Constant defining Desired Position interface
constexpr char HW_IF_POSITION_DES[] = "position_des";
/// Constant defining Desired Velocity interface
constexpr char HW_IF_VELOCITY_DES[] = "velocity_des";
}


namespace legged
{

struct JointData{
  std::string name;
  double pos_, vel_, tau_;
  double pos_cmd_, vel_cmd_, ff_cmd_;
  double kp_, kd_;
  double tau_range_[2]; // min, max torque range
  int adr; // index in sdk or sim
  double pos_des_, vel_des_; // desired position and velocity, used for reading
};

struct ImuData{
  std::string name;
  double quat_[4];  // quaternion, x, y, z, w
  double ang_vel_[3]; // angular velocity, x, y, z
  double lin_acc_[3]; // linear acceleration, x, y, z
};


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC LeggedSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LeggedSystemInterface)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(LeggedSystemInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

protected:

  std::shared_ptr<rclcpp::Logger> logger_;

  // Data
  std::vector<JointData> joint_data_;
  std::vector<ImuData> imu_data_;

  virtual bool build_joint_data_() = 0;

};

}  // namespace legged


