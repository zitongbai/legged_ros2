/**
 * @file mujoco_system_interface.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "legged_ros2_control/legged_system_interface.hpp"
#include "legged_ros2_control/visibility_control.h"

#include "mujoco/mujoco.h"
#include "urdf/model.h"
#include "rclcpp/rclcpp.hpp"

namespace legged
{

struct MjExtraJointData
{
  int mj_pos_adr;  // Mujoco position address
  int mj_vel_adr;  // Mujoco velocity address
  int mj_ctrl_adr; // Mujoco control address
};

class HARDWARE_INTERFACE_PUBLIC MujocoSystemInterfaceBase : public LeggedSystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MujocoSystemInterfaceBase)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(MujocoSystemInterfaceBase)

  virtual void init_sim(
    mjModel *mujoco_model, mjData *mujoco_data
  ) = 0;
};

class HARDWARE_INTERFACE_PUBLIC MujocoSystemInterface : public MujocoSystemInterfaceBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MujocoSystemInterface)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(MujocoSystemInterface)

  void init_sim(
    mjModel *mujoco_model, mjData *mujoco_data
  ) override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  mjModel *mj_model_;
  mjData *mj_data_;

  std::vector<MjExtraJointData> mj_extra_joint_data_;
  bool build_joint_data_() override;


};


} // namespace legged

