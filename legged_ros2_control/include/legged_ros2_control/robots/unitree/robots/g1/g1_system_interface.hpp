/**
 * @file g1_system_interface.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <map>
#include <mutex>
#include "rclcpp_lifecycle/state.hpp"

#include "legged_ros2_control/legged_system_interface.hpp"
#include "legged_ros2_control/visibility_control.h"
#include "legged_ros2_control/robots/unitree/robots/g1/define.hpp"
#include "legged_ros2_control/robots/unitree/robots/g1/g1_pub.hpp"
#include "legged_ros2_control/robots/unitree/robots/g1/g1_sub.hpp"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

namespace legged {

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::g1;
using namespace unitree_hg::msg::dds_;

class HARDWARE_INTERFACE_PUBLIC G1SystemInterface : public LeggedSystemInterface
{

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(G1SystemInterface)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(G1SystemInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:

  bool build_joint_data_() override;

  std::string network_interface_;
  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

  std::unique_ptr<g1::LowCmdPublisher> lowcmd_publisher_;
  std::shared_ptr<g1::LowStateSubscriber> lowstate_subscriber_;

  bool enable_lowlevel_write_ = false; // if true, do not write lowcmd to robot

  uint8_t mode_machine_ = 0;  // G1 Type

};


}
