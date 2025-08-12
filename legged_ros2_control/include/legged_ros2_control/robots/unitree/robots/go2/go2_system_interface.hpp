/**
 * @file go2_system_interface.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
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

#include "legged_ros2_control/robots/unitree/robots/go2/go2_pub.h"
#include "legged_ros2_control/robots/unitree/robots/go2/go2_sub.h"
#include "legged_ros2_control/robots/unitree/robots/go2/define.h"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>


namespace legged {

using namespace unitree::common;
using namespace unitree::robot;

class HARDWARE_INTERFACE_PUBLIC Go2SystemInterface : public LeggedSystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Go2SystemInterface)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(Go2SystemInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  static const std::string TOPIC_LOWCMD;
  static const std::string TOPIC_LOWSTATE;

protected:
  bool build_joint_data_() override;

    std::string network_interface_;

    std::unique_ptr<go2::LowCmdPublisher> lowcmd_publisher_;
    std::unique_ptr<go2::LowStateSubscriber> lowstate_subscriber_;

};


}

