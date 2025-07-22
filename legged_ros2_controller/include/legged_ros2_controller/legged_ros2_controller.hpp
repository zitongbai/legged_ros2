/**
 * @file legged_ros2_controller.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>


#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "semantic_components/imu_sensor.hpp"
#include "legged_ros2_controller/semantic_components/joint_interface.hpp"

namespace legged{

class LeggedController : public controller_interface::ControllerInterface
{
public:
  LeggedController() : controller_interface::ControllerInterface() {};

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  
  std::vector<std::string> joint_names_;
  std::vector<std::string> imu_names_;

  std::vector<std::unique_ptr<semantic_components::IMUSensor>> imu_interfaces_;
  std::unique_ptr<JointInterface> joint_interface_;

};

} // namespace legged


