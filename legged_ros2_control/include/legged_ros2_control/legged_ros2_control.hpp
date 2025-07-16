/**
 * @file legged_ros2_control.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief This class serves as an substitute for the `ros2_control_node`
 *       https://github.com/ros-controls/ros2_control/blob/humble/controller_manager/src/ros2_control_node.cpp
 * @ref https://github.com/moveit/mujoco_ros2_control/blob/main/mujoco_ros2_control/src/mujoco_ros2_control.cpp
 * @version 0.1
 * @date 2025-07-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <memory>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "legged_ros2_control/legged_system_interface.hpp"

namespace legged {

class LeggedRos2Control{
public:
  LeggedRos2Control(rclcpp::Node::SharedPtr &node);
  ~LeggedRos2Control();

  void init();

  virtual void update(const rclcpp::Time &time, const rclcpp::Duration &period);

protected:
  std::string urdf_string_;
  std::string get_robot_description_();

  virtual void import_components_(std::vector<hardware_interface::HardwareInfo> &hardware_info, 
                        std::unique_ptr<hardware_interface::ResourceManager> &resource_manager);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  std::shared_ptr<pluginlib::ClassLoader<LeggedSystemInterface>> system_interface_loader_;

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  rclcpp::Executor::SharedPtr cm_executor_;
  std::thread cm_thread_;
  std::thread spin_thread_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

};




} // namespace legged