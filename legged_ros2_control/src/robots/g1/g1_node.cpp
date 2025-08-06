/**
 * @file g1_node.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "legged_ros2_control/legged_ros2_control.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("g1_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  RCLCPP_INFO(node->get_logger(), "Initializing G1 node...");

  // Create LeggedRos2Control instance
  legged::LeggedRos2Control legged_control(node);

  legged_control.init();  // thread of controller manager and its spin is started here

  while(rclcpp::ok())
  {
    rclcpp::sleep_for(std::chrono::milliseconds(50));  // Sleep to avoid busy-waiting
  }
  
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

