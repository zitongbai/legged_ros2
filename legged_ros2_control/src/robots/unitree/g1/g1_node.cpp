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
#include "legged_ros2_control/robots/unitree/robots/g1/g1_sub.hpp"
#include "legged_ros2_control/robots/unitree/robots/unitree_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<legged::UnitreeNode<unitree::robot::g1::LowStateSubscriber>>();
  
  node->run();  // Start the node's run loop
  
  return EXIT_SUCCESS;
}

