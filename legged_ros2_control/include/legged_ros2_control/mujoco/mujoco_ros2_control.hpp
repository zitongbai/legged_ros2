/**
 * @file mujoco_ros2_control.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "legged_ros2_control/legged_ros2_control.hpp"
#include "legged_ros2_control/mujoco/mujoco_system_interface.hpp"


namespace legged {

class MujocoRos2Control : public LeggedRos2Control {

public:
  MujocoRos2Control(rclcpp::Node::SharedPtr &node, mjModel *mujoco_model, mjData *mujoco_data) 
      : LeggedRos2Control(node), mj_model_(mujoco_model), mj_data_(mujoco_data) {}

  void update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:

  void import_components_(std::vector<hardware_interface::HardwareInfo> &hardware_info, 
                        std::unique_ptr<hardware_interface::ResourceManager> &resource_manager) override;

  std::shared_ptr<pluginlib::ClassLoader<MujocoSystemInterface>> system_interface_loader_;


  mjModel *mj_model_;
  mjData *mj_data_;

};

}