/**
 * @file legged_jump_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include "legged_rl_controller/legged_jump_controller.hpp"


namespace legged{

controller_interface::CallbackReturn LeggedJumpController::on_configure(const rclcpp_lifecycle::State & previous_state) {
  if(LeggedJumpController::configure_parameters_() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (LeggedController::on_configure(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  jump_cmd_buffer_ = std::make_shared<JumpCmdBuffer>();
  jump_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      "jump_cmd", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
          std::cout << "Received jump command: " << msg->data << std::endl;
          auto buf = jump_cmd_buffer_;
          if (buf) {
            buf->writeFromNonRT(msg);
            std::cout << "Jump command written to buffer: " << msg->data << std::endl;
          } else {
            RCLCPP_WARN(get_node()->get_logger(), "jump_cmd_buffer_ is null, skipping");
          }
  });

  jump_dist_buffer_ = std::make_shared<JumpDistBuffer>();
  jump_dist_sub_ = get_node()->create_subscription<std_msgs::msg::Float32>(
      "jump_distance", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
          std::cout << "Received jump distance: " << msg->data << std::endl;
          auto buf = jump_dist_buffer_;
          if (buf) {
            buf->writeFromNonRT(msg);
            std::cout << "Jump distance written to buffer: " << msg->data << std::endl;
          } else {
            RCLCPP_WARN(get_node()->get_logger(), "jump_dist_buffer_ is null, skipping");
          }
  });

  // Assume we only have one IMU
  robot_ = std::make_shared<LeggedJumpArticulation>(imu_interfaces_[0], joint_interface_, jump_cmd_buffer_, jump_dist_buffer_);
  env_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(std::move(env_cfg_), robot_);

  RCLCPP_INFO(get_node()->get_logger(), "Legged Jump Controller configured successfully.");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedJumpController::on_activate(const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(get_node()->get_logger(), "Activating Legged Jump Controller...");
  if (LeggedController::on_activate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // // Reset command buffer
  // jump_cmd_buffer_.reset();

  // Reset env
  env_->reset();

  RCLCPP_INFO(get_node()->get_logger(), "Legged Jump Controller activated successfully.");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedJumpController::configure_parameters_(){
    if(LeggedRLController::configure_parameters_() != controller_interface::CallbackReturn::SUCCESS){
        return controller_interface::CallbackReturn::ERROR;
    }

  // -------------------------------------------
  // Other configurations
  // -------------------------------------------
  env_cfg_.set_extra<double>("jump_duration", params_.jump_duration);

  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  legged::LeggedJumpController, controller_interface::ControllerInterface)

