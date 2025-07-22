/**
 * @file separate_joint_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "legged_ros2_controller/separate_joint_controller.hpp"

namespace legged
{

controller_interface::CallbackReturn SeparateJointController::on_init(){
  if (LeggedController::on_init() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  kp_ = auto_declare<std::vector<double>>("kp", std::vector<double>());
  kd_ = auto_declare<std::vector<double>>("kd", std::vector<double>());

  if (kp_.size() != joint_names_.size() || kd_.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Size of kp or kd does not match the number of joints.");
    return controller_interface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SeparateJointController::on_configure(
    const rclcpp_lifecycle::State & previous_state){
  
  if (LeggedController::on_configure(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  auto joint_msg_callback = [this](const JointStateMsgSharedPtr msg) {

    auto ref_msg = std::make_shared<sensor_msgs::msg::JointState>();
    
    ref_msg->name.resize(joint_names_.size());
    ref_msg->position.resize(joint_names_.size());
    ref_msg->velocity.resize(joint_names_.size());
    ref_msg->effort.resize(joint_names_.size());

    for(size_t i=0; i<joint_names_.size(); ++i){
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it != msg->name.end()) {
        size_t index = std::distance(msg->name.begin(), it);
        ref_msg->position[i] = msg->position.at(index);
      } else {
        ref_msg->position[i] = 0.0; // Default value if joint not found
      }
    }
    joint_cmd_buffer_.writeFromNonRT(ref_msg);
  };

  joint_cmd_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_cmd", rclcpp::SystemDefaultsQoS(), joint_msg_callback);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SeparateJointController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

  joint_cmd_msg_ = *joint_cmd_buffer_.readFromRT();

  if (joint_cmd_msg_ == nullptr) {
    joint_cmd_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
    joint_cmd_msg_->name.resize(joint_names_.size());
    joint_cmd_msg_->position.resize(joint_names_.size(), 0.0);
    joint_cmd_msg_->velocity.resize(joint_names_.size(), 0.0);
    joint_cmd_msg_->effort.resize(joint_names_.size(), 0.0);
  }

  try {
    joint_interface_->set_joint_command(
      joint_cmd_msg_->position, joint_cmd_msg_->velocity, joint_cmd_msg_->effort,
      kp_, kd_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in set_joint_command: %s", e.what());
    return controller_interface::return_type::ERROR;
  }

  // // debug print
  // for(size_t i = 0; i < joint_names_.size(); ++i) {
  //   RCLCPP_INFO(
  //     get_node()->get_logger(),
  //     "Joint %s: pos_cmd=%.2f, vel_cmd=%.2f, ff_cmd=%.2f, kp=%.2f, kd=%.2f",
  //     joint_names_[i].c_str(),
  //     joint_cmd_msg_->position[i],
  //     joint_cmd_msg_->velocity[i],
  //     joint_cmd_msg_->effort[i],
  //     kp_[i],
  //     kd_[i]);
  // }

  return controller_interface::return_type::OK;
}

} // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  legged::SeparateJointController, controller_interface::ControllerInterface)