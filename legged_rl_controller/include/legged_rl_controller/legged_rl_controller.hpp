/**
 * @file legged_rl_controller.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <deque>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "legged_ros2_controller/legged_ros2_controller.hpp"

#include "legged_rl_controller/isaaclab/envs/manager_based_rl_env.h"
#include "legged_rl_controller/legged_articulation.hpp"



namespace legged{

class LeggedRLController : public LeggedController
{
public:

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_sub_;
  using WaypointsMsgSharedPtr = std::shared_ptr<geometry_msgs::msg::PoseArray>;
  using WaypointsBuffer = realtime_tools::RealtimeBuffer<WaypointsMsgSharedPtr>;
  std::shared_ptr<WaypointsBuffer> waypoints_buffer_;

  /*
  // Disabled: legacy cmd_vel command pipeline.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  using TwistMsgSharedPtr = std::shared_ptr<geometry_msgs::msg::Twist>;
  using CmdBuffer = realtime_tools::RealtimeBuffer<TwistMsgSharedPtr>;
  std::shared_ptr<CmdBuffer> cmd_vel_buffer_;
  TwistMsgSharedPtr cmd_vel_msg_;
  */

  std::unique_ptr<isaaclab::ManagerBasedRLEnv> env_;
  std::shared_ptr<isaaclab::Articulation> robot_;

  std::string onnx_model_path_;
  std::string io_descriptors_path_;
  double waypoint_first_xyz_norm_max_ = 0.5;

};

}
