/**
 * @file legged_articulation.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>

#include "semantic_components/imu_sensor.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "legged_rl_controller/isaaclab/assets/articulation/articulation.h"
#include "legged_ros2_controller/semantic_components/joint_interface.hpp"


namespace legged{

using WaypointsMsgSharedPtr = std::shared_ptr<geometry_msgs::msg::PoseArray>;
using WaypointsBuffer = realtime_tools::RealtimeBuffer<WaypointsMsgSharedPtr>;

class LeggedArticulation : public isaaclab::Articulation
{
public:
  LeggedArticulation(std::shared_ptr<semantic_components::IMUSensor> imu_interface,
                     std::shared_ptr<JointInterface> joint_interface, 
                     std::shared_ptr<WaypointsBuffer> waypoints_buffer)
    : imu_interface_(std::move(imu_interface)),
      joint_interface_(std::move(joint_interface)),
      waypoints_buffer_(std::move(waypoints_buffer))
  {}

  void update() override {
    if (!joint_interface_ || !imu_interface_ || !waypoints_buffer_) {
      return;
    }
    // update joint data
    auto joint_pos_double = joint_interface_->get_joint_position();
    auto joint_vel_double = joint_interface_->get_joint_velocity();
    const size_t expected_joints = data.joint_names.size();
    if (expected_joints != 0U &&
        (joint_pos_double.size() != expected_joints || joint_vel_double.size() != expected_joints)) {
      return;
    }
    if (expected_joints == 0U) {
      return;
    }

    // Map the std::vector<double> to an Eigen::Map<Eigen::VectorXd> without copying
    // Then cast the entire vector from double to float and assign it.
    data.joint_pos = Eigen::Map<const Eigen::VectorXd>(joint_pos_double.data(), joint_pos_double.size()).cast<float>();
    data.joint_vel = Eigen::Map<const Eigen::VectorXd>(joint_vel_double.data(), joint_vel_double.size()).cast<float>();

    // Update IMU data
    
    // base angular velocity
    std::array<double, 3> ang_vel = imu_interface_->get_angular_velocity();
    data.root_ang_vel_b = Eigen::Vector3f(ang_vel[0], ang_vel[1], ang_vel[2]);
    
    // quaternion orientation
    std::array<double, 4> quat = imu_interface_->get_orientation();  // (x,y,z,w)
    Eigen::Quaternionf q(quat[3], quat[0], quat[1], quat[2]); // w,x,y,z
    data.projected_gravity_b = q.conjugate() * data.GRAVITY_VEC_W;
    data.root_quat = q;

    // Update waypoint command from PoseArray (flattened [x, y, z] * num_waypoints).
    auto & waypoint_data = data.waypoint_command;
    if (waypoint_data.path_command.size() != waypoint_data.generated_command_dim) {
      waypoint_data.path_command.assign(waypoint_data.generated_command_dim, 0.0f);
    }

    WaypointsMsgSharedPtr waypoints_msg = *waypoints_buffer_->readFromRT();
    if (waypoints_msg == nullptr) {
      std::fill(waypoint_data.path_command.begin(), waypoint_data.path_command.end(), 0.0f);
    } else {
      const auto expected_waypoints = waypoint_data.num_waypoints;
      if (waypoints_msg->poses.size() != expected_waypoints) {
        throw std::runtime_error(
          "Waypoint size mismatch: expected " + std::to_string(expected_waypoints) +
          ", got " + std::to_string(waypoints_msg->poses.size()) + ".");
      }

      for (size_t i = 0; i < expected_waypoints; ++i) {
        const auto & p = waypoints_msg->poses[i].position;
        const size_t base = i * 3U;
        waypoint_data.path_command[base + 0U] = static_cast<float>(p.x);
        waypoint_data.path_command[base + 1U] = static_cast<float>(p.y);
        waypoint_data.path_command[base + 2U] = static_cast<float>(p.z);
      }
    }

    /*
    // Disabled: legacy cmd_vel command pipeline.
    TwistMsgSharedPtr cmd_vel_msg = *cmd_vel_buffer_->readFromRT();
    if(cmd_vel_msg == nullptr){
      data.velocity_command.lin_vel_x = 0.0;
      data.velocity_command.lin_vel_y = 0.0;
      data.velocity_command.ang_vel_z = 0.0;
    } else {
      data.velocity_command.lin_vel_x = cmd_vel_msg->linear.x;
      data.velocity_command.lin_vel_y = cmd_vel_msg->linear.y;
      data.velocity_command.ang_vel_z = cmd_vel_msg->angular.z;
    }
    */
  }

private:
  std::shared_ptr<semantic_components::IMUSensor> imu_interface_;
  std::shared_ptr<JointInterface> joint_interface_;
  std::shared_ptr<WaypointsBuffer> waypoints_buffer_;
};

}
