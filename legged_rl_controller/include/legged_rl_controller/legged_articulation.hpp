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

#include "legged_rl_controller/isaaclab/assets/articulation/articulation.h"
#include "semantic_components/imu_sensor.hpp"
#include "legged_ros2_controller/semantic_components/joint_interface.hpp"

#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace legged{

using TwistMsgSharedPtr = std::shared_ptr<geometry_msgs::msg::Twist>;
using CmdBuffer = realtime_tools::RealtimeBuffer<TwistMsgSharedPtr>;

class LeggedArticulation : public isaaclab::Articulation
{
public:
  LeggedArticulation(std::shared_ptr<semantic_components::IMUSensor> imu_interface,
                     std::shared_ptr<JointInterface> joint_interface, 
                     std::shared_ptr<CmdBuffer> cmd_vel_buffer)
    : imu_interface_(std::move(imu_interface)), joint_interface_(std::move(joint_interface)), cmd_vel_buffer_(std::move(cmd_vel_buffer))
  {}

  void update() override {
    // update joint data
    auto joint_pos_double = joint_interface_->get_joint_position();
    auto joint_vel_double = joint_interface_->get_joint_velocity();

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

    // Update Command
    TwistMsgSharedPtr cmd_vel_msg = *cmd_vel_buffer_->readFromRT();
    if(cmd_vel_msg == nullptr){
      data.command.lin_vel_x = 0.0;
      data.command.lin_vel_y = 0.0;
      data.command.ang_vel_z = 0.0;
    } else {
      data.command.lin_vel_x = cmd_vel_msg->linear.x;
      data.command.lin_vel_y = cmd_vel_msg->linear.y;
      data.command.ang_vel_z = cmd_vel_msg->angular.z;
    }
  }

private:
  std::shared_ptr<semantic_components::IMUSensor> imu_interface_;
  std::shared_ptr<JointInterface> joint_interface_;
  std::shared_ptr<CmdBuffer> cmd_vel_buffer_;
};

}

