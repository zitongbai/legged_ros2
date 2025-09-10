/**
 * @file legged_jump_articulation.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "legged_rl_controller/isaaclab/assets/articulation/articulation.h"
#include "semantic_components/imu_sensor.hpp"
#include "legged_ros2_controller/semantic_components/joint_interface.hpp"

#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace legged{

using BoolMsgSharedPtr = std::shared_ptr<std_msgs::msg::Bool>;
using JumpCmdBuffer = realtime_tools::RealtimeBuffer<BoolMsgSharedPtr>;
using Float32MsgSharedPtr = std::shared_ptr<std_msgs::msg::Float32>;
using JumpDistBuffer = realtime_tools::RealtimeBuffer<Float32MsgSharedPtr>;

class LeggedJumpArticulation: public isaaclab::Articulation{

public:
    LeggedJumpArticulation(std::shared_ptr<semantic_components::IMUSensor> imu_interface,
                         std::shared_ptr<JointInterface> joint_interface,
                         std::shared_ptr<JumpCmdBuffer> jump_cmd_buffer,
                         std::shared_ptr<JumpDistBuffer> jump_dist_buffer
                        )
        : imu_interface_(std::move(imu_interface)), 
        joint_interface_(std::move(joint_interface)), 
        jump_cmd_buffer_(std::move(jump_cmd_buffer)),
        jump_dist_buffer_(std::move(jump_dist_buffer))
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
        BoolMsgSharedPtr jump_cmd_msg = *jump_cmd_buffer_->readFromRT();
        if(jump_cmd_msg == nullptr){
            data.command.jump_cmd = false;
        } else {
            data.command.last_jump_cmd = data.command.jump_cmd;
            data.command.jump_cmd = jump_cmd_msg->data;
        }

        Float32MsgSharedPtr jump_dist_msg = *jump_dist_buffer_->readFromRT();
        if(jump_dist_msg == nullptr){
            data.command.jump_distance = 0.35f; // default jump distance
        } else {
            data.command.jump_distance = jump_dist_msg->data;
        }
    }


private:
    std::shared_ptr<semantic_components::IMUSensor> imu_interface_;
    std::shared_ptr<JointInterface> joint_interface_;
    std::shared_ptr<JumpCmdBuffer> jump_cmd_buffer_;
    std::shared_ptr<JumpDistBuffer> jump_dist_buffer_;
};

}