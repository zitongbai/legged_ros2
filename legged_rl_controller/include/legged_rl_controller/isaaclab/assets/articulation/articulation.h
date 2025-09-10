// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>

namespace isaaclab
{

/**
 * @brief Articulation data structure
 * Note that all its joint-related data's order should align with that in IsaacLab
 */
struct ArticulationData
{
    Eigen::Vector3f GRAVITY_VEC_W = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f FORWARD_VEC_B = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

    std::vector<float> joint_stiffness; 
    std::vector<float> joint_damping;

    // Joint positions of all joints.
    Eigen::VectorXf joint_pos;
    
    // Default joint positions of all joints.
    Eigen::VectorXf default_joint_pos;

    // Joint velocities of all joints.
    Eigen::VectorXf joint_vel;

    // Root angular velocity in base world frame.
    Eigen::Vector3f root_ang_vel_b;

    // Projection of the gravity direction on base frame.
    Eigen::Vector3f projected_gravity_b;

    // Command
    struct Command{
        float lin_vel_x = 0.0f;
        float lin_vel_y = 0.0f;
        float ang_vel_z = 0.0f;
        struct Range{
            std::array<float, 2U> lin_vel_x = {0.0f, 0.0f};
            std::array<float, 2U> lin_vel_y = {0.0f, 0.0f};
            std::array<float, 2U> ang_vel_z = {0.0f, 0.0f};
        } range;
        // only for jump task. TODO: create it other where
        float start_jump_time;
        bool jump_cmd;
        bool last_jump_cmd;
        float jump_distance;
    } command;
};

class Articulation
{
public:
    Articulation(){}

    virtual void update(){};

    ArticulationData data;
};

};