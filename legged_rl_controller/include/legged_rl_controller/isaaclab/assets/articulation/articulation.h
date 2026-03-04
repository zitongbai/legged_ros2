/**
 * @file articulation.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief Articulation base class
 * @ref This file is adapted from unitree_rl_lab.
 * @version 0.1
 * @date 2026-01-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <eigen3/Eigen/Dense>

namespace isaaclab
{

struct ArticulationData
{
    Eigen::Vector3f GRAVITY_VEC_W = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f FORWARD_VEC_B = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

    std::vector<std::string> joint_names; // lab order

    std::vector<float> joint_stiffness; // lab order
    std::vector<float> joint_damping; // lab order

    // Joint positions of all joints.
    Eigen::VectorXf joint_pos; // lab order
    
    // Default joint positions of all joints.
    Eigen::VectorXf default_joint_pos; // lab order

    // Default joint velocities of all joints.
    Eigen::VectorXf default_joint_vel; // lab order

    // Joint velocities of all joints.
    Eigen::VectorXf joint_vel; // lab order

    // Root angular velocity in base frame.
    Eigen::Vector3f root_ang_vel_b;

    // Projection of the gravity direction on base frame.
    Eigen::Vector3f projected_gravity_b;

    // Root orientation quaternion (w,x,y,z)
    Eigen::Quaternionf root_quat;

    // Velocity command
    struct VelocityCommand{
        float lin_vel_x = 0.0f;
        float lin_vel_y = 0.0f;
        float ang_vel_z = 0.0f;
        struct Range{
            std::array<float, 2U> lin_vel_x = {0.0f, 0.0f};
            std::array<float, 2U> lin_vel_y = {0.0f, 0.0f};
            std::array<float, 2U> ang_vel_z = {0.0f, 0.0f};
        } range;
    } velocity_command;

    // Waypoint command (flattened [x, y, z] * num_waypoints)
    struct WaypointCommand{
        size_t num_waypoints = 0U;
        size_t generated_command_dim = 0U;
        std::vector<float> path_command;
    } waypoint_command;

};

class Articulation
{
public:
    Articulation(){}

    virtual void update(){};

    ArticulationData data;
};

};
