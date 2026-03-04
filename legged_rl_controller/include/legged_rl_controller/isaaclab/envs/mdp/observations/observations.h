/**
 * @file observations.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "legged_rl_controller/isaaclab/envs/manager_based_rl_env.h"

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(base_ang_vel)
{
    (void)params;  // Reserved for per-observation config.
    auto & asset = env->robot;
    auto & data = asset->data.root_ang_vel_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(projected_gravity)
{
    (void)params;  // Reserved for per-observation config.
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(joint_pos)
{
    (void)params;  // Reserved for per-observation config.
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_pos.size());
    for (Eigen::Index i = 0; i < asset->data.joint_pos.size(); ++i) {
        data[i] = asset->data.joint_pos[i];
    }

    return data;
}

REGISTER_OBSERVATION(joint_pos_rel)
{
    (void)params;  // Reserved for per-observation config.
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_pos.size());
    for (Eigen::Index i = 0; i < asset->data.joint_pos.size(); ++i) {
        data[i] = asset->data.joint_pos[i] - asset->data.default_joint_pos[i];
    }

    return data;
}

REGISTER_OBSERVATION(joint_vel)
{
    (void)params;  // Reserved for per-observation config.
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_vel.size());
    for (Eigen::Index i = 0; i < asset->data.joint_vel.size(); ++i) {
        data[i] = asset->data.joint_vel[i];
    }
    
    return data;
}

REGISTER_OBSERVATION(joint_vel_rel)
{
    (void)params;  // Reserved for per-observation config.
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_vel.size());
    for (Eigen::Index i = 0; i < asset->data.joint_vel.size(); ++i) {
        data[i] = asset->data.joint_vel[i] - asset->data.default_joint_vel[i];
    }

    return data;
}

REGISTER_OBSERVATION(last_action)
{
    (void)params;  // Reserved for per-observation config.
    auto data = env->action_manager->action();
    return std::vector<float>(data.data(), data.data() + data.size());
};

REGISTER_OBSERVATION(generated_commands)
{
    (void)params;  // Reserved for per-observation config.
    return env->robot->data.waypoint_command.path_command;
}

/*
// Disabled: legacy cmd_vel-based generated_commands implementation.
REGISTER_OBSERVATION(generated_commands)
{
    (void)params;  // Reserved for per-observation config.
    std::vector<float> obs(3);
    auto & data = env->robot->data;

    obs[0] = std::clamp(data.velocity_command.lin_vel_x, data.velocity_command.range.lin_vel_x[0], data.velocity_command.range.lin_vel_x[1]);
    obs[1] = std::clamp(data.velocity_command.lin_vel_y, data.velocity_command.range.lin_vel_y[0], data.velocity_command.range.lin_vel_y[1]);
    obs[2] = std::clamp(data.velocity_command.ang_vel_z, data.velocity_command.range.ang_vel_z[0], data.velocity_command.range.ang_vel_z[1]);

    return obs;
}
*/


}
}
