// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "legged_rl_controller/isaaclab/envs/manager_based_rl_env.h"

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(base_ang_vel)
{
    auto & asset = env->robot;
    auto & data = asset->data.root_ang_vel_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(projected_gravity)
{
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(joint_pos)
{
    auto & asset = env->robot;
    std::vector<float> data;
    data.resize(asset->data.joint_pos.size());
    for(Eigen::Index i = 0; i < asset->data.joint_pos.size(); ++i)
    {
        data[i] = asset->data.joint_pos[i] - asset->data.default_joint_pos[i];
    }

    return data;
}

REGISTER_OBSERVATION(joint_vel)
{
    auto & asset = env->robot;
    auto & data = asset->data.joint_vel;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(actions)
{
    auto data = env->action_manager->action();
    return std::vector<float>(data.data(), data.data() + data.size());
};

REGISTER_OBSERVATION(velocity_commands)
{
    std::vector<float> obs(3);
    auto & data = env->robot->data;

    obs[0] = std::clamp(data.command.lin_vel_x, data.command.range.lin_vel_x[0], data.command.range.lin_vel_x[1]);
    obs[1] = std::clamp(data.command.lin_vel_y, data.command.range.lin_vel_y[0], data.command.range.lin_vel_y[1]);
    obs[2] = std::clamp(data.command.ang_vel_z, data.command.range.ang_vel_z[0], data.command.range.ang_vel_z[1]);

    return obs;
}

// REGISTER_OBSERVATION(gait_phase)
// {
//     float period = env->cfg["observations"]["gait_phase"]["params"]["period"].as<float>();
//     float delta_phase = env->step_dt * (1.0f / period);

//     env->global_phase += delta_phase;
//     env->global_phase = std::fmod(env->global_phase, 1.0f);

//     std::vector<float> obs(2);
//     obs[0] = std::sin(env->global_phase * 2 * M_PI);
//     obs[1] = std::cos(env->global_phase * 2 * M_PI);
//     return obs;
// }

}
}