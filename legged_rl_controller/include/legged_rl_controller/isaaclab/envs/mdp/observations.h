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

REGISTER_OBSERVATION(jump_phase)
{
    std::vector<float> obs(1);
    
    if(!env->cfg.has_extra("jump_duration")) {
        throw std::runtime_error("Observation term 'jump_phase' requires 'jump_duration' in env cfg extras.");
    }

    if(!env->robot->data.command.jump_cmd) {
        obs[0] = 0.0f;
        return obs;
    }

    float jump_duration = env->cfg.get_extra<double>("jump_duration").value_or(1.0);
    if(env->robot->data.command.jump_cmd && !env->robot->data.command.last_jump_cmd) {
        std::cout << "Jump command detected at time: " << env->episode_length_s << "s" << std::endl;
        env->robot->data.command.start_jump_time = env->episode_length_s;
    }

    float phase = (env->episode_length_s - env->robot->data.command.start_jump_time) / jump_duration;
    obs[0] = std::clamp(phase, 0.0f, 1.0f);

    return obs;
}

REGISTER_OBSERVATION(jump_distance)
{
    std::vector<float> obs(1);
    obs[0] = env->robot->data.command.jump_distance;
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