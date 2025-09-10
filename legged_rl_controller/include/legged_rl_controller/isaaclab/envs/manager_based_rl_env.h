// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "legged_rl_controller/isaaclab/manager/observation_manager.h"
#include "legged_rl_controller/isaaclab/manager/action_manager.h"
#include "legged_rl_controller/isaaclab/assets/articulation/articulation.h"
#include "legged_rl_controller/isaaclab/algorithms/algorithms.h"
#include <iostream>
#include <unordered_map>
#include <any>
#include <optional>
#include <stdexcept>

namespace isaaclab
{
class ObservationManager;
class ActionManager;

struct ManagerBasedRLEnvCfg{
    std::string policy_net_path;
    std::vector<std::unique_ptr<ActionConfig>> action_cfgs;
    std::vector<std::unique_ptr<ObservationTermCfg>> observation_cfgs;
    std::vector<float> default_joint_pos;
    std::vector<float> joint_stiffness;
    std::vector<float> joint_damping;
    struct CommandVelocityRange {
        std::array<float, 2U> lin_vel_x = {0.0f, 0.0f};
        std::array<float, 2U> lin_vel_y = {0.0f, 0.0f};
        std::array<float, 2U> ang_vel_z = {0.0f, 0.0f};
    } command_velocity_range;

    // 新增：扩展属性包
    std::unordered_map<std::string, std::any> extras;

    // 设置扩展值
    template<typename T>
    void set_extra(const std::string &key, T value) {
        extras[key] = std::any(std::move(value));
    }

    // 获取扩展值（如果不存在或类型不匹配返回 std::nullopt）
    template<typename T>
    std::optional<T> get_extra(const std::string &key) const {
        auto it = extras.find(key);
        if (it == extras.end()) return std::nullopt;
        try {
            return std::any_cast<T>(it->second);
        } catch (const std::bad_any_cast&) {
            return std::nullopt;
        }
    }

    // 是否存在扩展键
    bool has_extra(const std::string &key) const {
        return extras.find(key) != extras.end();
    }

    // 删除扩展键
    void remove_extra(const std::string &key) {
        extras.erase(key);
    }

};

class ManagerBasedRLEnv
{
public:
    // Constructor
    ManagerBasedRLEnv(ManagerBasedRLEnvCfg cfg_, std::shared_ptr<Articulation> robot_)
    : cfg(std::move(cfg_)), robot(std::move(robot_))
    {
        // Parse configuration
        robot->data.joint_pos.resize(cfg.default_joint_pos.size());
        robot->data.joint_vel.resize(cfg.default_joint_pos.size());

        { // default joint positions
            robot->data.default_joint_pos = Eigen::VectorXf::Map(cfg.default_joint_pos.data(), cfg.default_joint_pos.size());
        }
        { // joint stiffness and damping
            robot->data.joint_stiffness = cfg.joint_stiffness;
            robot->data.joint_damping = cfg.joint_damping;
        }
        { // command velocity ranges
            robot->data.command.range.lin_vel_x = cfg.command_velocity_range.lin_vel_x;
            robot->data.command.range.lin_vel_y = cfg.command_velocity_range.lin_vel_y;
            robot->data.command.range.ang_vel_z = cfg.command_velocity_range.ang_vel_z;
        }

        // load managers
        action_manager = std::make_unique<ActionManager>(std::move(cfg.action_cfgs), this);
        observation_manager = std::make_unique<ObservationManager>(std::move(cfg.observation_cfgs), this);

        // load algorithm
        if (!cfg.policy_net_path.empty()) {
            alg = std::make_unique<OrtRunner>(cfg.policy_net_path);
        } else {
            std::cerr << "Warning: No policy network path provided, using default algorithm." << std::endl;
        }
    }

    void reset()
    {
        robot->update();
        global_phase = 0;
        episode_length = 0;
        episode_length_s = 0;
        action_manager->reset();
        observation_manager->reset();
    }

    void step(double period)
    {
        episode_length += 1;
        episode_length_s += period;
        robot->update();
        auto obs = observation_manager->compute();
        auto action = alg->act(obs);
        action_manager->process_action(action);
    }

    ManagerBasedRLEnvCfg cfg;
    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    long episode_length = 0;
    double episode_length_s = 0;
    float global_phase = 0.0f;
};

};