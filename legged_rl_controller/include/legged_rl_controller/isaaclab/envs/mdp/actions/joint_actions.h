// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "legged_rl_controller/isaaclab/envs/manager_based_rl_env.h"
#include "legged_rl_controller/isaaclab/manager/action_manager.h"

namespace isaaclab {

struct JointActionConfig : public ActionConfig {
    std::vector<float> scale;
    std::vector<float> offset;
    std::vector<std::vector<float>> clip;
};

class JointAction : public ActionTerm {
 public:
    JointAction(std::unique_ptr<JointActionConfig> cfg, ManagerBasedRLEnv* env)
            : ActionTerm(std::move(cfg), env),
                num_joints_(env->robot->data.default_joint_pos.size()),
                raw_actions_(num_joints_, 0.0f),
                processed_actions_(num_joints_, 0.0f) {

        auto * typed_cfg = static_cast<JointActionConfig*>(cfg_.get());
        scale_ = typed_cfg->scale;
        offset_ = typed_cfg->offset;
        if (!typed_cfg->clip.empty()) {
            clip_ = typed_cfg->clip;
        }
    }

    void process_actions(const std::vector<float>& actions) {
        raw_actions_ = actions;
        for (int i = 0; i < num_joints_; ++i) {
            processed_actions_[i] = raw_actions_[i] * scale_[i] + offset_[i];
        }
        if (!clip_.empty()) {
            for (int i = 0; i < num_joints_; ++i) {
                processed_actions_[i] =
                        std::clamp(processed_actions_[i], clip_[i][0], clip_[i][1]);
            }
        }
    }

    int action_dim() { return num_joints_; }

    std::vector<float> raw_actions() { return raw_actions_; }

    std::vector<float> processed_actions() { return processed_actions_; }

    void reset() { raw_actions_.assign(num_joints_, 0.0f); }

 protected:
    int num_joints_;

    std::vector<float> raw_actions_;
    std::vector<float> processed_actions_;

    std::vector<float> scale_;
    std::vector<float> offset_;
    std::vector<std::vector<float>> clip_;
};

class JointPositionAction : public JointAction {
public:
    JointPositionAction(std::unique_ptr<JointActionConfig> cfg, ManagerBasedRLEnv* env)
            : JointAction(std::move(cfg), env) {}
};

REGISTER_ACTION(JointPositionAction, JointActionConfig);

}  // namespace isaaclab
