// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "legged_rl_controller/isaaclab/envs/manager_based_rl_env.h"
#include "legged_rl_controller/isaaclab/manager/manager_term_cfg.h"
#include <numeric>

namespace isaaclab {

struct ActionConfig {
    std::string name;
    virtual ~ActionConfig() = default;
};

class ActionTerm {
 public:
    ActionTerm(std::unique_ptr<ActionConfig> cfg, ManagerBasedRLEnv* env) : cfg_(std::move(cfg)), env_(env) {}

    virtual int action_dim() = 0;
    virtual std::vector<float> raw_actions() = 0;
    virtual std::vector<float> processed_actions() = 0;
    virtual void process_actions(const std::vector<float>& actions) = 0;
    virtual void reset() {}

 protected:
    std::unique_ptr<ActionConfig> cfg_;
    ManagerBasedRLEnv* env_;
};

inline std::map<std::string, std::function<std::unique_ptr<ActionTerm>(std::unique_ptr<ActionConfig>, ManagerBasedRLEnv*)>>& ActionsMap() {
    static std::map<std::string, std::function<std::unique_ptr<ActionTerm>(std::unique_ptr<ActionConfig>, ManagerBasedRLEnv*)>> instance;
    return instance;
}

#define REGISTER_ACTION(name, config_type) \
    inline struct name##_registrar { \
        name##_registrar() { \
            ActionsMap()[#name] = [](std::unique_ptr<ActionConfig> cfg, ManagerBasedRLEnv* env) { \
                auto typed_cfg = std::unique_ptr<config_type>(dynamic_cast<config_type*>(cfg.release())); \
                if (!typed_cfg) { \
                    std::cerr << "Error: Invalid config type for action term '" #name "'" << std::endl; \
                    throw std::runtime_error("Invalid config type for action term '" #name "'"); \
                } \
                return std::make_unique<name>(std::move(typed_cfg), env); \
            }; \
        } \
    } name##_registrar_instance;

class ActionManager {
 public:
    ActionManager(std::vector<std::unique_ptr<ActionConfig>> cfgs, ManagerBasedRLEnv* env)
            : env_(env) {
        prapare_terms_(std::move(cfgs));
        action_.resize(total_action_dim(), 0.0f);
    }

    void reset() {
        action_.assign(total_action_dim(), 0.0f);
        for (auto& term : terms_) {
            term->reset();
        }
    }

    std::vector<float> action() const {
        return action_;
    }

    std::vector<float> processed_actions() const {
        std::vector<float> actions;
        for (const auto& term : terms_) {
            auto term_action = term->processed_actions();
            actions.insert(actions.end(), term_action.begin(), term_action.end());
        }
        return actions;
    }

    void process_action(const std::vector<float>& action) {
        action_ = action;
        int idx = 0;
        for (auto& term : terms_) {
            auto term_action = std::vector<float>(action.begin() + idx, action.begin() + idx + term->action_dim());
            term->process_actions(term_action);
            idx += term->action_dim();
        }
    }

    int total_action_dim() const {
        auto dims = action_dim();
        return std::accumulate(dims.begin(), dims.end(), 0);
    }

    std::vector<int> action_dim() const {
        std::vector<int> dims;
        for (const auto& term : terms_) {
            dims.push_back(term->action_dim());
        }
        return dims;
    }

    ManagerBasedRLEnv* env_;

 private:
    void prapare_terms_(std::vector<std::unique_ptr<ActionConfig>> cfgs) {
        for (auto& cfg : cfgs) {
            const std::string& action_name = cfg->name;
            std::cout << "Preparing action term: " << action_name << std::endl; // 添加调试信息
            if (ActionsMap().find(action_name) == ActionsMap().end()) {
                std::cerr << "Error: Action term '" << action_name << "' is not registered." << std::endl;
                throw std::runtime_error("Action term '" + action_name + "' is not registered.");
            }
            auto term = ActionsMap()[action_name](std::move(cfg), env_);
            terms_.push_back(std::move(term));
            std::cout << "Finished preparing action term: " << action_name << std::endl; // 添加调试信息
        }
    }

    std::vector<float> action_;
    std::vector<std::unique_ptr<ActionTerm>> terms_;
};

}  // namespace isaaclab
