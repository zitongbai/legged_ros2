// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <unordered_set>
#include <iostream>

#include "legged_rl_controller/isaaclab/manager/manager_term_cfg.h"

namespace isaaclab
{

using ObsMap = std::map<std::string, ObsFunc>;

inline ObsMap& observations_map() {
    static ObsMap instance;
    return instance;
}

#define REGISTER_OBSERVATION(name) \
    inline std::vector<float> name(ManagerBasedRLEnv* env); \
    inline struct name##_registrar { \
        name##_registrar() { observations_map()[#name] = name; } \
    } name##_registrar_instance; \
    inline std::vector<float> name(ManagerBasedRLEnv* env)


class ObservationManager
{
public:
    using ObsTermCfgPtr = std::unique_ptr<ObservationTermCfg>;
    ObservationManager(const std::vector<ObsTermCfgPtr> & cfgs, ManagerBasedRLEnv* env)
    : env(env)
    {
        prepare_terms_(cfgs);
    }

    void reset()
    {
        for(auto & term : obs_term_cfgs)
        {
            term.reset(term.func(this->env));
        }
    }

    std::vector<float> compute()
    {
        std::vector<float> obs;
        for(auto & term : obs_term_cfgs)
        {
            term.add(term.func(this->env));
            auto term_obs_scaled = term.get();
            obs.insert(obs.end(), term_obs_scaled.begin(), term_obs_scaled.end());
        }
        return obs;
    }

protected:
    void prepare_terms_(const std::vector<ObsTermCfgPtr> & cfgs)
    {
        for(const auto & cfg : cfgs)
        {
            const auto& name = cfg->name;

            ObservationTermCfg term_cfg;
            term_cfg.history_length = cfg->history_length;
            if(term_cfg.history_length < 1){
                term_cfg.history_length = 1; // 0 history length means only use current observation
            }

            if(observations_map()[name] == nullptr)
            {
                throw std::runtime_error("Observation term '" + name + "' is not registered.");
            }
            term_cfg.func = observations_map()[name];

            auto obs = term_cfg.func(this->env);
            term_cfg.reset(obs);
            term_cfg.scale = cfg->scale;
            term_cfg.clip = cfg->clip;

            this->obs_term_cfgs.push_back(std::move(term_cfg));
        }
    }

    ManagerBasedRLEnv* env;
private:
    std::vector<ObservationTermCfg> obs_term_cfgs;
};

};