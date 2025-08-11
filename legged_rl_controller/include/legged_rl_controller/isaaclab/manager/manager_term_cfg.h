// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <iostream>
#include <deque>
#include <vector>
#include <functional>

namespace isaaclab
{

class ManagerBasedRLEnv;

using ObsFunc = std::function<std::vector<float>(ManagerBasedRLEnv*)>;

struct ObservationTermCfg
{
    std::string name;
    ObsFunc func;
    std::vector<float> clip;
    float scale = 1.0f; // default scale is 1.0
    size_t history_length = 1;

    void reset(std::vector<float> obs)
    {
        for(size_t i(0); i < history_length; ++i)
        {
            add(obs);
        }
    }

    void add(std::vector<float> obs)
    {
        buff_.push_back(obs);
        if (buff_.size() > history_length)
        {
            buff_.pop_front();
        }
    }

    /**
     * @brief Get the observation from the buffer, applying scaling and clipping if necessary.
     * Complete circular buffer with most recent entry at the end and oldest entry at the beginning.
     * 
     * @return std::vector<float> Scaled and clipped observation vector with history.
     */
    std::vector<float> get() {
        if (buff_.size() < history_length) {
            // Print warning.
            std::cerr << "Warning: Observation history buffer is not full. Expected size: "
                      << history_length << ", Current size: " << buff_.size() << std::endl;
            return {};
        }
        // Pre-allocate memory for the output vector to avoid reallocations.
        // Assumes all historical observations have the same size.
        const size_t obs_dim = buff_.front().size();
        std::vector<float> obs;
        obs.reserve(buff_.size() * obs_dim);

        // Iterate through the history buffer.
        for (const auto& history_obs : buff_) {
            for (size_t j = 0; j < history_obs.size(); ++j) {
                float val = history_obs[j];
                if (!clip.empty()) {
                    val = std::clamp(val, clip[0], clip[1]);
                }
                val *= scale;
                obs.push_back(val);
            }
        }

        return obs;
    }

private:
    std::deque<std::vector<float>> buff_;
};

};