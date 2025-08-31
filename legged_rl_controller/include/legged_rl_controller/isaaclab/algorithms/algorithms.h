// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <cstring>

namespace isaaclab {

class Algorithms {
 public:
    virtual ~Algorithms() = default;
    virtual std::vector<float> act(const std::vector<float>& obs) = 0;

    std::vector<float> get_action() {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action_;
    }

 protected:
    std::vector<float> action_;
    mutable std::mutex act_mtx_;
};

enum class RNNType {
    NONE,
    LSTM,
    GRU
};

class OrtRunner : public Algorithms {
 public:
    explicit OrtRunner(const std::string& model_path)
            : env_(ORT_LOGGING_LEVEL_WARNING, "onnx_model"), rnn_type_(RNNType::NONE) {
        session_options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

        // get the number of inputs and outputs
        num_inputs_ = session_->GetInputCount();
        num_outputs_ = session_->GetOutputCount();

        // get the rnn type based on number of inputs
        if (num_inputs_ == 1) {
            rnn_type_ = RNNType::NONE;
            input_names_ = {"obs"};
            output_names_ = {"actions"};
        } else if (num_inputs_ == 2) {
            rnn_type_ = RNNType::GRU;
            input_names_ = {"obs", "h_in"};
            output_names_ = {"actions", "h_out"};
        } else if (num_inputs_ == 3) {
            rnn_type_ = RNNType::LSTM;
            input_names_ = {"obs", "h_in", "c_in"};
            output_names_ = {"actions", "h_out", "c_out"};
        } else {
            throw std::runtime_error("Unsupported model input configuration");
        }

        // get the shapes of each input and output
        input_shapes_.resize(num_inputs_);
        output_shapes_.resize(num_outputs_);
        
        for (size_t i = 0; i < num_inputs_; ++i) {
            Ort::TypeInfo input_type = session_->GetInputTypeInfo(i);
            input_shapes_[i] = input_type.GetTensorTypeAndShapeInfo().GetShape();
        }
        
        for (size_t i = 0; i < num_outputs_; ++i) {
            Ort::TypeInfo output_type = session_->GetOutputTypeInfo(i);
            output_shapes_[i] = output_type.GetTensorTypeAndShapeInfo().GetShape();
        }

        // initialize action vector
        action_.resize(output_shapes_[0][1]); 

        // initialize hidden state
        if (rnn_type_ == RNNType::GRU) {
            h_state_.resize(input_shapes_[1][0] * input_shapes_[1][1] * input_shapes_[1][2], 0.0f);
        } else if (rnn_type_ == RNNType::LSTM) {
            h_state_.resize(input_shapes_[1][0] * input_shapes_[1][1] * input_shapes_[1][2], 0.0f);
            c_state_.resize(input_shapes_[2][0] * input_shapes_[2][1] * input_shapes_[2][2], 0.0f);
        }
    }

    std::vector<float> act(const std::vector<float>& obs) override {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        
        std::vector<Ort::Value> input_tensors;
        std::vector<const char*> input_names_ptr;
        std::vector<const char*> output_names_ptr;

        // prepare input
        if (rnn_type_ == RNNType::NONE) {
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info, const_cast<float*>(obs.data()), obs.size(),
                input_shapes_[0].data(), input_shapes_[0].size()));
            input_names_ptr = {"obs"};
            output_names_ptr = {"actions"};
        } else if (rnn_type_ == RNNType::GRU) {
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info, const_cast<float*>(obs.data()), obs.size(),
                input_shapes_[0].data(), input_shapes_[0].size()));
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info, h_state_.data(), h_state_.size(),
                input_shapes_[1].data(), input_shapes_[1].size()));
            input_names_ptr = {"obs", "h_in"};
            output_names_ptr = {"actions", "h_out"};
        } else if (rnn_type_ == RNNType::LSTM) {
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info, const_cast<float*>(obs.data()), obs.size(),
                input_shapes_[0].data(), input_shapes_[0].size()));
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info, h_state_.data(), h_state_.size(),
                input_shapes_[1].data(), input_shapes_[1].size()));
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info, c_state_.data(), c_state_.size(),
                input_shapes_[2].data(), input_shapes_[2].size()));
            input_names_ptr = {"obs", "h_in", "c_in"};
            output_names_ptr = {"actions", "h_out", "c_out"};
        }

        // run inference
        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr}, input_names_ptr.data(), input_tensors.data(), 
            input_tensors.size(), output_names_ptr.data(), output_names_ptr.size());

        // get actions
        auto actions_data = output_tensors[0].GetTensorMutableData<float>();

        std::lock_guard<std::mutex> lock(act_mtx_);
        std::memcpy(action_.data(), actions_data, output_shapes_[0][1] * sizeof(float));

        // update hidden state
        if (rnn_type_ == RNNType::GRU) {
            auto h_out_data = output_tensors[1].GetTensorMutableData<float>();
            std::memcpy(h_state_.data(), h_out_data, h_state_.size() * sizeof(float));
        } else if (rnn_type_ == RNNType::LSTM) {
            auto h_out_data = output_tensors[1].GetTensorMutableData<float>();
            auto c_out_data = output_tensors[2].GetTensorMutableData<float>();
            std::memcpy(h_state_.data(), h_out_data, h_state_.size() * sizeof(float));
            std::memcpy(c_state_.data(), c_out_data, c_state_.size() * sizeof(float));
        }

        return action_;
    }

    // reset hidden states
    void reset_hidden_states() {
        if (rnn_type_ != RNNType::NONE) {
            std::fill(h_state_.begin(), h_state_.end(), 0.0f);
            if (rnn_type_ == RNNType::LSTM) {
                std::fill(c_state_.begin(), c_state_.end(), 0.0f);
            }
        }
    }

 private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;

    RNNType rnn_type_;
    size_t num_inputs_;
    size_t num_outputs_;
    
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    
    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::vector<int64_t>> output_shapes_;
    
    // hidden states for RNNs
    std::vector<float> h_state_;  // GRU and LSTM hidden states
    std::vector<float> c_state_;  // LSTM cell state
};

}  // namespace isaaclab