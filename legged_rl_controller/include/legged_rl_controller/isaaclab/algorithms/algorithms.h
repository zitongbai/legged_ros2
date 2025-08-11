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

class OrtRunner : public Algorithms {
 public:
    explicit OrtRunner(const std::string& model_path)
            : env_(ORT_LOGGING_LEVEL_WARNING, "onnx_model") {
        session_options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

        Ort::TypeInfo input_type = session_->GetInputTypeInfo(0);
        input_shape_ = input_type.GetTensorTypeAndShapeInfo().GetShape();
        Ort::TypeInfo output_type = session_->GetOutputTypeInfo(0);
        output_shape_ = output_type.GetTensorTypeAndShapeInfo().GetShape();

        action_.resize(output_shape_[1]);
    }

    std::vector<float> act(const std::vector<float>& obs) override {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        auto input_tensor = Ort::Value::CreateTensor<float>(
                memory_info, const_cast<float*>(obs.data()), obs.size(),
                input_shape_.data(), input_shape_.size());
        auto output_tensor = session_->Run(
                Ort::RunOptions{nullptr}, input_names_.data(), &input_tensor, 1,
                output_names_.data(), 1);
        auto floatarr = output_tensor.front().GetTensorMutableData<float>();

        std::lock_guard<std::mutex> lock(act_mtx_);
        std::memcpy(action_.data(), floatarr, output_shape_[1] * sizeof(float));
        return action_;
    }

 private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;

    const std::vector<const char*> input_names_ = {"obs"};
    const std::vector<const char*> output_names_ = {"actions"};

    std::vector<int64_t> input_shape_;
    std::vector<int64_t> output_shape_;
};

}  // namespace isaaclab