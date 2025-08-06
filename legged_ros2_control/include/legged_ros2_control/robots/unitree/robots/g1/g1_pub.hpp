// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include "legged_ros2_control/robots/unitree/dds_wrapper/common/Publisher.h"
#include "legged_ros2_control/robots/unitree/dds_wrapper/common/crc.h"

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/go2/MotorCmds_.hpp>


namespace unitree {
namespace robot {
namespace g1 {

class LowCmdPublisher : public RealTimePublisher<unitree_hg::msg::dds_::LowCmd_>{
public:
  LowCmdPublisher(std::string topic = "rt/lowcmd") : RealTimePublisher<MsgType>(topic) 
  {
    msg_.mode_pr() = static_cast<uint8_t>(Mode::PR);
    for(auto & m : msg_.motor_cmd()) m.mode(1);
  }

private:
    /**
     * @brief Something before sending the message.
     */
    void pre_communication() override {
        msg_.crc() = crc32_core((uint32_t*)&msg_, (sizeof(MsgType)>>2)-1);
    }
};


} // namespace g1
} // namespace robot
} // namespace unitree
