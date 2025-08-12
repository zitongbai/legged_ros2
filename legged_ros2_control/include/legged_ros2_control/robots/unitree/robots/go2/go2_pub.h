// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "legged_ros2_control/robots/unitree/dds_wrapper/common/Publisher.h"
#include "legged_ros2_control/robots/unitree/dds_wrapper/common/crc.h"

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LidarState_.hpp>
#include <unitree/idl/go2/UwbState_.hpp>
#include <unitree/idl/go2/HeightMap_.hpp>
#include <unitree/idl/ros2/Time_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/go2/Res_.hpp>
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>


namespace unitree
{
namespace robot
{
namespace go2
{ 

class LowCmdPublisher : public RealTimePublisher<unitree_go::msg::dds_::LowCmd_>
{
public:
  LowCmdPublisher(std::string topic = "rt/lowcmd")
  : RealTimePublisher<MsgType>(topic) 
  {
    msg_.head() = {0xFE, 0xEF};
    msg_.level_flag() = 0xFF;
    msg_.gpio() = 0;

    for (auto & m : msg_.motor_cmd()){
       m.mode() = (0x01);   // motor switch to servo (PMSM) mode
       m.q() = 0.0f;
       m.dq() = 0.0f;
       m.tau() = 0.0f;
       m.kp() = 0.0f;
       m.kd() = 0.0f;
    }
  } 

private:
  /**
   * @brief Something before sending the message.
   */
  void pre_communication() override {
    msg_.crc() = crc32_core((uint32_t*)&msg_, (sizeof(MsgType)>>2)-1);
  }
};


} // namespace go2
} // namespace robot
} // namespace unitree