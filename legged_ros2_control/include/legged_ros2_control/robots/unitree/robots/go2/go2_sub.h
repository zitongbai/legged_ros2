// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <algorithm>
#include <unordered_map>
#include "legged_ros2_control/robots/unitree/dds_wrapper/common/Subscription.h"

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>


namespace unitree
{
namespace robot
{
namespace go2
{ 

class LowStateSubscriber : public SubscriptionBase<unitree_go::msg::dds_::LowState_>
{
public:
  using SharedPtr = std::shared_ptr<LowStateSubscriber>;

  LowStateSubscriber(std::string topic = "rt/lowstate") : SubscriptionBase<MsgType>(topic) {}

  void update() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // ********** Joystick ********** //
    // Check if all joystick values are zero to determine if the joystick is inactive
    if(std::all_of(msg_.wireless_remote().begin(), msg_.wireless_remote().end(), [](uint8_t i){return i == 0;}))
    {
      auto now = std::chrono::system_clock::now();
      auto elasped_time = now - last_joystick_time_;
      if(elasped_time > std::chrono::milliseconds(joystick_timeout_ms_))
      {
        isJoystickTimeout_ = true;
      }
    } else {
      last_joystick_time_ = std::chrono::system_clock::now();
      isJoystickTimeout_ = false;
    }

    // update joystick state
    unitree::common::REMOTE_DATA_RX key;
    memcpy(&key, &msg_.wireless_remote()[0], 40);
    joystick.back(key.RF_RX.btn.components.Select);
    joystick.start(key.RF_RX.btn.components.Start);
    joystick.LB(key.RF_RX.btn.components.L1);
    joystick.RB(key.RF_RX.btn.components.R1);
    joystick.F1(key.RF_RX.btn.components.f1);
    joystick.F2(key.RF_RX.btn.components.f2);
    joystick.A(key.RF_RX.btn.components.A);
    joystick.B(key.RF_RX.btn.components.B);
    joystick.X(key.RF_RX.btn.components.X);
    joystick.Y(key.RF_RX.btn.components.Y);
    joystick.up(key.RF_RX.btn.components.up);
    joystick.down(key.RF_RX.btn.components.down);
    joystick.left(key.RF_RX.btn.components.left);
    joystick.right(key.RF_RX.btn.components.right);
    joystick.LT(key.RF_RX.btn.components.L2);
    joystick.RT(key.RF_RX.btn.components.R2);
    joystick.lx(key.RF_RX.lx);
    joystick.ly(key.RF_RX.ly);
    joystick.rx(key.RF_RX.rx);
    joystick.ry(key.RF_RX.ry);
  }

  bool isJoystickTimeout() const  { return isJoystickTimeout_; }

private:
  uint32_t joystick_timeout_ms_ = 3000;
  bool isJoystickTimeout_ = false;
  std::chrono::time_point<std::chrono::system_clock> last_joystick_time_;
};


} // namespace go2
} // namespace robot
} // namespace unitree