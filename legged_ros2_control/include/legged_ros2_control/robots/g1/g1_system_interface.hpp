/**
 * @file g1_system_interface.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <map>
#include "rclcpp_lifecycle/state.hpp"

#include "legged_ros2_control/legged_system_interface.hpp"
#include "legged_ros2_control/visibility_control.h"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

namespace legged {

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
  }

  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
  }

 private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

// Parallel mechanism (ankle and waist) control mode
enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

enum G1JointIndex {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  WaistYaw = 12,
  WaistRoll = 13,        // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistA = 13,           // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistPitch = 14,       // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistB = 14,           // NOTE INVALID for g1 23dof/29dof with waist locked
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,   // NOTE INVALID for g1 23dof
  LeftWristYaw = 21,     // NOTE INVALID for g1 23dof
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,  // NOTE INVALID for g1 23dof
  RightWristYaw = 28     // NOTE INVALID for g1 23dof
};

static const std::map<std::string, int> g1_joint_index_map = {
  {"left_hip_pitch_joint", LeftHipPitch},
  {"left_hip_roll_joint", LeftHipRoll},
  {"left_hip_yaw_joint", LeftHipYaw},
  {"left_knee_joint", LeftKnee},
  {"left_ankle_pitch_joint", LeftAnklePitch},
  {"left_ankle_b_joint", LeftAnkleB},
  {"left_ankle_roll_joint", LeftAnkleRoll},
  {"left_ankle_a_joint", LeftAnkleA},
  {"right_hip_pitch_joint", RightHipPitch},
  {"right_hip_roll_joint", RightHipRoll},
  {"right_hip_yaw_joint", RightHipYaw},
  {"right_knee_joint", RightKnee},
  {"right_ankle_pitch_joint", RightAnklePitch},
  {"right_ankle_b_joint", RightAnkleB},
  {"right_ankle_roll_joint", RightAnkleRoll},
  {"right_ankle_a_joint", RightAnkleA},
  {"waist_yaw_joint", WaistYaw},
  {"waist_roll_joint", WaistRoll},
  {"waist_a_joint", WaistA},
  {"waist_pitch_joint", WaistPitch},
  {"waist_b_joint", WaistB},
  {"left_shoulder_pitch_joint", LeftShoulderPitch},
  {"left_shoulder_roll_joint", LeftShoulderRoll},
  {"left_shoulder_yaw_joint", LeftShoulderYaw},
  {"left_elbow_joint", LeftElbow},
  {"left_wrist_roll_joint", LeftWristRoll},
  {"left_wrist_pitch_joint", LeftWristPitch},
  {"left_wrist_yaw_joint", LeftWristYaw},
  {"right_shoulder_pitch_joint", RightShoulderPitch},
  {"right_shoulder_roll_joint", RightShoulderRoll},
  {"right_shoulder_yaw_joint", RightShoulderYaw},
  {"right_elbow_joint", RightElbow},
  {"right_wrist_roll_joint", RightWristRoll},
  {"right_wrist_pitch_joint", RightWristPitch},
  {"right_wrist_yaw_joint", RightWristYaw}
};

class HARDWARE_INTERFACE_PUBLIC G1SystemInterface : public LeggedSystemInterface
{

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(G1SystemInterface)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(G1SystemInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  static const std::string HG_CMD_TOPIC;
  static const std::string HG_STATE_TOPIC;



protected:

  bool build_joint_data_() override;

  std::string network_interface_;
  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;
  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;

  void lowstate_handler_(const void * msg);
  bool connected_;
  LowState_ low_state_;
  LowCmd_ low_cmd_;

  Mode mode_pr_ = Mode::PR;   // Parallel mechanism (ankle and waist) control mode
  uint8_t mode_machine_ = 0;  // G1 Type

};


}
