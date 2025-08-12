// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <array>
#include <map>
#include <string>

namespace unitree
{
namespace robot
{
namespace go2
{

enum class JointIndex{
    FR_Hip = 0,
    FR_Thigh = 1,
    FR_Calf = 2,
    FL_Hip = 3,
    FL_Thigh = 4,
    FL_Calf = 5,
    RR_Hip = 6,
    RR_Thigh = 7,
    RR_Calf = 8,
    RL_Hip = 9,
    RL_Thigh = 10,
    RL_Calf = 11,
};

enum class FSMMode{
    idle = 0,
    balanceStand = 1,
    pose = 2,
    locomotion = 3,
    lieDown = 5,
    jointLock = 6,
    damping = 7,
    recoveryStand = 8,
    sit = 10,
    frontFlip = 11,
    frontJump = 12,
    frontPounc = 13,
};
  
enum class GaitType{
    idle = 0,
    trot = 1,
    run = 2,
    climb_stair = 3,
    forwardDownStair = 4,
    adjust = 9,
};

static const std::map<std::string, JointIndex> joint_index_map = {
    {"FR_hip_joint", JointIndex::FR_Hip},
    {"FR_thigh_joint", JointIndex::FR_Thigh},
    {"FR_calf_joint", JointIndex::FR_Calf},
    {"FL_hip_joint", JointIndex::FL_Hip},
    {"FL_thigh_joint", JointIndex::FL_Thigh},
    {"FL_calf_joint", JointIndex::FL_Calf},
    {"RR_hip_joint", JointIndex::RR_Hip},
    {"RR_thigh_joint", JointIndex::RR_Thigh},
    {"RR_calf_joint", JointIndex::RR_Calf},
    {"RL_hip_joint", JointIndex::RL_Hip},
    {"RL_thigh_joint", JointIndex::RL_Thigh},
    {"RL_calf_joint", JointIndex::RL_Calf},
};


} // namespace go2
} // namespace robot
} // namespace unitree