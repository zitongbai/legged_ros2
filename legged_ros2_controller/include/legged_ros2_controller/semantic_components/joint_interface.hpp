/**
 * @file joint_interface.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <limits>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "controller_interface/helpers.hpp"
#include "controller_interface/controller_interface.hpp"

#include "legged_ros2_control/legged_system_interface.hpp"

namespace legged{

class JointInterface{

public: 

  explicit JointInterface(const std::vector<std::string> & joint_names){
    joint_names_ = joint_names; // order of all values aligns with joint_names
    joint_num_ = joint_names_.size();

    state_interface_names_.reserve(joint_num_*3); // 3 interfaces per joint: position, velocity, effort
    command_interface_names_.reserve(joint_num_*5); // 5 interfaces per joint: position, velocity, effort, kp, kd
    for (const auto & joint_name : joint_names_) {
      state_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
      state_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
      state_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    }
    for (const auto & joint_name : joint_names_) {
      command_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
      command_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
      command_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
      command_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_KP);
      command_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_KD);
    }
  }

  ~JointInterface() = default;

  /// Assign loaned state interfaces from the hardware.
  /**
   * Assign loaned state interfaces on the controller start.
   *
   * \param[in] state_interfaces vector of interfaces provided by the controller.
   */
  bool assign_loaned_state_interfaces(
    std::vector<hardware_interface::LoanedStateInterface> & state_interfaces)
  {
    return controller_interface::get_ordered_interfaces<hardware_interface::LoanedStateInterface>(
      state_interfaces, state_interface_names_, "", state_interfaces_);
  }

  /// Assign loaned command interfaces from the hardware.
  /**
   * Assign loaned command interfaces on the controller start.
   * \param[in] command_interfaces vector of interfaces provided by the controller.
   */
  bool assign_loaned_command_interfaces(
    std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces)
  {
    return controller_interface::get_ordered_interfaces<hardware_interface::LoanedCommandInterface>(
      command_interfaces, command_interface_names_, "", command_interfaces_);
  }

  /// Release loaned interfaces from the hardware.
  void release_interfaces() { state_interfaces_.clear(); command_interfaces_.clear(); }

  /// Definition of state interface names for the component.
  /**
   * The function should be used in "state_interface_configuration()" of a controller to provide
   * standardized interface names semantic component.
   *
   * \default Default implementation defined state interfaces as "name/NR" where NR is number
   * from 0 to size of values;
   * \return list of strings with state interface names for the semantic component.
   */
  std::vector<std::string> get_state_interface_names(){
    return state_interface_names_;
  }

  /// Definition of command interface names for the component.
  /**
   * The function should be used in "command_interface_configuration()" of a controller to provide
   * standardized interface names semantic component.
   *
   * \default Default implementation defined command interfaces as "name/NR" where NR is number
   * from 0 to size of values;
   * \return list of strings with command interface names for the semantic component.
   */
  std::vector<std::string> get_command_interface_names(){
    return command_interface_names_;
  }

  std::vector<double> get_joint_position() const {
    std::vector<double> positions;
    positions.reserve(joint_num_);
    for(size_t i=0; i<joint_num_; i++){
      positions.push_back(state_interfaces_[3*i].get().get_value());
    }
    return positions;
  }

  std::vector<double> get_joint_velocity() const {
    std::vector<double> velocities;
    velocities.reserve(joint_num_);
    for(size_t i=0; i<joint_num_; i++){
      velocities.push_back(state_interfaces_[3*i+1].get().get_value());
    }
    return velocities;
  }

  std::vector<double> get_joint_effort() const {
    std::vector<double> efforts;
    efforts.reserve(joint_num_);
    for(size_t i=0; i<joint_num_; i++){
      efforts.push_back(state_interfaces_[3*i+2].get().get_value());
    }
    return efforts;
  }

  void set_joint_command(const std::vector<double> & pos, 
    const std::vector<double> & vel, 
    const std::vector<double> & ff, 
    const std::vector<double> & kp, 
    const std::vector<double> & kd){
    // check length is the same
    if (pos.size() != joint_num_ || vel.size() != joint_num_ || ff.size() != joint_num_ || kp.size() != joint_num_ || kd.size() != joint_num_) {
      std::cout << "Error: Input vector sizes do not match the number of joints." 
       << "Pos size: " << pos.size()
        << ", Vel size: " << vel.size()
        << ", FF size: " << ff.size()
        << ", Kp size: " << kp.size()
        << ", Kd size: " << kd.size()
        << ", Joint num: " << joint_num_ << std::endl;
      throw std::invalid_argument("Input vector sizes must match the number of joints.");
    }
    
    for(size_t i=0; i<joint_num_; i++){
      command_interfaces_[5*i].get().set_value(pos[i]);
      command_interfaces_[5*i+1].get().set_value(vel[i]);
      command_interfaces_[5*i+2].get().set_value(ff[i]);
      command_interfaces_[5*i+3].get().set_value(kp[i]);
      command_interfaces_[5*i+4].get().set_value(kd[i]);
    }
  }

private:
  std::vector<std::string> joint_names_;
  size_t joint_num_;
  
  std::vector<std::string> state_interface_names_;
  std::vector<std::string> command_interface_names_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_;
};

}
