/**
 * @file legged_rl_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "legged_rl_controller/legged_rl_controller.hpp"
#include <regex>

#include "legged_rl_controller/isaaclab/envs/mdp/actions/joint_actions.h"
#include "legged_rl_controller/isaaclab/manager/manager_term_cfg.h"
#include "legged_rl_controller/isaaclab/envs/mdp/observations.h"

namespace legged
{

controller_interface::CallbackReturn LeggedRLController::on_init(){
  if (LeggedController::on_init() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    param_listener_ = std::make_shared<legged_rl_controller::ParamListener>(get_node());
  }catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during LeggedRLController's on_init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rl_policy_path_ = auto_declare<std::string>("rl_policy_path", "");  // This parameter is set in launch file

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn LeggedRLController::on_configure(const rclcpp_lifecycle::State & previous_state) {

  if(configure_parameters_() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (LeggedController::on_configure(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  cmd_vel_buffer_ = std::make_shared<CmdBuffer>();
  // Command velocity subscriber
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_buffer_->writeFromNonRT(msg);
    });
  
  // Assume we only have one IMU
  robot_ = std::make_shared<LeggedArticulation>(imu_interfaces_[0], joint_interface_, cmd_vel_buffer_);
  env_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(std::move(env_cfg_), robot_);

  RCLCPP_INFO(get_node()->get_logger(), "Legged RL Controller configured successfully.");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedRLController::on_activate(const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(get_node()->get_logger(), "Activating Legged RL Controller...");
  if (LeggedController::on_activate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Reset command velocity buffer
  cmd_vel_buffer_->reset();

  // Reset env
  env_->reset();

  RCLCPP_INFO(get_node()->get_logger(), "Legged RL Controller activated successfully.");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedRLController::on_deactivate(const rclcpp_lifecycle::State & previous_state){

  if (LeggedController::on_deactivate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_interface_->set_joint_command(
    std::vector<double>(joint_names_.size(), 0.0), 
    std::vector<double>(joint_names_.size(), 0.0), 
    std::vector<double>(joint_names_.size(), 0.0), 
    std::vector<double>(joint_names_.size(), 0.0), 
    std::vector<double>(joint_names_.size(), 0.0)
  );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeggedRLController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){

  // fall detection
  if (detect_fall_()) {
    RCLCPP_WARN(get_node()->get_logger(), "Robot is falling! Stopping the controller.");
    // Set all joint commands to zero
    joint_interface_->set_joint_command(
      std::vector<double>(joint_names_.size(), 0.0),
      std::vector<double>(joint_names_.size(), 0.0),
      std::vector<double>(joint_names_.size(), 0.0),
      std::vector<double>(joint_names_.size(), 0.0),
      std::vector<double>(joint_names_.size(), 0.0)
    );
    return controller_interface::return_type::ERROR;
  }

  env_->step(period.seconds());
  auto action = env_->action_manager->processed_actions();

  joint_interface_->set_joint_command(
    action, 
    std::vector<float>(joint_names_.size(), 0.0f),
    std::vector<float>(joint_names_.size(), 0.0f),
    env_->robot->data.joint_stiffness,
    env_->robot->data.joint_damping
  );

  return controller_interface::return_type::OK;
}

bool LeggedRLController::detect_fall_(){
  // Check if the robot is falling based on IMU data
  if (imu_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No IMU interfaces found for fall detection.");
    return false;
  }

  auto quat = imu_interfaces_[0]->get_orientation();  // (x,y,z,w)
  double x = quat[0];
  double y = quat[1];
  double z = quat[2];
  double w = quat[3];

  double roll, pitch;
  // Convert quaternion to Euler angles
  roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  pitch = asin(2 * (w * y - z * x));

  // Check if robot is falling
  double roll_pitch_threshold = M_PI / 3; // 60 degrees
  return fabs(roll) > roll_pitch_threshold || fabs(pitch) > roll_pitch_threshold;
}

/*********************************************************************
 * Parameter
 *********************************************************************/

void LeggedRLController::update_parameters_(){
  if (!param_listener_->is_old(params_)) {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn LeggedRLController::configure_parameters_(){
  update_parameters_();
  
  joint_names_ = params_.joint_names;
  imu_names_ = params_.imu_names;

  if(imu_names_.size() > 1) {
    RCLCPP_WARN(get_node()->get_logger(), "Multiple IMUs detected, using the first one for RL observations.");
  }
  
  env_cfg_.policy_net_path = rl_policy_path_;
  // -------------------------------------------
  // Joint and action configuration
  // -------------------------------------------
  env_cfg_.default_joint_pos.resize(joint_names_.size(), 0.0f);
  env_cfg_.joint_stiffness.resize(joint_names_.size(), 0.0f);
  env_cfg_.joint_damping.resize(joint_names_.size(), 0.0f);

  env_cfg_.action_cfgs.clear();
  auto action_cfg_ptr = std::make_unique<isaaclab::JointActionConfig>();
  action_cfg_ptr->name = "JointPositionAction";
  action_cfg_ptr->scale.resize(joint_names_.size(), 1.0f);
  action_cfg_ptr->offset.resize(joint_names_.size(), 0.0f);
  action_cfg_ptr->clip.resize(joint_names_.size(), 
    std::vector<float>{-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()});

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto & joint_name = joint_names_[i];
    if (params_.action_cfg.joint_names_map.find(joint_name) != params_.action_cfg.joint_names_map.end()) {
      const auto& action_cfg = params_.action_cfg.joint_names_map.at(joint_name);
      // default pos
      env_cfg_.default_joint_pos[i] = action_cfg.default_pos;
      action_cfg_ptr->offset[i] = action_cfg.default_pos;
      // Kp, Kd
      env_cfg_.joint_stiffness[i] = action_cfg.kp;
      env_cfg_.joint_damping[i] = action_cfg.kd;
      // Clip
      if(action_cfg.clip.size() == 2 ) {
        action_cfg_ptr->clip[i][0] = action_cfg.clip[0];
        action_cfg_ptr->clip[i][1] = action_cfg.clip[1];
      }
      // Scale
      action_cfg_ptr->scale[i] = action_cfg.scale;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint name %s not found in action configuration", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  env_cfg_.action_cfgs.push_back(std::move(action_cfg_ptr));

  // -------------------------------------------
  // Observation configuration
  // -------------------------------------------
  env_cfg_.observation_cfgs.clear();
  auto & obs_cfg_map = params_.obs_cfg.obs_names_map;
  for(const auto & obs_name : params_.obs_names) {
    // Check if the observation name exists in the configuration map
    if (obs_cfg_map.find(obs_name) == obs_cfg_map.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Observation name %s not found in observation configuration", obs_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    auto obs_cfg = obs_cfg_map.at(obs_name);
    auto obs_term_cfg_ptr = std::make_unique<isaaclab::ObservationTermCfg>();
    obs_term_cfg_ptr->name = obs_name;
    obs_term_cfg_ptr->history_length = obs_cfg.history_length;
    obs_term_cfg_ptr->scale = obs_cfg.scale;
    if(obs_cfg.clip.size() == 2) {
      obs_term_cfg_ptr->clip = {
        static_cast<float>(obs_cfg.clip[0]), 
        static_cast<float>(obs_cfg.clip[1])
      };
    } else {
      // obs_term_cfg_ptr->clip = { -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity() };
      std::cout << "[LeggedRLController] Observation " << obs_name 
                << " clip not set, defaulting to no clipping." << std::endl;
      obs_term_cfg_ptr->clip.clear();
    }
    env_cfg_.observation_cfgs.emplace_back(std::move(obs_term_cfg_ptr));

    // debug print
    std::cout << "[LeggedRLController] Observation " << obs_name 
              << " configured with history length: " << obs_cfg.history_length
              << ", scale: " << obs_cfg.scale
              << ", clip: [" << obs_cfg.clip[0] << ", " << obs_cfg.clip[1] << "]" << std::endl;
  }

  // -------------------------------------------
  // Command velocity configuration
  // -------------------------------------------
  env_cfg_.command_velocity_range.lin_vel_x = {
    static_cast<float>(params_.range_lin_vel_x[0]),
    static_cast<float>(params_.range_lin_vel_x[1])
  };
  env_cfg_.command_velocity_range.lin_vel_y = {
    static_cast<float>(params_.range_lin_vel_y[0]),
    static_cast<float>(params_.range_lin_vel_y[1])
  };
  env_cfg_.command_velocity_range.ang_vel_z = {
    static_cast<float>(params_.range_ang_vel_z[0]),
    static_cast<float>(params_.range_ang_vel_z[1])
  };

  // -------------------------------------------
  // Other configurations
  // -------------------------------------------
  env_cfg_.set_extra<double>("gait_period", params_.gait_period);

  RCLCPP_INFO(get_node()->get_logger(), "LeggedRLController parameters configured");

  return controller_interface::CallbackReturn::SUCCESS;
}



} // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  legged::LeggedRLController, controller_interface::ControllerInterface)