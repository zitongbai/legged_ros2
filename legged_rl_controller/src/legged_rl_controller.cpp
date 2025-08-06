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

  // Command velocity subscriber
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_buffer_.writeFromNonRT(msg);
    });
  
  // Load RL policy network
  try{
    RCLCPP_INFO(get_node()->get_logger(), "Loading RL policy network from: %s", rl_policy_path_.c_str());
    policy_net_ = std::make_shared<torch::jit::script::Module>(torch::jit::load(rl_policy_path_));
    policy_net_->eval(); // Set the model to evaluation mode
  } catch (const c10::Error & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load RL policy network: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Successfully loaded RL policy network.");

  // Initialize observation tensors
  obs_tensor_ = torch::zeros({1, static_cast<long>(obs_num_)}, torch::kFloat32).to(torch::kCPU);

  RCLCPP_INFO(get_node()->get_logger(), "Legged RL Controller configured successfully.");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedRLController::on_activate(const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(get_node()->get_logger(), "Activating Legged RL Controller...");
  if (LeggedController::on_activate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Reset command velocity buffer
  cmd_vel_buffer_.reset();

  action_tensor_ = torch::zeros({static_cast<long>(joint_names_.size())}, torch::kFloat32).to(torch::kCPU);
  target_joint_pos_.resize(joint_names_.size(), 0.0);
  // start inference thread
  inference_thread_ = std::thread([this]() {
    using clock = std::chrono::high_resolution_clock;
    const std::chrono::duration<double> desired_period(1.0 / this->params_.control_rate); // e.g., 1.0 / 50 Hz = 0.02 s
    const auto dt = std::chrono::duration_cast<clock::duration>(desired_period);

    const auto start_time = clock::now();
    auto sleep_till = start_time + dt;

    while (rclcpp::ok() 
      && (this->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE || 
          this->get_state().id() == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING)
    ) {
      update_observations_();

      torch::NoGradGuard no_grad; // Disable gradients for inference

      action_tensor_ = policy_net_->forward({obs_tensor_}).toTensor().squeeze(0).to(torch::kFloat32);
      
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        // ref: isaaclab.envs.mdp.actions.joint_actions.py: JointAction.process_actions
        // scale the raw action and add the default joint position
        target_joint_pos_[i] = action_tensor_[i].item<double>() * action_term_.scale[i] + action_term_.default_joint_pos[i];
        // clip the joint position
        target_joint_pos_[i] = std::clamp(target_joint_pos_[i], action_term_.clip_min[i], action_term_.clip_max[i]);
      }

      // Sleep to maintain the loop rate
      std::this_thread::sleep_until(sleep_till);
      sleep_till += dt;
    }
  });

  RCLCPP_INFO(get_node()->get_logger(), "Legged RL Controller activated successfully.");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedRLController::on_deactivate(const rclcpp_lifecycle::State & previous_state){

  if (LeggedController::on_deactivate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // stop inference thread
  if (inference_thread_.joinable()) {
    inference_thread_.join();
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

controller_interface::return_type LeggedRLController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

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

  joint_interface_->set_joint_command(
    target_joint_pos_, 
    std::vector<double>(joint_names_.size(), 0.0),
    std::vector<double>(joint_names_.size(), 0.0),
    action_term_.kp,
    action_term_.kd
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
  double roll_pitch_threshold = M_PI / 4; // 45 degrees
  return fabs(roll) > roll_pitch_threshold || fabs(pitch) > roll_pitch_threshold;
}

void LeggedRLController::update_observations_(){
  size_t offset = 0;
  for (auto& term : obs_terms_) {
    const auto & obs = term.func(&term);

    // Clip and scale the observation
    torch::Tensor obs_scaled = obs.clamp(term.clip[0], term.clip[1]) * term.scale;

    // Fill the observation tensor
    obs_tensor_.slice(1, offset, offset + obs.size(0)) = obs_scaled.to(torch::kFloat32).unsqueeze(0);
    offset += obs.size(0);
  }
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
  
  obs_terms_.reserve(params_.obs_names.size());
  obs_num_ = 0;
  for (const auto& obs_name : params_.obs_names) {
    auto & obs_cfg = params_.obs_cfg.obs_names_map;

    // find the observation function
    ObsFunc func;
    int obs_n = 0;
    try{
      obs_n = find_obs_func_(obs_name, func);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to find observation function for %s: %s", obs_name.c_str(), e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    // history buffer
    int history_length = obs_cfg[obs_name].history_length;
    HistoryBuffer<torch::Tensor> history_buffer(history_length);
    if (history_length == 0) {
      history_length = 1; // default to 1 if history length is 0
    } else if (history_length < 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid history length %d for observation %s", history_length, obs_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    // init buffer with zero tensors
    for (int i = 0; i < history_length; ++i) {
      history_buffer.push(torch::zeros({obs_n}, torch::kFloat32));
    }
    
    // initialize the observation tensor
    torch::Tensor obs_tensor = torch::zeros({obs_n * history_length}, torch::kFloat32);
    
    // size of input observation tensor
    obs_num_ += obs_n * history_length;

    // observation clip
    std::vector<double> clip;
    clip.resize(2, 0.0);
    if(obs_cfg[obs_name].clip.size() == 2){
      clip = obs_cfg[obs_name].clip;
    } else {
      clip.at(0) = -std::numeric_limits<double>::infinity();
      clip.at(1) = std::numeric_limits<double>::infinity();
    }

    // observation scale
    double scale = obs_cfg[obs_name].scale;

    // create the observation term
    obs_terms_.emplace_back(ObsTerm{
      obs_name, 
      func, 
      obs_n, 
      scale, 
      clip, 
      obs_tensor,
      history_buffer
    });

    RCLCPP_DEBUG(get_node()->get_logger(), "Observation term %s: size = %d, clip = [%.2f, %.2f], scale = %.2f, history_length = %d",
                obs_name.c_str(), obs_n, clip[0], clip[1], scale, history_length);
  }

  action_term_.clip_min.resize(joint_names_.size(), -std::numeric_limits<double>::infinity());
  action_term_.clip_max.resize(joint_names_.size(), std::numeric_limits<double>::infinity());
  action_term_.scale.resize(joint_names_.size(), 1.0);
  action_term_.kp.resize(joint_names_.size(), 0.0);
  action_term_.kd.resize(joint_names_.size(), 0.0);
  action_term_.default_joint_pos.resize(joint_names_.size(), 0.0);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& joint_name = joint_names_[i];
    if (params_.action_cfg.joint_names_map.find(joint_name) != params_.action_cfg.joint_names_map.end()) {
      const auto& action_cfg = params_.action_cfg.joint_names_map.at(joint_name);
      if(action_cfg.clip.size() == 2) {
        action_term_.clip_min[i] = action_cfg.clip[0];
        action_term_.clip_max[i] = action_cfg.clip[1];
      }else {
        action_term_.clip_min[i] = -std::numeric_limits<double>::infinity();
        action_term_.clip_max[i] = std::numeric_limits<double>::infinity();
      }
      action_term_.scale[i] = action_cfg.scale;
      action_term_.kp[i] = action_cfg.kp;
      action_term_.kd[i] = action_cfg.kd;
      action_term_.default_joint_pos[i] = action_cfg.default_pos;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint name %s not found in action configuration", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_DEBUG(get_node()->get_logger(), 
      "Joint %s: clip_min = %.2f, clip_max = %.2f, scale = %.2f, kp = %.2f, kd = %.2f, default_pos = %.2f",
      joint_name.c_str(), action_term_.clip_min[i], action_term_.clip_max[i], 
      action_term_.scale[i], action_term_.kp[i], action_term_.kd[i], action_term_.default_joint_pos[i]);
  }

  RCLCPP_INFO(get_node()->get_logger(), "LeggedRLController parameters configured");

  return controller_interface::CallbackReturn::SUCCESS;
}


/*********************************************************************
 * Observation Functions
 *********************************************************************/

int LeggedRLController::find_obs_func_(const std::string& name, ObsFunc& func){
  if (name == "base_ang_vel") {
    func = std::bind(&LeggedRLController::obs_base_ang_vel_, this, std::placeholders::_1);
    return 3;
  } else if (name == "projected_gravity") {
    func = std::bind(&LeggedRLController::obs_projected_gravity_, this, std::placeholders::_1);
    return 3;
  } else if (name == "velocity_commands") {
    func = std::bind(&LeggedRLController::obs_velocity_commands_, this, std::placeholders::_1);
    return 3;
  } else if (name == "joint_pos") {
    func = std::bind(&LeggedRLController::obs_joint_pos_, this, std::placeholders::_1);
    return joint_names_.size();
  } else if (name == "joint_vel") {
    func = std::bind(&LeggedRLController::obs_joint_vel_, this, std::placeholders::_1);
    return joint_names_.size();
  } else if (name == "actions") {
    func = std::bind(&LeggedRLController::obs_actions_, this, std::placeholders::_1);
    action_tensor_ = torch::zeros({static_cast<long>(joint_names_.size())}, torch::kFloat64);
    return joint_names_.size();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Unknown observation term: %s", name.c_str());
    throw std::runtime_error("Unknown observation term");
  }
}

const torch::Tensor& LeggedRLController::obs_base_ang_vel_(ObsTerm* obs_term){
  // assume we only use the first IMU for RL observation
  auto ang_vel = imu_interfaces_[0]->get_angular_velocity();
  if (obs_term->history_length() == 0){
    obs_term->obs_tensor[0] = ang_vel[0];
    obs_term->obs_tensor[1] = ang_vel[1];
    obs_term->obs_tensor[2] = ang_vel[2];
  } else {
    torch::Tensor ang_vel_tensor = torch::from_blob(
      ang_vel.data(), {3}, torch::kFloat64).clone();
    obs_term->history_buffer.push(ang_vel_tensor);
    auto history_obs_list = obs_term->history_buffer.get_all();
    obs_term->obs_tensor = torch::cat(history_obs_list, 0);
  }

  return obs_term->obs_tensor;
}

const torch::Tensor& LeggedRLController::obs_projected_gravity_(ObsTerm* obs_term){
  // assume we only use the first IMU for RL observation
  auto quat = imu_interfaces_[0]->get_orientation();  // (x,y,z,w)
  
  Eigen::Vector3d gravity_vector(0.0, 0.0, -1.0); // Assuming gravity in -Z direction
  Eigen::Quaterniond q(quat[3], quat[0], quat[1], quat[2]); // Convert to Eigen quaternion
  Eigen::Vector3d projected_gravity = q.inverse() * gravity_vector;
  
  if (obs_term->history_length() == 0){
    obs_term->obs_tensor[0] = projected_gravity[0];
    obs_term->obs_tensor[1] = projected_gravity[1];
    obs_term->obs_tensor[2] = projected_gravity[2];
  } else {
    torch::Tensor projected_gravity_tensor = torch::from_blob(
      projected_gravity.data(), {3}, torch::kFloat64).clone();
    obs_term->history_buffer.push(projected_gravity_tensor);
    auto history_obs_list = obs_term->history_buffer.get_all();
    obs_term->obs_tensor = torch::cat(history_obs_list, 0);
  }
  
  return obs_term->obs_tensor;
}

const torch::Tensor& LeggedRLController::obs_velocity_commands_(ObsTerm* obs_term){

  cmd_vel_msg_ = *cmd_vel_buffer_.readFromRT();
  torch::Tensor velocity_commands_tensor = torch::zeros({3}, torch::kFloat64);
  if(cmd_vel_msg_ == nullptr){
    velocity_commands_tensor[0] = 0.0;
    velocity_commands_tensor[1] = 0.0;
    velocity_commands_tensor[2] = 0.0;
  } else {
    velocity_commands_tensor[0] = cmd_vel_msg_->linear.x;
    velocity_commands_tensor[1] = cmd_vel_msg_->linear.y;
    velocity_commands_tensor[2] = cmd_vel_msg_->angular.z;
  }

  if(obs_term->history_length() == 0){
    obs_term->obs_tensor = velocity_commands_tensor;
  } else {
    obs_term->history_buffer.push(velocity_commands_tensor);
    auto history_obs_list = obs_term->history_buffer.get_all();
    obs_term->obs_tensor = torch::cat(history_obs_list, 0);
  }

  return obs_term->obs_tensor;
}

const torch::Tensor& LeggedRLController::obs_joint_pos_(ObsTerm* obs_term){
  auto joint_pos = joint_interface_->get_joint_position();

  static thread_local torch::Tensor joint_pos_tensor = torch::zeros({static_cast<long>(joint_names_.size())}, torch::kFloat64);
  
  // Note that we use relative joint positions here. 
  for(size_t i=0; i<joint_names_.size(); i++){
    joint_pos_tensor[i] = joint_pos[i] - action_term_.default_joint_pos[i];
  }

  if (obs_term->history_length() == 0){
    obs_term->obs_tensor = joint_pos_tensor;
  } else {
    obs_term->history_buffer.push(joint_pos_tensor);
    auto history_obs_list = obs_term->history_buffer.get_all();
    obs_term->obs_tensor = torch::cat(history_obs_list, 0);
  }

  return obs_term->obs_tensor;
}

const torch::Tensor& LeggedRLController::obs_joint_vel_(ObsTerm* obs_term){
  auto joint_vel = joint_interface_->get_joint_velocity();

  static thread_local torch::Tensor joint_vel_tensor = torch::zeros({static_cast<long>(joint_names_.size())}, torch::kFloat64);
  
  joint_vel_tensor = torch::from_blob(
    joint_vel.data(), {static_cast<long>(joint_vel.size())}, torch::kFloat64).clone();

  if (obs_term->history_length() == 0){
    obs_term->obs_tensor = joint_vel_tensor;
  } else {
    obs_term->history_buffer.push(joint_vel_tensor);
    auto history_obs_list = obs_term->history_buffer.get_all();
    obs_term->obs_tensor = torch::cat(history_obs_list, 0);
  }

  return obs_term->obs_tensor;
}

const torch::Tensor& LeggedRLController::obs_actions_(ObsTerm* obs_term){
  // Assuming action_tensor_ is updated elsewhere in the code
  if(obs_term->history_length() == 0){
    obs_term->obs_tensor = action_tensor_;
  } else {
    obs_term->history_buffer.push(action_tensor_);
    auto history_obs_list = obs_term->history_buffer.get_all();
    obs_term->obs_tensor = torch::cat(history_obs_list, 0);
  }
  return obs_term->obs_tensor;
}



} // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  legged::LeggedRLController, controller_interface::ControllerInterface)