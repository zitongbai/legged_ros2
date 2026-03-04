/**
 * @file legged_rl_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief
 * @version 0.1
 * @date 2026-01-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "legged_rl_controller/legged_rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "legged_rl_controller/isaaclab/algorithms/algorithms.h"
#include "legged_rl_controller/isaaclab/envs/mdp/actions/joint_actions.h"
#include "legged_rl_controller/isaaclab/envs/mdp/observations/observations.h"

namespace legged
{

controller_interface::CallbackReturn LeggedRLController::on_init()
{
  if (LeggedController::on_init() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_names_ = auto_declare<std::vector<std::string>>(
    "joint_names", std::vector<std::string>());
  imu_names_ = auto_declare<std::vector<std::string>>(
    "imu_names", std::vector<std::string>());

  onnx_model_path_ = auto_declare<std::string>("onnx_model_path", "");
  io_descriptors_path_ = auto_declare<std::string>("io_descriptors_path", "");
  auto_declare<std::string>("waypoints_topic", "/waypoints");
  waypoint_first_xyz_norm_max_ = auto_declare<double>("waypoint_first_xyz_norm_max", 0.5);

  /*
  // Disabled: legacy cmd_vel command pipeline.
  auto_declare<std::string>("cmd_vel_topic", "cmd_vel");
  auto_declare<std::vector<double>>("cmd_vel_range_lin_vel_x", std::vector<double>());
  auto_declare<std::vector<double>>("cmd_vel_range_lin_vel_y", std::vector<double>());
  auto_declare<std::vector<double>>("cmd_vel_range_ang_vel_z", std::vector<double>());
  */

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedRLController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  onnx_model_path_ = get_node()->get_parameter("onnx_model_path").as_string();
  io_descriptors_path_ = get_node()->get_parameter("io_descriptors_path").as_string();
  auto waypoints_topic = get_node()->get_parameter("waypoints_topic").as_string();
  waypoint_first_xyz_norm_max_ =
    get_node()->get_parameter("waypoint_first_xyz_norm_max").as_double();

  if (onnx_model_path_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'onnx_model_path' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (io_descriptors_path_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'io_descriptors_path' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(waypoint_first_xyz_norm_max_) || waypoint_first_xyz_norm_max_ <= 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'waypoint_first_xyz_norm_max' must be finite and > 0, got %.6f.",
      waypoint_first_xyz_norm_max_);
    return controller_interface::CallbackReturn::ERROR;
  }

  YAML::Node env_cfg;
  try {
    env_cfg = YAML::LoadFile(io_descriptors_path_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load IO descriptors: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto robot_node = env_cfg["articulations"]["robot"];
  if (!robot_node.IsDefined()) {
    RCLCPP_ERROR(get_node()->get_logger(), "IO descriptors missing 'articulations.robot'.");
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto yaml_joint_names = robot_node["joint_names"].as<std::vector<std::string>>();
  if (joint_names_.empty()) {
    joint_names_ = yaml_joint_names;
  } else if (joint_names_.size() != yaml_joint_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Joint names size mismatch: param=%zu, yaml=%zu.",
      joint_names_.size(), yaml_joint_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto parse_generated_command_dim = [&](const YAML::Node &cfg) -> size_t {
      const auto observations_node = cfg["observations"];
      if (!observations_node.IsDefined() || !observations_node.IsMap()) {
        throw std::runtime_error("IO descriptors missing 'observations' map.");
      }

      size_t generated_command_dim = 0U;
      bool found = false;

      auto parse_group = [&](const YAML::Node &group_cfg) {
          if (!group_cfg.IsSequence()) {
            throw std::runtime_error("Observation group must be a sequence.");
          }
          for (auto it = group_cfg.begin(); it != group_cfg.end(); ++it) {
            const auto term = *it;
            const auto name_node = term["name"];
            if (!name_node.IsDefined() || name_node.as<std::string>() != "generated_commands") {
              continue;
            }
            const auto shape_node = term["shape"];
            if (!shape_node.IsDefined() || !shape_node.IsSequence() || shape_node.size() == 0U) {
              throw std::runtime_error("Observation 'generated_commands' missing valid 'shape'.");
            }
            const int dim = shape_node[0].as<int>(-1);
            if (dim <= 0) {
              throw std::runtime_error("Observation 'generated_commands' has non-positive shape[0].");
            }
            const size_t dim_size_t = static_cast<size_t>(dim);
            if (!found) {
              generated_command_dim = dim_size_t;
              found = true;
            } else if (generated_command_dim != dim_size_t) {
              throw std::runtime_error(
                "Observation 'generated_commands' has inconsistent shape[0] across groups.");
            }
          }
      };

      if (observations_node.size() == 1U) {
        parse_group(observations_node.begin()->second);
      } else {
        for (auto group = observations_node.begin(); group != observations_node.end(); ++group) {
          parse_group(group->second);
        }
      }
      if (!found) {
        throw std::runtime_error("Observation 'generated_commands' not found in IO descriptors.");
      }
      if (generated_command_dim % 3U != 0U) {
        throw std::runtime_error(
          "Observation 'generated_commands' shape[0] must be divisible by 3.");
      }
      return generated_command_dim;
    };

  size_t generated_command_dim = 0U;
  size_t num_waypoints = 0U;
  try {
    generated_command_dim = parse_generated_command_dim(env_cfg);
    num_waypoints = generated_command_dim / 3U;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid generated_commands config: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (LeggedController::on_configure(previous_state) !=
    controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (imu_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No IMU interfaces available for RL controller.");
    return controller_interface::CallbackReturn::ERROR;
  }

  waypoints_buffer_ = std::make_shared<WaypointsBuffer>();
  waypoints_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseArray>(
    waypoints_topic, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
      if (msg && !msg->poses.empty()) {
        const auto & p0 = msg->poses.front().position;
        const double first_norm = std::sqrt(p0.x * p0.x + p0.y * p0.y + p0.z * p0.z);
        if (!std::isfinite(first_norm) || first_norm > waypoint_first_xyz_norm_max_) {
          auto safe_msg = std::make_shared<geometry_msgs::msg::PoseArray>(*msg);
          for (auto & pose : safe_msg->poses) {
            pose.position.x = 0.0;
            pose.position.y = 0.0;
            pose.position.z = 0.0;
          }
          waypoints_buffer_->writeFromNonRT(safe_msg);
          RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "Waypoint guard triggered: first point xyz norm %.3f exceeds threshold %.3f. "
            "All waypoints are zeroed.",
            first_norm,
            waypoint_first_xyz_norm_max_);
          return;
        }
      }
      waypoints_buffer_->writeFromNonRT(msg);
    });

  robot_ = std::make_shared<LeggedArticulation>(
    imu_interfaces_[0], joint_interface_, waypoints_buffer_); // Use the first IMU interface

  robot_->data.waypoint_command.generated_command_dim = generated_command_dim;
  robot_->data.waypoint_command.num_waypoints = num_waypoints;
  robot_->data.waypoint_command.path_command.assign(generated_command_dim, 0.0f);

  /*
  // Disabled: legacy cmd_vel command pipeline.
  cmd_vel_buffer_ = std::make_shared<CmdBuffer>();
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_buffer_->writeFromNonRT(msg);
    });

  auto range_lin_vel_x = get_node()->get_parameter("cmd_vel_range_lin_vel_x").as_double_array();
  auto range_lin_vel_y = get_node()->get_parameter("cmd_vel_range_lin_vel_y").as_double_array();
  auto range_ang_vel_z = get_node()->get_parameter("cmd_vel_range_ang_vel_z").as_double_array();

  auto set_range = [](const std::vector<double> &range, std::array<float, 2U> &target) {
    if (range.size() == 2U) {
      target = {static_cast<float>(range[0]), static_cast<float>(range[1])};
    } else {
      target = {
        -std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity()
      };
    }
  };
  set_range(range_lin_vel_x, robot_->data.velocity_command.range.lin_vel_x);
  set_range(range_lin_vel_y, robot_->data.velocity_command.range.lin_vel_y);
  set_range(range_ang_vel_z, robot_->data.velocity_command.range.ang_vel_z);
  */

  try {
    env_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(env_cfg, robot_);
    env_->alg = std::make_unique<isaaclab::OrtRunner>(onnx_model_path_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize RL environment: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Legged RL Controller configured successfully. generated_commands dim=%zu, "
    "num_waypoints=%zu, waypoints_topic=%s, waypoint_first_xyz_norm_max=%.3f",
    generated_command_dim, num_waypoints, waypoints_topic.c_str(), waypoint_first_xyz_norm_max_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedRLController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  if (LeggedController::on_activate(previous_state) !=
    controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (waypoints_buffer_) {
    waypoints_buffer_->reset();
  }
  if (robot_) {
    std::fill(
      robot_->data.waypoint_command.path_command.begin(),
      robot_->data.waypoint_command.path_command.end(),
      0.0f);
  }
  if (env_) {
    env_->reset();
  }

  /*
  // Disabled: legacy cmd_vel command pipeline.
  if (cmd_vel_buffer_) {
    cmd_vel_buffer_->reset();
  }
  */

  RCLCPP_INFO(get_node()->get_logger(), "Legged RL Controller activated successfully.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedRLController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  if (LeggedController::on_deactivate(previous_state) !=
    controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_interface_->set_joint_command(
    std::vector<double>(joint_names_.size(), 0.0),
    std::vector<double>(joint_names_.size(), 0.0),
    std::vector<double>(joint_names_.size(), 0.0),
    std::vector<double>(joint_names_.size(), 0.0),
    std::vector<double>(joint_names_.size(), 0.0));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeggedRLController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!env_ || !env_->alg) {
    RCLCPP_ERROR(get_node()->get_logger(), "RL environment is not initialized.");
    return controller_interface::return_type::ERROR;
  }

  try {
    env_->step();
    auto action = env_->action_manager->processed_actions();
    if (action.size() != joint_names_.size()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Action size mismatch: action=%zu, joints=%zu.",
        action.size(), joint_names_.size());
      return controller_interface::return_type::ERROR;
    }

    joint_interface_->set_joint_command(
      action,
      std::vector<float>(joint_names_.size(), 0.0f),
      std::vector<float>(joint_names_.size(), 0.0f),
      env_->robot->data.joint_stiffness,
      env_->robot->data.joint_damping);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "RL update failed: %s", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

}  // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  legged::LeggedRLController, controller_interface::ControllerInterface)
