/**
 * @file mujoco_node.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "mujoco/mujoco.h"

#include "rclcpp/rclcpp.hpp"

#include "legged_ros2_control/legged_ros2_control.hpp"
#include "legged_ros2_control/mujoco/mujoco_ros2_control.hpp"
#include "legged_ros2_control/mujoco/mujoco_system_interface.hpp"
#include "legged_ros2_control/mujoco/mujoco_rendering.hpp"
#include "legged_ros2_control/mujoco/mujoco_cameras.hpp"


// MuJoCo data structures
mjModel *mujoco_model = nullptr;
mjData *mujoco_data = nullptr;


// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "mujoco_ros2_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");

  auto model_path = node->get_parameter("mujoco_model_path").as_string();

  // load and compile model
  char error[1000];
  mujoco_model = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
  if (!mujoco_model)
  {
    mju_error("Load model error: %s", error);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);

  // create MujocoRos2Control instance
  legged::MujocoRos2Control mujoco_control(node, mujoco_model, mujoco_data);

  mujoco_control.init();  // thread of controller manager and its spin is started here
  // TODO: sim time clock publisher, maybe in a separate thread

  mujoco_model->opt.timestep = 1.0 / mujoco_control.get_update_rate();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco model timestep set to: " << mujoco_model->opt.timestep << " seconds");

  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // initialize mujoco visualization environment for rendering and cameras
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }
  auto rendering = legged::MujocoRendering::get_instance();
  rendering->init(mujoco_model, mujoco_data);
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");

  auto cameras = std::make_unique<legged::MujocoCameras>(node);
  cameras->init(mujoco_model);

  // run main loop, target real-time simulation and 60 fps rendering with cameras around 6 hz
  mjtNum last_cam_update = mujoco_data->time;
  while(rclcpp::ok() && ! rendering->is_close_flag_raised()){
    mjtNum simstart = mujoco_data->time;
    while (mujoco_data->time - simstart < 1.0 / 60.0){
      // sleep 
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    rendering->update();
    // Updating cameras at ~6 Hz
    // TODO(eholum): Break control and rendering into separate processes
    if (simstart - last_cam_update > 1.0 / 6.0)
    {
      cameras->update(mujoco_model, mujoco_data);
      last_cam_update = simstart;
    }
  }

  rendering->close();
  cameras->close();

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 0;
}

