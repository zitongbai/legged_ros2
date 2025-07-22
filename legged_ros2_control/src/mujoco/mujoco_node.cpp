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
#include "rosgraph_msgs/msg/clock.hpp"

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

  // // sim time clock publisher
  // auto sim_time_clock_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>(
  //   "/clock", 10);

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

  // std::thread sim_time_thread([&]() {
  //   std::chrono::milliseconds sleep_duration(1);
  //   rosgraph_msgs::msg::Clock clock_msg;
  //   auto sim_time = mujoco_data->time;
  //   while(rclcpp::ok()){
  //     sim_time = mujoco_data->time;
  //     int sim_time_sec = static_cast<int>(sim_time);
  //     int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);
  //     rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  //     clock_msg.clock = sim_time_ros;
  //     sim_time_clock_publisher->publish(clock_msg);
  //     std::this_thread::sleep_for(sleep_duration);
  //   }
  // });

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
  auto last_render_time = std::chrono::steady_clock::now();
  const auto render_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / 60.0));

  while(rclcpp::ok() && ! rendering->is_close_flag_raised()){
    auto current_time = std::chrono::steady_clock::now();
    
    // check if it's time to render
    if (current_time - last_render_time >= render_period) {
      rendering->update();
      last_render_time = current_time;
      
      // Updating cameras at ~6 Hz
      mjtNum current_sim_time = mujoco_data->time;
      if (current_sim_time - last_cam_update > 1.0 / 6.0)
      {
        cameras->update(mujoco_model, mujoco_data);
        last_cam_update = current_sim_time;
      }
    }

    // short sleep to avoid busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  rendering->close();
  cameras->close();

  // sim_time_thread.join();

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 0;
}

