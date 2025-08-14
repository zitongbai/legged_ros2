/**
 * @file legged_ros2_control.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"
#include "realtime_tools/realtime_helpers.hpp"

#include "legged_ros2_control/legged_ros2_control.hpp"


namespace legged
{
LeggedRos2Control::LeggedRos2Control(rclcpp::Node::SharedPtr node) : 
  node_(node), 
  logger_(rclcpp::get_logger(node_->get_name()+std::string(".legged_ros2_control")))
{
}

LeggedRos2Control::~LeggedRos2Control()
{
  cm_executor_->remove_node(controller_manager_);
  cm_executor_->cancel();

  if (cm_thread_.joinable()) cm_thread_.join();
  if (spin_thread_.joinable()) spin_thread_.join();
}

std::string LeggedRos2Control::get_robot_description_()
{
  // Getting robot description from parameter first. If not set trying from topic
  std::string robot_description;

  auto node = std::make_shared<rclcpp::Node>(
    "robot_description_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  if (node->has_parameter("robot_description"))
  {
    robot_description = node->get_parameter("robot_description").as_string();
    return robot_description;
  }

  RCLCPP_WARN(
    logger_,
    "Failed to get robot_description from parameter. Will listen on the ~/robot_description "
    "topic...");

  auto robot_description_sub = node->create_subscription<std_msgs::msg::String>(
    "robot_description", rclcpp::QoS(1).transient_local(),
    [&](const std_msgs::msg::String::SharedPtr msg)
    {
      if (!msg->data.empty() && robot_description.empty()) robot_description = msg->data;
    });

  while (robot_description.empty() && rclcpp::ok())
  {
    rclcpp::spin_some(node);
    RCLCPP_INFO(node->get_logger(), "Waiting for robot description message");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  return robot_description;
}


void LeggedRos2Control::init()
{

  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  urdf_string_ = this->get_robot_description_();

  // in the default usage of ros2 control, the controlller manager is responsible for 
  // parsing the hardware info from URDF and load the components
  // But here we will do it manually

  // Parse the URDF string to get the control hardware info
  std::vector<hardware_interface::HardwareInfo> hardware_info;
  try{
    hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string_);
  }catch (const std::runtime_error & ex){
    RCLCPP_ERROR_STREAM(logger_, "Error parsing hardware info from URDF: " << ex.what());
    return ;
  }


  // Create resource manager
  std::unique_ptr<hardware_interface::ResourceManager> resource_manager = 
    std::make_unique<hardware_interface::ResourceManager>();

  // Parse urdf in resource manager
  try{
    resource_manager->load_urdf(urdf_string_, false, false); // Do not validate interfaces and do not load components
  }catch (const std::runtime_error & ex){
    RCLCPP_ERROR_STREAM(logger_, "Error loading URDF in resource manager: " << ex.what());
    return;
  }

  // Import components according to the hardware info
  import_components_(hardware_info, resource_manager);

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller manager...");
  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), cm_executor_, manager_node_name, node_->get_namespace());
  cm_executor_->add_node(controller_manager_);
  
  const bool use_sim_time = controller_manager_->get_parameter_or<bool>("use_sim_time", false);
  RCLCPP_INFO(
    logger_, "Controller manager using simulation time: %s", use_sim_time ? "true" : "false");

  const int cpu_affinity = controller_manager_->get_parameter_or<int>("cpu_affinity", -1);
  if(cpu_affinity>=0){
    const auto affinity_result = realtime_tools::set_current_thread_affinity(cpu_affinity);
    if(!affinity_result.first){
      RCLCPP_WARN(logger_, "Unable to set the CPU affinity : '%s'", affinity_result.second.c_str());
    }
  }

  const bool has_realtime = realtime_tools::has_realtime_kernel();
  const bool lock_memory = controller_manager_->get_parameter_or<bool>("lock_memory", has_realtime);
  if(lock_memory){
    const auto lock_result = realtime_tools::lock_memory();
    if(!lock_result.first){
      RCLCPP_WARN(logger_, "Unable to lock memory : '%s'", lock_result.second.c_str());
    }
  }

  if(!controller_manager_->has_parameter("update_rate")){
    RCLCPP_ERROR_STREAM(logger_, "controller manager doesn't have an update_rate parameter");
    return;
  }
  update_rate_ = controller_manager_->get_parameter("update_rate").as_int();
  RCLCPP_INFO(logger_, "Controller manager update rate: %d Hz", update_rate_);

  const int thread_priority = controller_manager_->get_parameter_or<int>("thread_priority", 50);
  RCLCPP_INFO(
    logger_, "Spawning %s RT thread with scheduler priority: %d", controller_manager_->get_name(), thread_priority);

  cm_thread_ = std::thread(
    [&](){
      if(realtime_tools::has_realtime_kernel()){
        if (!realtime_tools::configure_sched_fifo(thread_priority)){
          RCLCPP_WARN(
            this->logger_,
            "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
            "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
            "for details on how to enable realtime scheduling.",
            errno, strerror(errno));
        } else {
          RCLCPP_INFO(
            this->logger_, "Successful set up FIFO RT scheduling policy with priority %i.",
            thread_priority);
        }
      } else {
        RCLCPP_WARN(
          this->logger_,
          "No real-time kernel detected on this system. See "
          "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
          "for details on how to enable realtime scheduling.");
      } // end if has_realtime

      const auto period = std::chrono::nanoseconds(1'000'000'000 / update_rate_);
      const auto cm_now = std::chrono::nanoseconds(this->controller_manager_->now().nanoseconds());
      std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> next_iteration_time{cm_now};

      rclcpp::Time previous_time = this->controller_manager_->now();
      rclcpp::Duration period_duration = rclcpp::Duration::from_nanoseconds(period.count());
      rclcpp::Duration period_error_threshold = rclcpp::Duration::from_nanoseconds(0.1 * period.count()); // 10% threshold

      while(rclcpp::ok()){
        // calculate measured period
        const auto current_time = this->controller_manager_->now();
        const auto measured_period = current_time - previous_time;
        previous_time = current_time;

        // execute update loop
        this->update(current_time, measured_period);

        // wait until we hit the end of the period
        next_iteration_time += period;

        // if(measured_period - period_duration > period_error_threshold){
        //   RCLCPP_WARN_THROTTLE(
        //     logger_,
        //     *node_->get_clock(),
        //     1000, // 1 second throttle
        //     "Measured period (%f s) is larger than expected period (%f s). "
        //     "This can lead to performance issues.",
        //     measured_period.seconds(), period_duration.seconds());
        // }

        if(use_sim_time){ // TODO: check sim time
          this->controller_manager_->get_clock()->sleep_until(current_time + period);
        } else {
          std::this_thread::sleep_until(next_iteration_time);
        }
      }
    }
  );

  // sched_param sched;
  // sched.sched_priority = sched_get_priority_max(SCHED_FIFO);
  // if (pthread_setschedparam(cm_thread_.native_handle(), SCHED_FIFO, &sched) != 0) {
  //   RCLCPP_ERROR(logger_, "Failed to set thread scheduling policy to FIFO");
  // } else {
  //   RCLCPP_INFO(logger_, "Controller manager thread scheduling policy set to FIFO");
  // }

  spin_thread_ = std::thread(
    [this](){
      RCLCPP_INFO(logger_, "Spinning controller manager executor in a separate thread");
      this->cm_executor_->spin();
    }
  );
}


void LeggedRos2Control::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  this->controller_manager_->read(time, period);
  this->controller_manager_->update(time, period);
  this->controller_manager_->write(time, period);
}

void LeggedRos2Control::import_components_(std::vector<hardware_interface::HardwareInfo> &hardware_info, 
                        std::unique_ptr<hardware_interface::ResourceManager> &resource_manager)
{
  // Create the system interface loader
  try{
    system_interface_loader_.reset(new pluginlib::ClassLoader<LeggedSystemInterface>(
      "legged_ros2_control", "legged::LeggedSystemInterface"));
  }catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to create hardware interface loader:  " << ex.what());
    return;
  }

  // Import components according to the hardware info
  for(const auto & hw_info: hardware_info){
    std::string hw_class_type = hw_info.hardware_class_type;
    LeggedSystemInterface::UniquePtr system_interface;
    try{
      system_interface = LeggedSystemInterface::UniquePtr(
        system_interface_loader_->createUnmanagedInstance(hw_class_type));
    }catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to create system interface for " << hw_class_type << ": " << ex.what());
      continue;
    }

    resource_manager->import_component(std::move(system_interface), hw_info);

    rclcpp_lifecycle::State state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, 
      hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager->set_component_state(hw_info.name, state);
  }
}



} // namespace legged