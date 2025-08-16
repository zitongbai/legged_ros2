/**
 * @file unitree_node.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "legged_ros2_control/legged_ros2_control.hpp"
#include "legged_ros2_control/robots/unitree/dds_wrapper/common/Subscription.h"


namespace legged {

template<typename UnitreeSubscriberType>
class UnitreeNode {
public:
  UnitreeNode()
    : node_(std::make_shared<rclcpp::Node>("unitree_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
      legged_control_(std::make_shared<legged::LeggedRos2Control>(node_)),
      cmd_vel_pub_(node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS())),
      cmd_vel_msg_(),
      cmd_vel_scale_{
        node_->get_parameter_or<float>("cmd_vel.scale.lin_vel_x", 1.0),
        node_->get_parameter_or<float>("cmd_vel.scale.lin_vel_y", 1.0),
        node_->get_parameter_or<float>("cmd_vel.scale.ang_vel_z", 1.0)
      },
      unitree_net_if_(node_->get_parameter_or<std::string>("network_interface", "lo")),
      controller_switch_client_(node_->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller")),
      switch_controller_request_(std::make_shared<controller_manager_msgs::srv::SwitchController::Request>())
  {
    RCLCPP_INFO(node_->get_logger(), "Initializing Unitree node...");
    legged_control_->init();

    switch_controller_request_->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    wait_for_service();

    unitree::robot::ChannelFactory::Instance()->Init(0, unitree_net_if_);
    low_state_subscriber_ = std::make_shared<UnitreeSubscriberType>("rt/lowstate");
    RCLCPP_INFO(node_->get_logger(), "Unitree Node initialized with network interface: %s. Waiting for low state messages...", unitree_net_if_.c_str());
    low_state_subscriber_->wait_for_connection();
    RCLCPP_INFO(node_->get_logger(), "Unitree Node is ready to run.");
  }

  void run() {
    rclcpp::Rate rate(100); // 100 Hz

    // print joystick operation instructions
    std::cout << "----------------------------------------------------" << std::endl;
    std::cout << "Joystick Operation Instructions:" << std::endl;
    std::cout << "  LB+A: Switch to RL controller" << std::endl;
    std::cout << "  LB+B: Switch to Static controller" << std::endl;
    std::cout << "  LB+Y: Switch to Joint State Broadcaster only" << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;

    while (rclcpp::ok()) {
      low_state_subscriber_->update();

      cmd_vel_msg_.linear.x = cmd_vel_scale_.lin_vel_x * low_state_subscriber_->joystick.ly();
      cmd_vel_msg_.linear.y = -cmd_vel_scale_.lin_vel_y * low_state_subscriber_->joystick.lx();
      cmd_vel_msg_.angular.z = -cmd_vel_scale_.ang_vel_z * low_state_subscriber_->joystick.rx();
      cmd_vel_pub_->publish(cmd_vel_msg_);

      handle_controller_switch();
      rate.sleep();
    }
    RCLCPP_INFO(node_->get_logger(), "Unitree SDK2 run loop exited.");
    rclcpp::shutdown();
  }

private:
  void wait_for_service() const {
    while (!controller_switch_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(node_->get_logger(), "Waiting for controller_manager/switch_controller service...");
    }
  }

  void handle_controller_switch() {
    if (low_state_subscriber_->joystick.A.on_pressed && 
        low_state_subscriber_->joystick.LB.pressed) {
      set_controller_switch({"rl_controller"}, {"static_controller"}, "Switched to RL controller.");
    } else if (low_state_subscriber_->joystick.B.on_pressed && 
        low_state_subscriber_->joystick.LB.pressed) {
      set_controller_switch({"static_controller"}, {"rl_controller"}, "Switched to Static controller.");
    } else if (low_state_subscriber_->joystick.Y.on_pressed && 
        low_state_subscriber_->joystick.LB.pressed) {
      set_controller_switch({}, {"rl_controller", "static_controller"}, "Switched to Joint State Broadcaster only.");
    }
  }

  void set_controller_switch(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate, const std::string& log_msg) {
    switch_controller_request_->activate_controllers = activate;
    switch_controller_request_->deactivate_controllers = deactivate;
    switch_controller_request_->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    switch_controller_request_->activate_asap = true;
    controller_switch_client_->async_send_request(switch_controller_request_);
    RCLCPP_INFO(node_->get_logger(), "%s", log_msg.c_str());
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<legged::LeggedRos2Control> legged_control_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist cmd_vel_msg_;
  struct CmdVelScale {
    float lin_vel_x = 1.0;
    float lin_vel_y = 1.0;
    float ang_vel_z = 1.0;
  } cmd_vel_scale_;
  std::string unitree_net_if_;
  std::shared_ptr<UnitreeSubscriberType> low_state_subscriber_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_switch_client_;
  std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> switch_controller_request_;
};

}
