cmake_minimum_required(VERSION 3.8)
project(legged_ros2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(legged_ros2_control REQUIRED)
find_package(generate_parameter_library REQUIRED)


generate_parameter_library(
  static_controller_parameters
  src/static_controller.yaml
)


add_library(legged_ros2_controller SHARED
  src/legged_ros2_controller.cpp
  src/separate_joint_controller.cpp
  src/static_controller.cpp
)
target_compile_features(legged_ros2_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(legged_ros2_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(legged_ros2_controller PUBLIC
  static_controller_parameters
)
ament_target_dependencies(
  legged_ros2_controller PUBLIC
  "rclcpp"
  "controller_interface"
  "rclcpp_lifecycle"
  "realtime_tools"
  "legged_ros2_control"
  "pluginlib"
  "sensor_msgs"
)

pluginlib_export_plugin_description_file(controller_interface legged_ros2_controller_plugins.xml)

install(
  TARGETS legged_ros2_controller static_controller_parameters
  EXPORT export_legged_ros2_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  legged_ros2_controller
)

ament_export_include_directories(
  include
)

ament_package()
