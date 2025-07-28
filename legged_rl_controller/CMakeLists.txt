cmake_minimum_required(VERSION 3.8)
project(legged_rl_controller)

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
find_package(legged_ros2_control REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(legged_ros2_controller REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_package(generate_parameter_library REQUIRED)

get_filename_component(LIBTORCH_ABSOLUTE_PATH "../third_party/libtorch" ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH "${LIBTORCH_ABSOLUTE_PATH}")
find_package(Torch REQUIRED)

generate_parameter_library(
  legged_rl_controller_parameters
  src/legged_rl_controller.yaml
)

add_library(legged_rl_controller SHARED
  src/legged_rl_controller.cpp
)
target_compile_features(legged_rl_controller PUBLIC c_std_99 cxx_std_17)
target_include_directories(legged_rl_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(legged_rl_controller PUBLIC
  ${TORCH_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  legged_rl_controller_parameters
)
set_target_properties(legged_rl_controller PROPERTIES
    INSTALL_RPATH "${LIBTORCH_ABSOLUTE_PATH}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)
ament_target_dependencies(
  legged_rl_controller PUBLIC
  rclcpp
  pluginlib
  hardware_interface
  controller_interface
  rclcpp_lifecycle
  legged_ros2_control
  realtime_tools
  sensor_msgs
  legged_ros2_controller
)

pluginlib_export_plugin_description_file(controller_interface legged_rl_controller_plugins.xml)

install(
  TARGETS legged_rl_controller legged_rl_controller_parameters
  EXPORT export_legged_rl_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
  legged_rl_controller
)

ament_package()
