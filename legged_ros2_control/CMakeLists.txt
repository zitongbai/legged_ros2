cmake_minimum_required(VERSION 3.8)
project(legged_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
find_package(urdf REQUIRED)
find_package(glfw3 REQUIRED)

# Try to find mujoco
find_package(mujoco QUIET)
if(mujoco_FOUND)
  message(STATUS "Mujoco build from source has been found")
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
elseif(DEFINED ENV{MUJOCO_DIR})
  message(STATUS "Mujoco build from source has not been found. Attempting to find the binary in $ENV{MUJOCO_DIR} instead.")
  find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "Failed to find binary in $ENV{MUJOCO_DIR}")
  endif()
  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
else()
  message(FATAL_ERROR "Failed to find mujoco with find_package.
  Either build and install mujoco from source or set the MUJOCO_DIR environment variable to tell CMake where to find the binary install. ")
endif()

set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  hardware_interface
  pluginlib
  controller_manager
  urdf
  glfw3
)

add_library(legged_system_interface src/legged_system_interface.cpp)
target_compile_features(legged_system_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(legged_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  legged_system_interface
  "rclcpp"
  "hardware_interface"
  "pluginlib"
  "rclcpp_lifecycle"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(legged_system_interface PRIVATE "LEGGED_ROS2_CONTROL_BUILDING_LIBRARY")


add_library(mujoco_system_interface SHARED src/mujoco/mujoco_system_interface.cpp)
target_compile_features(mujoco_system_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(mujoco_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}  
)
ament_target_dependencies(mujoco_system_interface ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_system_interface ${MUJOCO_LIB} legged_system_interface)


# Export hardware plugins
pluginlib_export_plugin_description_file(legged_ros2_control legged_ros2_control_plugins.xml)


add_executable(mujoco_node
  src/mujoco/mujoco_node.cpp
  src/mujoco/mujoco_ros2_control.cpp
  src/mujoco/mujoco_cameras.cpp
  src/mujoco/mujoco_rendering.cpp
  src/legged_ros2_control.cpp
)
target_compile_features(mujoco_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(mujoco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)
ament_target_dependencies(mujoco_node ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_node
  ${MUJOCO_LIB}
  mujoco_system_interface
  glfw
)


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS legged_system_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS mujoco_system_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS mujoco_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  legged_system_interface
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
