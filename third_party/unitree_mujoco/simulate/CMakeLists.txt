cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

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


file(GLOB SIM_SRC
    src/joystick/joystick.cc
    src/mujoco/*.cc
    src/unitree_sdk2_bridge/*.cc)

set(SIM_DEPENDENCIES
    pthread
    ${MUJOCO_LIB}
    glfw
    yaml-cpp
    unitree_sdk2)


add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco ${SIM_DEPENDENCIES})
target_include_directories(unitree_mujoco PUBLIC
    ${MUJOCO_INCLUDE_DIR}  
)
ament_target_dependencies(unitree_mujoco
  ament_cmake
  rclcpp
  std_msgs
)

set(CMAKE_BUILD_TYPE Release)

install(TARGETS unitree_mujoco
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch ../unitree_robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

