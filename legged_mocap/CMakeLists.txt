cmake_minimum_required(VERSION 3.8)
project(legged_mocap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(NOKOV_SDK_ARCH_DIR aarch64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  set(NOKOV_SDK_ARCH_DIR armhf)
else()
  set(NOKOV_SDK_ARCH_DIR x64)
endif()

set(NOKOV_SDK_LIBRARY
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libmotioncapture/deps/nokov_sdk/lib/${NOKOV_SDK_ARCH_DIR}/libnokov_sdk.so
)
get_filename_component(NOKOV_SDK_LIBRARY_DIR ${NOKOV_SDK_LIBRARY} DIRECTORY)

set(LIBMOTIONCAPTURE_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(LIBMOTIONCAPTURE_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libmotioncapture
  ${CMAKE_CURRENT_BINARY_DIR}/libmotioncapture
)

add_executable(mocap_pose_node src/mocap_pose_node.cpp)
target_include_directories(mocap_pose_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libmotioncapture/include
)
ament_target_dependencies(mocap_pose_node
  geometry_msgs
  rclcpp
)
target_link_libraries(mocap_pose_node
  Eigen3::Eigen
  libmotioncapture
)
set_target_properties(mocap_pose_node PROPERTIES
  BUILD_RPATH "${NOKOV_SDK_LIBRARY_DIR}"
  INSTALL_RPATH "$ORIGIN/.."
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

install(TARGETS mocap_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${NOKOV_SDK_LIBRARY}
  DESTINATION lib
)

install(DIRECTORY launch
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

ament_package()
