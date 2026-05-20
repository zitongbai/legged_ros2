cmake_minimum_required(VERSION 3.8)
project(legged_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(lidar_static_tf_node
  src/lidar_static_tf.cpp
)
target_compile_features(lidar_static_tf_node PUBLIC cxx_std_17)
ament_target_dependencies(lidar_static_tf_node
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(ground_odom_tf_node
  src/ground_odom_tf.cpp
)
target_compile_features(ground_odom_tf_node PUBLIC cxx_std_17)
ament_target_dependencies(ground_odom_tf_node
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  lidar_static_tf_node
  ground_odom_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_launch_test(test/test_lidar_static_tf.launch.py)
  add_launch_test(test/test_lidar_static_tf_nondefault.launch.py)
  add_launch_test(test/test_ground_odom_tf.launch.py)
  add_test(
    NAME test_invert_homogeneous_transform
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_invert_homogeneous_transform.py
  )
endif()

ament_package()
