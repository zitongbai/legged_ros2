find_package(fmt REQUIRED)

add_executable(go2_node
  go2_node.cpp
  ../../legged_ros2_control.cpp
  ../../legged_system_interface.cpp
)
target_compile_features(go2_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(go2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(go2_node ${THIS_PACKAGE_DEPENDS})
target_link_libraries(go2_node
  legged_system_interface
  unitree_sdk2
  fmt::fmt
)

install(
  TARGETS go2_node
  DESTINATION lib/${PROJECT_NAME}
)