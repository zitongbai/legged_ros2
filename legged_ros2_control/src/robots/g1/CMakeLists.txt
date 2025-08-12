
add_executable(g1_node
  g1_node.cpp
  ../../legged_ros2_control.cpp
)
target_compile_features(g1_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(g1_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(g1_node ${THIS_PACKAGE_DEPENDS})
target_link_libraries(g1_node
  unitree_system_interface
)

install(
  TARGETS g1_node
  DESTINATION lib/${PROJECT_NAME}
)