# Try to find Unitree SDK2 lib
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_library(unitree_system_interface SHARED 
  g1/g1_system_interface.cpp
  go2/go2_system_interface.cpp
)
target_compile_features(unitree_system_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(unitree_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(unitree_system_interface ${THIS_PACKAGE_DEPENDS})
target_link_libraries(unitree_system_interface unitree_sdk2 legged_system_interface)

add_subdirectory(g1)
add_subdirectory(go2)

install(
  TARGETS unitree_system_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)