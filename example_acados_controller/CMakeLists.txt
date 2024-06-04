cmake_minimum_required(VERSION 3.5)
project(example_acados_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-copy)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(DEPENDENCIES
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  Eigen3
  tf2_eigen
)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# add controller library
add_library(example_acados_controller
  SHARED
  src/example_acados_controller.cpp
)
target_include_directories(example_acados_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/example_acados_controller>
)
ament_target_dependencies(
  example_acados_controller
  PUBLIC ${DEPENDENCIES}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(example_acados_controller PRIVATE "example_acados_controller_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(example_acados_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)

# install
install(
  DIRECTORY include/
  DESTINATION include/example_acados_controller
)

install(TARGETS example_acados_controller
  EXPORT export_example_acados_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_example_acados_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()