cmake_minimum_required(VERSION 3.16)
project(rinkrover_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  rinkrover_hardware
  SHARED
  hardware/rr_system.cpp
)
target_compile_features(rinkrover_hardware PUBLIC cxx_std_17)
target_include_directories(rinkrover_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/rinkrover_hardware>
)
ament_target_dependencies(
  rinkrover_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface rinkrover_hardware.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/rinkrover_hardware
)
install(
  DIRECTORY launch
  DESTINATION share/rinkrover_hardware
)
install(TARGETS rinkrover_hardware
  EXPORT export_rinkrover_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_rinkrover_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
