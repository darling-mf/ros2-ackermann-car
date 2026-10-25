cmake_minimum_required(VERSION 3.16)
project(r5_steering_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ackermann_steering_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(r5_steering_controller SHARED
  src/r5_steering_controller.cpp
)
target_compile_features(r5_steering_controller PUBLIC cxx_std_17)
target_include_directories(r5_steering_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/r5_steering_controller>)
ament_target_dependencies(r5_steering_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(r5_steering_controller PRIVATE "R5_STEERING_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface r5_steering_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/r5_steering_controller
)

install(
  TARGETS r5_steering_controller
  EXPORT export_r5_steering_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(FILES STAGE1_GUIDE.md EXPERIMENT_LOG.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_r5_steering_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
