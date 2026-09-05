cmake_minimum_required(VERSION 3.8)
project(camera_pan_tilt_calibration)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(camera_calibration_capture_panel SHARED
  src/capture_panel.cpp
  include/camera_pan_tilt_calibration/capture_panel.hpp
)
target_include_directories(camera_calibration_capture_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_calibration_capture_panel
  pluginlib
  rclcpp
  rviz_common
  std_msgs
)
target_link_libraries(camera_calibration_capture_panel Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS camera_calibration_capture_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS
  scripts/capture_node.py
  solve_arm_camera_intrinsics.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_package()
