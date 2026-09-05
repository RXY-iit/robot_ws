cmake_minimum_required(VERSION 3.8)
project(touch_anything)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(touch_anything_rviz_panel SHARED
  src/touch_panel.cpp
  include/touch_anything/touch_panel.hpp
)
target_include_directories(touch_anything_rviz_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(touch_anything_rviz_panel
  pluginlib rclcpp rviz_common sensor_msgs std_msgs
)
target_link_libraries(touch_anything_rviz_panel Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS touch_anything_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS
  scripts/touch_observation_node.py
  scripts/touch_kinematics.py
  scripts/approach_geometry.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES plugin_description.xml README.md DESTINATION share/${PROJECT_NAME})

ament_package()
