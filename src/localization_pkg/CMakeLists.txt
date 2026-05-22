cmake_minimum_required(VERSION 3.8)
project(localization_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(livox_custom_to_pc2_node
  src/livox_custom_to_pc2_node.cpp
)
ament_target_dependencies(livox_custom_to_pc2_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/gicp_localizer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gicp_localizer_node
)

install(TARGETS livox_custom_to_pc2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/livox_custom_to_pc2_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME livox_custom_to_pc2_node_py
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
