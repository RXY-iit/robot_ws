cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk_examples)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Build
add_executable(read_write_node src/read_write_node.cpp)
ament_target_dependencies(read_write_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  geometry_msgs
  rclcpp
)
add_executable(cmd_vel_to_set_position src/cmd_vel_to_set_position.cpp)
ament_target_dependencies(cmd_vel_to_set_position
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  geometry_msgs
  rclcpp
)

add_executable(pub_position_node src/pub_position_node.cpp)
ament_target_dependencies(pub_position_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)
# add_executable(tri_read_write_node src/tri_read_write_node.cpp)
# ament_target_dependencies(tri_read_write_node
#   dynamixel_sdk_custom_interfaces
#   dynamixel_sdk
#   rclcpp
# )
add_executable(get_present_position src/get_present_position.cpp)
ament_target_dependencies(get_present_position
  rclcpp 
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
)

# Install
install(TARGETS
  read_write_node
  cmd_vel_to_set_position
  get_present_position
  pub_position_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS
#   tri_read_write_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()