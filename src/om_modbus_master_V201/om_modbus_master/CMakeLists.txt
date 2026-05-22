cmake_minimum_required(VERSION 3.8)

# pakage name
project(om_modbus_master)

# use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# show all warning
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Werror)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(om_msgs REQUIRED)
find_package(my_messages REQUIRED)

add_executable(om_modbusRTU_node
  src/om_node.cpp
  src/om_ros_message.cpp
  src/om_base.cpp
  src/om_first_gen.cpp
  src/om_second_gen.cpp
  src/om_broadcast.cpp
  src/om_idshare_mode.cpp
)

ament_target_dependencies(om_modbusRTU_node rclcpp std_msgs om_msgs)

include_directories(include)

# Install Launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(
  TARGETS om_modbusRTU_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY include/om_modbus_master
  DESTINATION include
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

# amentのリソースインデクスにパッケージを登録する
ament_package()
