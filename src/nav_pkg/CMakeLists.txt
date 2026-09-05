cmake_minimum_required(VERSION 3.8)
project(nav_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joy_source_arbiter_node.py
  scripts/nav_mode_switch_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # The host has a broken third-party typeguard pytest entry point.  The Joy
  # arbiter tests use no external pytest plugins, so isolate them from global
  # plugin auto-loading and use the ROS-compatible system Python explicitly.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_joy_source_arbiter
    test/test_joy_source_arbiter.py
    PYTHON_EXECUTABLE /usr/bin/python3
  )
endif()

ament_package()
