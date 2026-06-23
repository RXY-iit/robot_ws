cmake_minimum_required(VERSION 3.8)
project(omni_mppi_critics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)

# xtensor is a header-only library — locate via pkg-config or direct search
find_package(xtensor QUIET)
if(NOT xtensor_FOUND)
  find_path(XTENSOR_INCLUDE_DIR xtensor/xtensor.hpp
    PATHS /usr/include /usr/local/include)
  if(NOT XTENSOR_INCLUDE_DIR)
    message(FATAL_ERROR "xtensor headers not found — install libxtensor-dev")
  endif()
endif()

add_library(${PROJECT_NAME} SHARED
  src/steering_change_critic.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(xtensor_FOUND)
  target_link_libraries(${PROJECT_NAME} xtensor)
else()
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${XTENSOR_INCLUDE_DIR})
endif()

ament_target_dependencies(${PROJECT_NAME}
  nav2_mppi_controller
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_mppi_controller critics_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES critics_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(nav2_mppi_controller pluginlib)

ament_package()
