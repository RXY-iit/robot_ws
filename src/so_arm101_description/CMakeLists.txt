cmake_minimum_required(VERSION 3.8)
project(so_arm101_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)
install(FILES ATTRIBUTION.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
