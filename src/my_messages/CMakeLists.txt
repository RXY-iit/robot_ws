cmake_minimum_required(VERSION 3.8)
project(my_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # std_msgsを追加
find_package(builtin_interfaces REQUIRED)  

set(msg_files
  "msg/SteerMotor.msg"
  "msg/DriveMotor.msg"
)
# set(srv_files
#   "srv/CalcTwoFloats.srv"
# )
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
