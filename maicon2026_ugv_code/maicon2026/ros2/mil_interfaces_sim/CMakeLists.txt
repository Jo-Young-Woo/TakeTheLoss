cmake_minimum_required(VERSION 3.8)
project(mil_interfaces_sim)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg" "msg/Match.msg" "msg/OledLine.msg"
  "srv/Gimbal.srv" "srv/Fire.srv")
ament_export_dependencies(rosidl_default_runtime)
ament_package()

