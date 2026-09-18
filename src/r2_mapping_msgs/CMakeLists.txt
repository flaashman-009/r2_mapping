cmake_minimum_required(VERSION 3.8)
project(r2_mapping_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TopicHealth.msg"
  "msg/TfHealth.msg"
  "msg/MappingHealth.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

