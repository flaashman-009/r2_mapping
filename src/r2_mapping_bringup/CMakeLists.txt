cmake_minimum_required(VERSION 3.8)
project(r2_mapping_bringup)

find_package(ament_cmake REQUIRED)

# launch / config / rviz 都是运行时资源，直接用 symlink-install 装到 share/
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

