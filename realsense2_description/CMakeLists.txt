cmake_minimum_required(VERSION 3.10)
project(realsense2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


