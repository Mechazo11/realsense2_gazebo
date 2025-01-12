cmake_minimum_required(VERSION 3.10)
project(realsense2_gz)

find_package(ament_cmake REQUIRED)
find_package(realsense2_description)

install(DIRECTORY launch urdf world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


