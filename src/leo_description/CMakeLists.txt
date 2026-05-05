cmake_minimum_required(VERSION 3.8)
project(leo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    models
    urdf
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()