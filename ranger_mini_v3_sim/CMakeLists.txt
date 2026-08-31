cmake_minimum_required(VERSION 3.8)
project(ranger_mini_v3_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
