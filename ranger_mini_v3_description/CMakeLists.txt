cmake_minimum_required(VERSION 3.8)
project(ranger_mini_v3_description)

find_package(ament_cmake REQUIRED)

# Ship every asset the robot description, RViz, and launch system need.
install(
  DIRECTORY urdf meshes rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
