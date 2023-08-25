cmake_minimum_required(VERSION 3.5)
project(ranger_msgs)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_file
"msg/ActuatorState.msg"
"msg/ActuatorStateArray.msg"
"msg/DriverState.msg"
"msg/MotionState.msg"
"msg/MotorState.msg"
"msg/SystemState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_file}
  DEPENDENCIES 
    builtin_interfaces std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()