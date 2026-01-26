cmake_minimum_required(VERSION 3.8)
project(agx_bringup)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

###########################################################
# 生成 ROS2 接口（消息）
###########################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorLow.msg"
  "msg/MotorHigh.msg"
  "msg/MotorLowArray.msg"
  "msg/MotorHighArray.msg"
  "msg/FaultBit.msg"
  "msg/SystemStatus.msg"
  "msg/SystemStatusArray.msg"
  "msg/ChassisMotion.msg"
  "msg/LightControlStatus.msg"
  "msg/RemoteControlStatus.msg"
  "msg/SteeringAngles.msg"
  "msg/WheelSpeeds.msg"
  "msg/MotionModeFeedback.msg"
  "msg/FrontWheelOdometry.msg"
  "msg/BackWheelOdometry.msg"
  "msg/BMSStatus.msg"
  "msg/BMSAlarmWarning.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

###########################################################
# 创建可执行文件
###########################################################
add_executable(${PROJECT_NAME}_node
  node/bringup_node.cpp
  src/bring_up.cpp

  src/can/chassis_status.cpp
  src/can/chassis_service.cpp
  src/can/socket_can.cpp

  # CAN frame feedback
  src/can/frame/feedback/base_frame.cpp
  src/can/frame/feedback/frame_211.cpp
  src/can/frame/feedback/frame_221.cpp
  src/can/frame/feedback/frame_231.cpp
  src/can/frame/feedback/frame_241.cpp
  src/can/frame/feedback/frame_25x.cpp
  src/can/frame/feedback/frame_26x.cpp
  src/can/frame/feedback/frame_271.cpp
  src/can/frame/feedback/frame_281.cpp
  src/can/frame/feedback/frame_291.cpp
  src/can/frame/feedback/frame_311.cpp
  src/can/frame/feedback/frame_312.cpp
  src/can/frame/feedback/frame_361.cpp
  src/can/frame/feedback/frame_362.cpp
  src/can/frame/feedback/frame_371.cpp

  src/can/frame/handler/base_handler.cpp
  src/can/frame/handler/can_handler.cpp
  src/can/frame/handler/handler_111.cpp
  src/can/frame/handler/handler_121.cpp
  src/can/frame/handler/handler_141.cpp
  src/can/frame/handler/handler_423.cpp
  src/can/frame/handler/handler_441.cpp
  src/can/frame/handler/handler_461.cpp
)

###########################################################
# 头文件 include/
###########################################################
target_include_directories(${PROJECT_NAME}_node PUBLIC
  include
)

###########################################################
# 链接依赖
###########################################################
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_default_runtime
)

###########################################################
# *** 关键：链接 C++ 类型支持 ***
###########################################################
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_node
  ${cpp_typesupport_target}
)

###########################################################
# 安装执行文件
###########################################################
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

###########################################################
# 安装头文件
###########################################################
install(DIRECTORY include/
  DESTINATION include/
)

###########################################################
# 安装 launch 文件
###########################################################
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

###########################################################
# 安装 config 文件
###########################################################
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

###########################################################
# 最终
###########################################################
ament_package()
