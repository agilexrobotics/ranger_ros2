/*
 * ranger_base_node.cpp
 *
 * Created on: Oct 15, 2021 16:20
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "ranger_base/ranger_base_ros.hpp"

using namespace westonrobot;

std::shared_ptr<RangerBaseRos> robot;

void DetachRobot(int signal) {
  (void)signal;
  robot->Stop();
}

int main(int argc, char **argv) {
  // setup ROS node
  rclcpp::init(argc, argv);
  //   std::signal(SIGINT, DetachRobot);

  robot = std::make_shared<RangerBaseRos>("ranger");
  while (true) {
    // robot->Initialize();

    std::cout << "Robot initialized, start running ..." << std::endl;
    robot->Run();
  }

  return 0;
}