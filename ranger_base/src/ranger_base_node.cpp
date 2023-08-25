/**
* @file ranger_base_node.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include "ranger_base/ranger_messenger.hpp"

using namespace westonrobot;

void SignalHandler(int s)
{
  printf("Caught signal %d, program exit\n", s);
  exit(EXIT_FAILURE);
}

void controlSingal()
{
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char** argv)
{
  // setup ROS node
  rclcpp::init(argc, argv);
  controlSingal();

  auto node = rclcpp::Node::make_shared("ranger_base_node");
  // instantiate a robot object
  RangerROSMessenger messenger(node);
  messenger.Run();

  return 0;
}
