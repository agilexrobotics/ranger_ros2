/*
 * ranger_base_ros.hpp
 *
 * Created on: Oct 15, 2022 15:05
 * Description:
 *
 * Copyright (c) 2023 Weston Robot Pte. Ltd.
 */

#ifndef RANGER_BASE_ROS_HPP
#define RANGER_BASE_ROS_HPP

#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ugv_sdk/mobile_robot/ranger_robot.hpp"

namespace westonrobot {
class RangerBaseRos : public rclcpp::Node {
 public:
  RangerBaseRos(std::string node_name);

  bool Initialize();
  void Run();
  void Stop();

 private:
  
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool is_ranger_mini_ = false;
  bool is_omni_wheel_ = false;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;
  
  int version=2;
  bool is_omni_ = false;
  std::shared_ptr<RangerRobot> robot_;
  // std::shared_ptr<RangerMiniOmniRobot> omni_robot_;

  std::atomic<bool> keep_running_;

  void LoadParameters();
};
}  // namespace westonrobot

#endif /* RANGER_BASE_ROS_HPP */
