/*
 * ranger_base_ros.cpp
 *
 * Created on: Oct 15, 2021 14:35
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#include "ranger_base/ranger_base_ros.hpp"
#include "ranger_base/ranger_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
RangerBaseRos::RangerBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter("port_name", rclcpp::ParameterValue("can0"));   //声明参数

  this->declare_parameter("odom_frame", rclcpp::ParameterValue("odom"));
  this->declare_parameter("base_frame", rclcpp::ParameterValue("base_link"));
  this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("odom"));

  this->declare_parameter("is_ranger_mini", rclcpp::ParameterValue(false));
  this->declare_parameter("is_omni_wheel", rclcpp::ParameterValue(false));

  this->declare_parameter("simulated_robot", rclcpp::ParameterValue(false));
  this->declare_parameter("control_rate", rclcpp::ParameterValue(50));

  LoadParameters();
}

void RangerBaseRos::LoadParameters() {
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");//获取参数

  this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_,
                                      "odom");
  this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
  this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;

  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool RangerBaseRos::Initialize() {

  std::cout << "Robot base: Ranger" << std::endl;
  ProtocolDetector detector;
  // if(port_name.find("can") != std::string::npos)
  if (detector.Connect(port_name_)) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      robot_ = std::unique_ptr<RangerBase>();
      std::cout << "Creating interface for Ranger with AGX_V2 Protocol"
                  << std::endl;

    // auto proto = detector.DetectProtocolVersion(5);
    // if (proto == ProtocolVersion::AGX_V2) {
    //   std::cout << "Detected protocol: AGX_V2" << std::endl;
    //   robot_ = std::make_shared<RangerRobot>(ProtocolVersion::AGX_V2);

    // } else {
    //   std::cout << "Detected protocol: UNKONWN" << std::endl;
    //   return false;
    // }
  } else {

    return false;
  }

  return true;
}

void RangerBaseRos::Stop() { keep_running_ = false; }

void RangerBaseRos::Run() {

  robot_ = std::make_shared<RangerBase>();
  // instantiate a ROS messenger
    std::unique_ptr<RangerMessenger<RangerBase>> messenger =
        std::unique_ptr<RangerMessenger<RangerBase>>(
            new RangerMessenger<RangerBase>(robot_, this));
    messenger->SetOdometryFrame(odom_frame_);
    messenger->SetBaseFrame(base_frame_);
    messenger->SetOdometryTopicName(odom_topic_name_);
    if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);
    // connect to robot and setup ROS subscription
    if (port_name_.find("can") != std::string::npos) {
      if (robot_->Connect(port_name_)) {

        robot_->EnableCommandedMode();
        // std::cout << "run:state 6"<< std::endl;
        // std::cout << "EnableCommandedMode" << std::endl;
        std::cout << "Using CAN bus to talk with the robot" << std::endl;
      } else {
        std::cout << "Failed to connect to the robot CAN bus" << std::endl;
        return;
      }
    } else {
      std::cout << "Please check the specified port name is a CAN port"
                << std::endl;
      return;
    }
    
    // publish robot state at 50Hz while listening to twist commands
    messenger->SetupSubscription();
    rclcpp::Rate rate(50);
    keep_running_ = true;
    while (keep_running_) {
      messenger->PublishStateToROS();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    // }
  }
}
}  // namespace westonrobot
