/*
 * ranger_messenger.hpp
 *
 * Created on: Jun 14, 2019 10:24
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef RANGER_MESSENGER_HPP
#define RANGER_MESSENGER_HPP

#include <string>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ranger_base/ranger_params.hpp"

#include "ugv_sdk/mobile_robot/ranger_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include "ranger_msgs/msg/ranger_status.hpp"
#include "ranger_msgs/msg/ranger_setting.hpp"


namespace westonrobot {

// template <typename SystemModel>
// class SystemPropagator {
//  public:
//   asc::state_t Propagate(asc::state_t init_state,
//                          typename SystemModel::control_t u, double t0,
//                          double tf, double dt) {
//     double t = t0;
//     asc::state_t x = init_state;

//     while (t <= tf) {
//       integrator_(SystemModel(u), x, t, dt);
//       // Note: you may need to add additional constraints to [x]
//     }
//     return x;
//   }

//  private:
//   asc::RK4 integrator_;
// };

template <typename RangerType>
class RangerMessenger {
 public:
  RangerMessenger(std::shared_ptr<RangerType> ranger, rclcpp::Node *node)
      : ranger_(ranger), node_(node) {}
  // SystemPropagator<BicycleKinematics> model_;
  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }
  void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }

  void SetSimulationMode(int loop_rate) {
    simulated_robot_ = true;
    sim_control_rate_ = loop_rate;
  }

  void SetupSubscription() {
    // odometry publisher
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    odom_pub_ =
        node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 50);
    status_pub_ = node_->create_publisher<ranger_msgs::msg::RangerStatus>(
        "/ranger_status", 10);

    // cmd subscriber
    motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&RangerMessenger::TwistCmdCallback, this,
                  std::placeholders::_1));
    ranger_setting_sub_= node_->create_subscription<ranger_msgs::msg::RangerSetting>(
      "/ranger_setting",10,std::bind(&RangerMessenger::RangerSettingCbk,this, std::placeholders::_1));
    
  }

  void PublishStateToROS() {
    current_time_ = node_->get_clock()->now();
    uint8_t motion_mode_=0;
    static bool init_run = true;
    if (init_run) {
      last_time_ = current_time_;
      init_run = false;
      return;
    }
    double dt = (current_time_ - last_time_).seconds();

    auto state = ranger_->GetRobotState();
    
    // publish ranger state message
    ranger_msgs::msg::RangerStatus status_msg;

    status_msg.header.stamp = current_time_;
    status_msg.linear_velocity = state.motion_state.linear_velocity;
    status_msg.angular_velocity = state.motion_state.angular_velocity;
    // double phi =ConvertInnerAngleToCentral(state.motion_state.angular_velocity);
    // status_msg.steering_angle = phi;
    status_msg.steering_angle = state.motion_state.steering_angle;

    status_msg.vehicle_state = state.system_state.vehicle_state;
    status_msg.control_mode = state.system_state.control_mode;
    status_msg.error_code = state.system_state.error_code;
    status_msg.battery_voltage = state.system_state.battery_voltage;
    status_msg.motion_mode_state = state.current_motion_mode.motion_mode;
    motion_mode_ = status_msg.motion_mode_state;

    auto actuator = ranger_->GetActuatorState();

    for (int i = 0; i < 8; ++i) {
      // actuator_hs_state
      uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;

      status_msg.actuator_states[motor_id].rpm =
          actuator.actuator_hs_state[i].rpm;
      status_msg.actuator_states[motor_id].current =
          actuator.actuator_hs_state[i].current;
      status_msg.actuator_states[motor_id].pulse_count =
          actuator.actuator_hs_state[i].pulse_count;

      // actuator_ls_state
      motor_id = actuator.actuator_ls_state[i].motor_id;

      status_msg.actuator_states[motor_id].driver_voltage =
          actuator.actuator_ls_state[i].driver_voltage;
      status_msg.actuator_states[motor_id].driver_temperature =
          actuator.actuator_ls_state[i].driver_temp;
      status_msg.actuator_states[motor_id].motor_temperature =
          actuator.actuator_ls_state[i].motor_temp;
      status_msg.actuator_states[motor_id].driver_state =
          actuator.actuator_ls_state[i].driver_state;
    }
    // linear_velocity, angular_velocity, central steering_angle
    double l_v = 0.0, a_v = 0.0, phi = 0.0;
  // x , y direction linear velocity, motion radius
    double x_v = 0.0, y_v = 0.0, radius = 0.0;
    double phi_i = state.motion_state.steering_angle / 180.0 * M_PI;
    // std::cout <<  motion_mode_ << std::endl;
    
       
    switch (motion_mode_) 
    {
      case ranger_msgs::msg::RangerSetting::MOTION_MODE_ACKERMAN: {
        std::cout <<"motion_mode_ :0"<< std::endl;
        l_v = state.motion_state.linear_velocity;
        double r = s / std::tan(std::fabs(phi_i)) + s;
        phi = ConvertInnerAngleToCentral(phi_i);
        if (phi > steer_angle_tolerance) {
          a_v = state.motion_state.linear_velocity / r;
        } else {
          a_v = -state.motion_state.linear_velocity / r;
        }
        x_v = l_v * std::cos(phi);
        if (l_v >= 0.0) {
          y_v = l_v * std::sin(phi);
        } else {
          y_v = l_v * std::sin(-phi);
        }
        radius = r;
        break;
      }
      case ranger_msgs::msg::RangerSetting::MOTION_MODE_SLIDE: {
        std::cout <<"motion_mode_ :1"<< std::endl;
        l_v = state.motion_state.linear_velocity;
        phi = phi_i;
        a_v = 0.0;
        x_v = l_v * std::cos(phi);
        y_v = l_v * std::sin(phi);
        radius = 0.0;
        break;
      }
      case ranger_msgs::msg::RangerSetting::MOTION_MODE_ROUND: {
        std::cout << "motion_mode_ :2"<< std::endl;
        l_v = 0.0;
        phi = std::fabs(phi_i);
        a_v = -2.0 * state.motion_state.linear_velocity /
              (RangerParams::track * M_SQRT2);
        x_v = 0.0;
        y_v = 0.0;
        radius = a_v / state.motion_state.linear_velocity;
        break;
      }
      case ranger_msgs::msg::RangerSetting::MOTION_MODE_SLOPING: {
        std::cout <<  "motion_mode_:3"<< std::endl;
        l_v = -state.motion_state.linear_velocity;
        phi = std::fabs(phi_i);
        a_v = 0.0;
        x_v = 0.0;
        y_v = l_v;
        radius = 0.0;
        break;
      }
    }
    status_msg.linear_velocity = l_v;
    status_msg.angular_velocity = a_v;
    status_msg.lateral_velocity = 0.0;
    status_msg.steering_angle = phi;
    status_msg.x_linear_vel = x_v;
    status_msg.y_linear_vel = y_v;
    status_msg.motion_radius = radius;
    

    status_pub_->publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(l_v, a_v, x_v, y_v, dt);

    // record time for next integration
    last_time_ = current_time_;
  }
  void RangerSettingCbk(const ranger_msgs::msg::RangerSetting::SharedPtr msg) {
    auto mode = msg->motion_mode;
    motion_mode_ = mode;
    switch (mode) {
      case ranger_msgs::msg::RangerSetting::MOTION_MODE_ACKERMAN:{
        ranger_->SetMotionMode(ranger_msgs::msg::RangerSetting::MOTION_MODE_ACKERMAN);
        break;
      }
      case ranger_msgs::msg::RangerSetting::MOTION_MODE_SLIDE:{
        ranger_->SetMotionMode(ranger_msgs::msg::RangerSetting::MOTION_MODE_SLIDE);
        break;
      }
      case ranger_msgs::msg::RangerSetting::MOTION_MODE_ROUND:{
        ranger_->SetMotionMode(ranger_msgs::msg::RangerSetting::MOTION_MODE_ROUND);
        break;
      }
      case ranger_msgs::msg::RangerSetting::MOTION_MODE_SLOPING:{
        ranger_->SetMotionMode(ranger_msgs::msg::RangerSetting::MOTION_MODE_SLOPING);
        break;
      }
      default:
        // ROS_WARN("ranger motion mode not support %d", mode);
        break;
    }
}

 private:
  std::shared_ptr<RangerType> ranger_;
  rclcpp::Node *node_;
  

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  uint8_t motion_mode_=0;
  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  // westonrobot::SystemPropagator<BicycleKinematics> model_;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<ranger_msgs::msg::RangerStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<ranger_msgs::msg::RangerSetting>::SharedPtr ranger_setting_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // speed variables
  double linear_speed_ = 0.0;  // inear velocity
  double angular_vel_ = 0.0;   // angule velocity
  double x_linear_vel_ = 0.0;  // x direction linear velocity
  double y_linear_vel_ = 0.0;  // y direction linear velocity
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  static constexpr double l = RangerParams::wheelbase;
  static constexpr double w = RangerParams::track;
  static constexpr double s = w / 2.0;                    // half of track
  static constexpr double steer_angle_tolerance = 0.005;  // ~+-0.287 degrees


  rclcpp::Time last_time_;
  rclcpp::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    
    if (!simulated_robot_) {
      SetRangerMotionCommand(msg);
    } else {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("Cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }

  void SetRangerMotionCommand(const geometry_msgs::msg::Twist::SharedPtr &msg) {


    double steer_cmd = msg->angular.z;

    switch (motion_mode_) {
    case ranger_msgs::msg::RangerSetting::MOTION_MODE_ACKERMAN: {
      if (steer_cmd > RangerParams::max_steer_angle_central) {
        steer_cmd = RangerParams::max_steer_angle_central;
      }
      if (steer_cmd < -RangerParams::max_steer_angle_central) {
        steer_cmd = -RangerParams::max_steer_angle_central;
      }

      double phi_i = ConvertCentralAngleToInner(steer_cmd);

      double phi_degree = (phi_i / M_PI * 180.0);
      ranger_->SetMotionCommand(msg->linear.x, phi_degree);
      break;
    }
    case ranger_msgs::msg::RangerSetting::MOTION_MODE_SLIDE: {
      if (steer_cmd > RangerParams::max_steer_angle_slide) {
        steer_cmd = RangerParams::max_steer_angle_slide;
      }
      if (steer_cmd < -RangerParams::max_steer_angle_slide) {
        steer_cmd = -RangerParams::max_steer_angle_slide;
      }

      double phi_degree = -(steer_cmd / M_PI * 180.0);
      ranger_->SetMotionCommand(msg->linear.x, phi_degree);
      break;
    }
    case ranger_msgs::msg::RangerSetting::MOTION_MODE_ROUND:
    case ranger_msgs::msg::RangerSetting::MOTION_MODE_SLOPING: {
      ranger_->SetMotionCommand(0.0, 0.0, -(msg->linear.x), 0.0);
      break;
    }
  }
 
  }

  double ConvertCentralAngleToInner(double angle)
  {
    double phi = 0;
    double phi_i = angle;
    if (phi_i > steer_angle_tolerance) {
      // left turn
      double r = l / std::tan(phi_i) + w;
      phi = std::atan(l / r);
    } else if (phi_i < -steer_angle_tolerance) {
      // right turn
      double r = l / std::tan(-phi_i) + w;
      phi = std::atan(l / r);
      phi = -phi;
    }
    return phi;
  }
  double ConvertInnerAngleToCentral(double angle)
  {
    double phi = angle;
    double phi_i = 0;
    if (phi > steer_angle_tolerance) {
      // left turn
      phi_i =
          std::atan(l * std::sin(phi) / (l * std::cos(phi) - w * std::sin(phi)));
    } else if (phi < -steer_angle_tolerance) {
      // right turn
      phi = -phi;
      phi_i =
          std::atan(l * std::sin(phi) / (l * std::cos(phi) - w * std::sin(phi)));
      phi_i = -phi_i;
    }
    return phi_i;
  }

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  void PublishOdometryToROS(double linear, double angle_vel,
                            double x_linear_vel,double y_linear_vel, double dt) {
    linear_speed_ = linear;
    angular_vel_ = angle_vel;
    x_linear_vel_ = x_linear_vel;
    y_linear_vel_ = y_linear_vel;
    double theta = angular_vel_ * dt;

    position_x_ +=
      cos(theta_) * x_linear_vel_ * dt - sin(theta_) * y_linear_vel_ * dt;
    position_y_ +=
      sin(theta_) * x_linear_vel_ * dt + cos(theta_) * y_linear_vel_ * dt;
    theta_ = theta_ + angular_vel_ * dt;

    if (theta_ > M_PI) {
      theta_ -= 2 * M_PI;
    } else if (theta < -M_PI) {
      theta_ += 2 * M_PI;
    }
      
    geometry_msgs::msg::Quaternion odom_quat =
        createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = x_linear_vel_;
    odom_msg.twist.twist.linear.y = y_linear_vel_;
    odom_msg.twist.twist.angular.z = angular_vel_;

    odom_msg.pose.covariance = {
    0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
    0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
    0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
    0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
    0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
    0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};
    odom_msg.twist.covariance = {
    0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
    0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
    0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
    0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
    0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
    0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};

    odom_pub_->publish(odom_msg);
  }
};
}  // namespace westonrobot

#endif /* Ranger_MESSENGER_HPP */
