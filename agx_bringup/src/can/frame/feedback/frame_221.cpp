#include "can/frame/feedback/frame_221.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav
{

  Frame221::Frame221()
  {
    m_max_count = 50;
    m_publisher_initialized = false;
  }

  void Frame221::parse(const struct can_frame &frame)
  {
    try
    {
      // 延迟初始化发布者
      if (!m_publisher_initialized && m_node)
      {
        m_chassis_pub = m_node->create_publisher<agx_bringup::msg::ChassisMotion>(
            "/chassis_motion_feedback",
            rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
        m_publisher_initialized = true;

        if (m_node)
        {
          RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化底盘运动状态发布者");
        }
      }
      auto &gs = GlobalState::getInstance();

      // 解析线速度 (单位: m/s)
      int8_t speed_high = static_cast<int8_t>(frame.data[0]);
      int8_t speed_low = static_cast<int8_t>(frame.data[1]);
      double linear_speed = get_data(speed_high, speed_low) / 1000.0;

      // 解析旋转角速度 (单位: rad/s)
      int8_t rotate_angle_high = static_cast<int8_t>(frame.data[2]);
      int8_t rotate_angle_low = static_cast<int8_t>(frame.data[3]);
      double rotate_angular = get_data(rotate_angle_high, rotate_angle_low) / 1000.0;

      // 解析转向角 (单位: rad)
      int8_t angle_high = static_cast<int8_t>(frame.data[6]);
      int8_t angle_low = static_cast<int8_t>(frame.data[7]);
      double steering_angle = get_data(angle_high, angle_low) / 1000.0;

      // 创建自定义消息
      auto chassis_msg = agx_bringup::msg::ChassisMotion();
      
      // 设置运动数据
      chassis_msg.linear_speed = linear_speed;
      chassis_msg.rotate_angular = rotate_angular;
      chassis_msg.steering_angle = steering_angle;

      gs.setLinearVelocity(linear_speed);
      gs.setAngularVelocity(rotate_angular);
      gs.setSteeringAngle(steering_angle);

      // 发布自定义消息
      m_chassis_pub->publish(chassis_msg);

      if (m_node)
      {
        RCLCPP_DEBUG(m_node->get_logger(),
                     "[BU] => 发布底盘运动数据: 线速度=%.3f m/s, 角速度=%.3f rad/s, 转向角=%.3f rad",
                     linear_speed, rotate_angular, steering_angle);
      }
    }
    catch (const std::exception &e)
    {
      if (m_node)
      {
        RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析221帧失败: %s", e.what());
      }
    }
    catch (...)
    {
      if (m_node)
      {
        RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析221帧失败: 未知错误");
      }
    }
  }

  double Frame221::get_data(int8_t high, int8_t low)
  {
    return (static_cast<int16_t>(high) << 8) | (static_cast<uint16_t>(low) & 0xFF);
  }

} // namespace agx::nav