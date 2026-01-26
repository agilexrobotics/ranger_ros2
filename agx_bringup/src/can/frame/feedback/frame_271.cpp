#include "can/frame/feedback/frame_271.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav {

Frame271::Frame271()
{
  m_max_count = 50;
  m_publisher_initialized = false;
}

void Frame271::parse(const struct can_frame& frame)
{
  try
  {
    // 延迟初始化发布者
    if (!m_publisher_initialized && m_node) {
      m_steering_pub = m_node->create_publisher<agx_bringup::msg::SteeringAngles>(
        "/steering_angles", 
        rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
      m_publisher_initialized = true;
      
      if (m_node) {
        RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化转向角度发布者");
      }
    }
    
    if (!m_steering_pub) {
      return;
    }
    
    // 解析车轮角度数据
    int8_t angle_0 = static_cast<int8_t>(frame.data[0]);
    int8_t angle_1 = static_cast<int8_t>(frame.data[1]);
    int8_t angle_2 = static_cast<int8_t>(frame.data[2]);
    int8_t angle_3 = static_cast<int8_t>(frame.data[3]);
    int8_t angle_4 = static_cast<int8_t>(frame.data[4]);
    int8_t angle_5 = static_cast<int8_t>(frame.data[5]);
    int8_t angle_6 = static_cast<int8_t>(frame.data[6]);
    int8_t angle_7 = static_cast<int8_t>(frame.data[7]);

    // 创建转向角度消息
    auto steering_msg = agx_bringup::msg::SteeringAngles();
    
    // 转换角度数据 (单位: 0.001rad -> rad)
    steering_msg.steering_01 = get_data(angle_0, angle_1) / 1000.0;
    steering_msg.steering_02 = get_data(angle_2, angle_3) / 1000.0;
    steering_msg.steering_03 = get_data(angle_4, angle_5) / 1000.0;
    steering_msg.steering_04 = get_data(angle_6, angle_7) / 1000.0;

    // 发布转向角度消息
    m_steering_pub->publish(steering_msg);
    
    if (m_node) {
      RCLCPP_DEBUG(m_node->get_logger(), 
                  "[BU] => 发布转向角度: 01=%.3f rad, 02=%.3f rad, 03=%.3f rad, 04=%.3f rad",
                  steering_msg.steering_01, steering_msg.steering_02,
                  steering_msg.steering_03, steering_msg.steering_04);
    }
  }
  catch (const std::exception& e)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析271帧失败: %s", e.what());
    }
  }
  catch (...)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析271帧失败: 未知错误");
    }
  }
}

double Frame271::get_data(int8_t high, int8_t low)
{
  return (static_cast<int16_t>(high) << 8) | (static_cast<uint16_t>(low) & 0xFF);
}

} // namespace agx::nav