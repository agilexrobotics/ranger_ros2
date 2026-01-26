#include "can/frame/feedback/frame_281.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav {

Frame281::Frame281()
{
  m_max_count = 50;
  m_publisher_initialized = false;
}

void Frame281::parse(const struct can_frame& frame)
{
  try
  {
    // 延迟初始化发布者
    if (!m_publisher_initialized && m_node) {
      m_wheel_speeds_pub = m_node->create_publisher<agx_bringup::msg::WheelSpeeds>(
        "/wheel_speeds", 
        rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
      m_publisher_initialized = true;
      
      if (m_node) {
        RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化轮速发布者");
      }
    }
    
    if (!m_wheel_speeds_pub) {
      return;
    }
    
    // 解析转向速度数据
    int8_t speed_0 = static_cast<int8_t>(frame.data[0]);
    int8_t speed_1 = static_cast<int8_t>(frame.data[1]);
    int8_t speed_2 = static_cast<int8_t>(frame.data[2]);
    int8_t speed_3 = static_cast<int8_t>(frame.data[3]);
    int8_t speed_4 = static_cast<int8_t>(frame.data[4]);
    int8_t speed_5 = static_cast<int8_t>(frame.data[5]);
    int8_t speed_6 = static_cast<int8_t>(frame.data[6]);
    int8_t speed_7 = static_cast<int8_t>(frame.data[7]);

    // 创建轮速消息
    auto speed_msg = agx_bringup::msg::WheelSpeeds();
    
    // 转换速度数据 (单位: mm/s -> m/s)
    speed_msg.wheel_01 = get_data(speed_0, speed_1) / 1000.0;
    speed_msg.wheel_02 = get_data(speed_2, speed_3) / 1000.0;
    speed_msg.wheel_03 = get_data(speed_4, speed_5) / 1000.0;
    speed_msg.wheel_04 = get_data(speed_6, speed_7) / 1000.0;

    // 发布轮速消息
    m_wheel_speeds_pub->publish(speed_msg);
    
    if (m_node) {
      RCLCPP_DEBUG(m_node->get_logger(), 
                  "[BU] => 发布轮速: 01=%.3f m/s, 02=%.3f m/s, 03=%.3f m/s, 04=%.3f m/s",
                  speed_msg.wheel_01, speed_msg.wheel_02,
                  speed_msg.wheel_03, speed_msg.wheel_04);
    }
  }
  catch (const std::exception& e)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析281帧失败: %s", e.what());
    }
  }
  catch (...)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析281帧失败: 未知错误");
    }
  }
}

double Frame281::get_data(int8_t high, int8_t low)
{
  return (static_cast<int16_t>(high) << 8) | (static_cast<uint16_t>(low) & 0xFF);
}

} // namespace agx::nav