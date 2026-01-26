#include "can/frame/feedback/frame_231.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav {

Frame231::Frame231()
{
  m_max_count = 50;
  m_publisher_initialized = false;
}

void Frame231::parse(const struct can_frame& frame)
{
  try
  {
    // 延迟初始化发布者
    if (!m_publisher_initialized && m_node) {
      m_light_pub = m_node->create_publisher<agx_bringup::msg::LightControlStatus>(
        "/light_control_status", 
        rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
      m_publisher_initialized = true;
      
      if (m_node) {
        RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化灯光控制状态发布者");
      }
    }
    
    if (!m_light_pub) {
      return;
    }
    
    int8_t enable = static_cast<int8_t>(frame.data[0]);
    int8_t mode = static_cast<int8_t>(frame.data[1]);
    
    // 创建灯光控制状态消息
    auto light_msg = agx_bringup::msg::LightControlStatus();
    
    // 设置使能状态
    light_msg.enable = enable;
    if (enable == 0) {
      light_msg.enable_text = "Disabled";
    } else if (enable == 1) {
      light_msg.enable_text = "Enabled";
    } else {
      light_msg.enable_text = "Unknown";
    }
    
    // 设置灯光模式
    light_msg.mode = mode;
    if (mode == 0) {
      light_msg.mode_text = "Normally Off";
    } else if (mode == 1) {
      light_msg.mode_text = "Normally On";
    } else {
      light_msg.mode_text = "Unknown";
    }
    
    // 发布灯光控制状态消息
    m_light_pub->publish(light_msg);
    
    if (m_node) {
      RCLCPP_DEBUG(m_node->get_logger(), 
                  "[BU] => 发布灯光控制状态: 使能=%d(%s), 模式=%d(%s)", 
                  enable, light_msg.enable_text.c_str(),
                  mode, light_msg.mode_text.c_str());
    }
  }
  catch (const std::exception& e)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析231帧失败: %s", e.what());
    }
  }
  catch (...)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析231帧失败: 未知错误");
    }
  }
}

} // namespace agx::nav