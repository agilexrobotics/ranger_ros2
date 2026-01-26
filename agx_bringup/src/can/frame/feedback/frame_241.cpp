#include "can/frame/feedback/frame_241.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav {

Frame241::Frame241()
{
  m_max_count = 50;
  m_publisher_initialized = false;
}

void Frame241::parse(const struct can_frame& frame)
{
  try
  {
    // 延迟初始化发布者
    if (!m_publisher_initialized && m_node) {
      m_remote_pub = m_node->create_publisher<agx_bringup::msg::RemoteControlStatus>(
        "/remote_control_status", 
        rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
      m_publisher_initialized = true;
      
      if (m_node) {
        RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化遥控器状态发布者");
      }
    }
    
    if (!m_remote_pub) {
      return;
    }
    
    int8_t sw = static_cast<int8_t>(frame.data[0]);
    
    // 创建遥控器状态消息
    auto remote_msg = agx_bringup::msg::RemoteControlStatus();
    
    // 解析开关状态
    remote_msg.swa = static_cast<uint8_t>(sw & 0x03);
    remote_msg.swb = static_cast<uint8_t>((sw >> 2) & 0x03);
    remote_msg.swc = static_cast<uint8_t>((sw >> 4) & 0x03);
    remote_msg.swd = static_cast<uint8_t>((sw >> 6) & 0x03);
    
    // 解析摇杆数据
    remote_msg.right_lr = static_cast<int8_t>(frame.data[1]);
    remote_msg.right_ud = static_cast<int8_t>(frame.data[2]);
    remote_msg.left_ud = static_cast<int8_t>(frame.data[3]);
    remote_msg.left_lr = static_cast<int8_t>(frame.data[4]);
    remote_msg.left_vra = static_cast<int8_t>(frame.data[5]);
    
    // 发布遥控器状态消息
    m_remote_pub->publish(remote_msg);
    
    if (m_node) {
      RCLCPP_DEBUG(m_node->get_logger(), 
                  "[BU] => 发布遥控器状态: SWA=%u, SWB=%u, SWC=%u, SWD=%u, "
                  "右LR=%d, 右UD=%d, 左LR=%d, 左UD=%d, 左VRA=%d",
                  remote_msg.swa, remote_msg.swb, remote_msg.swc, remote_msg.swd,
                  remote_msg.right_lr, remote_msg.right_ud, 
                  remote_msg.left_lr, remote_msg.left_ud, remote_msg.left_vra);
    }
  }
  catch (const std::exception& e)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析241帧失败: %s", e.what());
    }
  }
  catch (...)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析241帧失败: 未知错误");
    }
  }
}

} // namespace agx::nav