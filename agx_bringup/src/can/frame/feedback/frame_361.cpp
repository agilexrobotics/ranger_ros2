#include "can/frame/feedback/frame_361.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav {

Frame361::Frame361()
{
  m_max_count = 50;
  m_publisher_initialized = false;
}

void Frame361::parse(const struct can_frame& frame)
{
  try
  {
    // 延迟初始化发布者
    if (!m_publisher_initialized && m_node) {
      m_bms_pub = m_node->create_publisher<agx_bringup::msg::BMSStatus>(
        "/bms_status", 
        rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
      m_publisher_initialized = true;
      
      if (m_node) {
        RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化BMS状态发布者");
      }
    }
    
    if (!m_bms_pub) {
      return;
    }
    
    // 创建BMS状态消息
    auto bms_msg = agx_bringup::msg::BMSStatus();
    
    // 解析SOC (State of Charge) - 电量百分比
    bms_msg.soc = static_cast<uint8_t>(frame.data[0]);
    
    // 解析SOH (State of Health) - 健康度百分比
    bms_msg.soh = static_cast<uint8_t>(frame.data[1]);
    
    // 解析电压 (单位: 0.01V -> V)
    int8_t vol_high = static_cast<int8_t>(frame.data[2]);
    int8_t vol_low = static_cast<int8_t>(frame.data[3]);
    bms_msg.voltage = get_data(vol_high, vol_low) / 100.0;
    
    // 解析电流 (单位: 0.1A -> A)
    int8_t current_high = static_cast<int8_t>(frame.data[4]);
    int8_t current_low = static_cast<int8_t>(frame.data[5]);
    bms_msg.current = get_data(current_high, current_low) / 10.0;
    
    // 解析温度 (单位: 0.1°C -> °C)
    int8_t temp_high = static_cast<int8_t>(frame.data[6]);
    int8_t temp_low = static_cast<int8_t>(frame.data[7]);
    bms_msg.temperature = get_data(temp_high, temp_low) / 10.0;
    
    // 充电状态 (需要根据实际协议解析，这里假设在data[0]的高位)
    bms_msg.status = (frame.data[0] >> 6) & 0x03;  // 假设状态在SOC字节的高2位
    
    // 设置充电状态文本描述
    switch (bms_msg.status) {
      case 0:
        bms_msg.status_text = "放电";
        break;
      case 1:
        bms_msg.status_text = "充电";
        break;
      case 2:
        bms_msg.status_text = "空闲";
        break;
      case 3:
        bms_msg.status_text = "故障";
        break;
      default:
        bms_msg.status_text = "未知";
        break;
    }
    
    // 发布BMS状态消息
    m_bms_pub->publish(bms_msg);
    
    if (m_node) {
      RCLCPP_DEBUG(m_node->get_logger(), 
                  "[BU] => 发布BMS状态: SOC=%d%%, SOH=%d%%, 电压=%.2fV, 电流=%.1fA, 温度=%.1f°C, 状态=%d(%s)",
                  bms_msg.soc, bms_msg.soh, bms_msg.voltage, 
                  bms_msg.current, bms_msg.temperature, 
                  bms_msg.status, bms_msg.status_text.c_str());
    }
  }
  catch (const std::exception& e)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析361帧失败: %s", e.what());
    }
  }
  catch (...)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析361帧失败: 未知错误");
    }
  }
}

double Frame361::get_data(int8_t high, int8_t low)
{
  return (static_cast<int16_t>(high) << 8) | (static_cast<uint16_t>(low) & 0xFF);
}

} // namespace agx::nav