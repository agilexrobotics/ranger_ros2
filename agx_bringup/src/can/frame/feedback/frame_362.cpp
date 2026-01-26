#include "can/frame/feedback/frame_362.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav {

Frame362::Frame362()
{
  m_max_count = 50;
  m_publisher_initialized = false;
}

void Frame362::parse(const struct can_frame& frame)
{
  try
  {
    // 延迟初始化发布者
    if (!m_publisher_initialized && m_node) {
      m_alarm_pub = m_node->create_publisher<agx_bringup::msg::BMSAlarmWarning>(
        "/bms_alarm_warning", 
        rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
      m_publisher_initialized = true;
      
      if (m_node) {
        RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化BMS报警警告发布者");
      }
    }
    
    if (!m_alarm_pub) {
      return;
    }
    
    // 解析报警和警告数据
    uint8_t alarm1 = static_cast<uint8_t>(frame.data[0]);
    uint8_t alarm2 = static_cast<uint8_t>(frame.data[1]);
    uint8_t warn1 = static_cast<uint8_t>(frame.data[2]);
    uint8_t warn2 = static_cast<uint8_t>(frame.data[3]);
    
    // 创建BMS报警警告消息
    auto alarm_msg = agx_bringup::msg::BMSAlarmWarning();
    
    // 设置原始数据
    alarm_msg.alarm1 = alarm1;
    alarm_msg.alarm2 = alarm2;
    alarm_msg.warn1 = warn1;
    alarm_msg.warn2 = warn2;
    
    // 解析具体的报警和警告位
    parse_alarm_warning(alarm1, alarm2, warn1, warn2, alarm_msg);
    
    // 检查是否有报警或警告
    bool has_alarm = false;
    bool has_warning = false;
    
    // 检查报警状态
    if (alarm_msg.over_voltage_alarm || alarm_msg.under_voltage_alarm || 
        alarm_msg.high_temp_alarm || alarm_msg.low_temp_alarm ||
        alarm_msg.discharge_overcurrent_alarm || alarm_msg.charge_overcurrent_alarm) {
      has_alarm = true;
    }
    
    // 检查警告状态
    if (alarm_msg.over_voltage_warning || alarm_msg.under_voltage_warning || 
        alarm_msg.high_temp_warning || alarm_msg.low_temp_warning ||
        alarm_msg.discharge_overcurrent_warning || alarm_msg.charge_overcurrent_warning) {
      has_warning = true;
    }
    
    // 设置总体状态
    alarm_msg.has_alarm = has_alarm;
    alarm_msg.has_warning = has_warning;
    
    // 发布BMS报警警告消息
    m_alarm_pub->publish(alarm_msg);
    
    if (m_node) {
      if (has_alarm) {
        RCLCPP_WARN(m_node->get_logger(), 
                  "[BU] => 发布BMS报警: alarm1=0x%02X, alarm2=0x%02X, warn1=0x%02X, warn2=0x%02X",
                  alarm1, alarm2, warn1, warn2);
      } else if (has_warning) {
        RCLCPP_WARN(m_node->get_logger(), 
                  "[BU] => 发布BMS警告: alarm1=0x%02X, alarm2=0x%02X, warn1=0x%02X, warn2=0x%02X",
                  alarm1, alarm2, warn1, warn2);
      } else {
        RCLCPP_DEBUG(m_node->get_logger(), 
                    "[BU] => 发布BMS状态: 正常");
      }
    }
  }
  catch (const std::exception& e)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析362帧失败: %s", e.what());
    }
  }
  catch (...)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析362帧失败: 未知错误");
    }
  }
}

void Frame362::parse_alarm_warning(uint8_t alarm1, uint8_t alarm2, uint8_t warn1, uint8_t warn2, 
                                  agx_bringup::msg::BMSAlarmWarning& msg)
{
  // 解析报警1 (alarm1)
  msg.over_voltage_alarm = (alarm1 >> 6) & 1;      // BIT1: 过压报警
  msg.under_voltage_alarm = (alarm1 >> 5) & 1;     // BIT2: 欠压报警
  msg.high_temp_alarm = (alarm1 >> 4) & 1;         // BIT3: 高温报警
  msg.low_temp_alarm = (alarm1 >> 3) & 1;          // BIT4: 低温报警
  msg.discharge_overcurrent_alarm = alarm1 & 1;    // BIT7: 放电过流报警
  
  // 解析报警2 (alarm2)
  msg.charge_overcurrent_alarm = (alarm2 >> 7) & 1; // BIT0: 充电过流报警
  
  // 解析警告1 (warn1)
  msg.over_voltage_warning = (warn1 >> 6) & 1;     // BIT1: 过压警告
  msg.under_voltage_warning = (warn1 >> 5) & 1;    // BIT2: 欠压警告
  msg.high_temp_warning = (warn1 >> 4) & 1;        // BIT3: 高温警告
  msg.low_temp_warning = (warn1 >> 3) & 1;         // BIT4: 低温警告
  msg.discharge_overcurrent_warning = warn1 & 1;   // BIT7: 放电过流警告
  
  // 解析警告2 (warn2)
  msg.charge_overcurrent_warning = (warn2 >> 7) & 1; // BIT0: 充电过流警告
}

} // namespace agx::nav