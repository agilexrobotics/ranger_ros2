#include "can/frame/feedback/frame_26x.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav {

Frame26x::Frame26x()
{
  m_max_count = 10;
  
  // 初始化1-8号电机数据存储
  for (int i = 1; i <= 8; ++i) {
    m_motor_data[i] = nlohmann::json::object();
  }
  
  m_received_count = 0;
  m_publisher_initialized = false;
}

void Frame26x::parse(const struct can_frame& frame)
{
  try
  {
    // 检查是否是0x261-0x268帧
    if (frame.can_id < 0x261 || frame.can_id > 0x268) {
      return;
    }
    
    // 计算电机编号 (0x261->1, 0x262->2, ..., 0x268->8)
    int motor_id = frame.can_id - 0x260;
    
    std::lock_guard<std::mutex> lock(m_data_mutex);
    
    nlohmann::json motor_result;
    motor_driver_low(frame, motor_result);
    
    // 添加电机基本信息
    motor_result["motor_id"] = motor_id;
    motor_result["can_id"] = frame.can_id;
    
    // 检查是否是新数据
    bool had_previous_data = false;
    if (!m_motor_data[motor_id].empty() && 
        m_motor_data[motor_id].contains("motor_id")) {
      had_previous_data = true;
    }
    
    // 存储JSON数据
    m_motor_data[motor_id] = motor_result;
    
    if (!had_previous_data) {
      m_received_count++;
    }
    
    if (m_node) {
      RCLCPP_DEBUG(m_node->get_logger(), "[BU] => 26x电机%d数据，已收集: %d/8", 
                  motor_id, m_received_count.load());
    }
    
    // 检查是否收集完整
    if (m_received_count >= 8) {
      check_and_publish_complete_set();
    }
  }
  catch (const std::exception& e)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析26x帧0x%x失败: %s", 
                  frame.can_id, e.what());
    }
  }
  catch (...)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析26x帧0x%x失败: 未知错误", 
                  frame.can_id);
    }
  }
}

void Frame26x::check_and_publish_complete_set()
{
  if (m_received_count >= 8) {
    try {
      // 延迟初始化发布者
      if (!m_publisher_initialized && m_node) {
        m_motor_array_pub = m_node->create_publisher<agx_bringup::msg::MotorLowArray>(
          "/motor_low_speed_feedback", 
          rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
        m_publisher_initialized = true;
        
        if (m_node) {
          RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化电机低速反馈发布者");
        }
      }
      
      if (!m_motor_array_pub) {
        return;
      }
      
      auto motor_array = agx_bringup::msg::MotorLowArray();
      
      for (int i = 1; i <= 8; ++i) {
        if (!m_motor_data[i].empty()) {
          agx_bringup::msg::MotorLow motor_low;
          json_to_motor_low(m_motor_data[i], motor_low);
          motor_array.motors.push_back(motor_low);
        }
      }
      
      m_motor_array_pub->publish(motor_array);
      
      if (m_node) {
        RCLCPP_DEBUG(m_node->get_logger(), "[BU] => 发布26x电机低速数据，数量: %zu", 
                    motor_array.motors.size());
      }
            
    } catch (const std::exception& e) {
      if (m_node) {
        RCLCPP_ERROR(m_node->get_logger(), "[BU] => 发布26x数据失败: %s", e.what());
      }
    }
    
    reset_collection();
  }
}

void Frame26x::json_to_motor_low(const nlohmann::json& json_data, agx_bringup::msg::MotorLow& motor_low)
{
  // 从JSON转换为自定义消息
  motor_low.motor_id = json_data.value("motor_id", 0);
  motor_low.can_id = json_data.value("can_id", 0);
  motor_low.driver_vol = json_data.value("driverVol", 0.0f) * 0.1f;
  motor_low.driver_temp = json_data.value("driverTemp", 0.0f);
  motor_low.motor_temp = json_data.value("motorTemp", 0.0f);
  motor_low.driver_status = json_data.value("driverStatus", 0);
  
  // 从driver_status中提取各个状态位
  uint8_t driver_status = json_data.value("driverStatus", 0);
  motor_low.enable_status = (driver_status >> 6) & 1;
  motor_low.error_status = (driver_status >> 5) & 1;
  motor_low.sensor_status = (driver_status >> 4) & 1;
  motor_low.over_temp_driver = (driver_status >> 3) & 1;
  motor_low.over_current = (driver_status >> 2) & 1;
  motor_low.over_temp_motor = (driver_status >> 1) & 1;
  motor_low.under_voltage = driver_status & 1;
}

void Frame26x::reset_collection()
{
  m_received_count = 0;
  for (int i = 1; i <= 8; ++i) {
    m_motor_data[i] = nlohmann::json::object();
  }
}

} // namespace agx::nav