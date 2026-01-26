#include "can/frame/feedback/frame_211.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav {

Frame211::Frame211()
{
  m_max_count = 50;
  m_publisher_initialized = false;
}

void Frame211::parse(const struct can_frame& frame)
{
  try
  {
    // 延迟初始化发布者
    if (!m_publisher_initialized && m_node) {
      m_json_pub = m_node->create_publisher<agx_bringup::msg::SystemStatus>(
        "/system_status", 
        rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
      m_publisher_initialized = true;
      
      if (m_node) {
        RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化系统状态发布者");
      }
    }
    
    if (!m_json_pub) {
      return;
    }
    
    // 解析状态
    nlohmann::json status_result;
    uint8_t status = static_cast<uint8_t>(frame.data[0]);
    get_status(status, status_result);
    
    // 解析控制模式
    nlohmann::json ctrl_mode_result;
    uint8_t ctrl_mode = static_cast<uint8_t>(frame.data[1]);
    get_ctrl_mode(ctrl_mode, ctrl_mode_result);
    
    // 解析电压
    int8_t vol_high = static_cast<int8_t>(frame.data[2]);
    int8_t vol_low = static_cast<int8_t>(frame.data[3]);
    double vol = get_vol(vol_high, vol_low);

    // 构建结果JSON
    nlohmann::json result = {
      { "status", status_result },
      { "ctrlMode", ctrl_mode_result },
      { "vol", vol }
    };
    
    // 解析错误明细
    int8_t f5 = static_cast<int8_t>(frame.data[5]);
    int8_t f6 = static_cast<int8_t>(frame.data[6]);
    int8_t f7 = static_cast<int8_t>(frame.data[7]);
    get_fault(f5, f6, f7, result);

    // 将JSON转换为自定义消息
    auto msg = agx_bringup::msg::SystemStatus();
    
    // 转换状态
    if (result.contains("status") && result["status"].is_object()) {
      auto& s = result["status"];
      if (s.contains("value")) msg.status = s["value"];
      if (s.contains("text")) msg.status_text = s["text"];
    }
    
    // 转换控制模式
    if (result.contains("ctrlMode") && result["ctrlMode"].is_object()) {
      auto& c = result["ctrlMode"];
      if (c.contains("value")) msg.ctrl_mode = c["value"];
      if (c.contains("text")) msg.ctrl_mode_text = c["text"];
    }
    
    // 转换电压
    if (result.contains("vol")) msg.vol = result["vol"];
    
    // 转换故障信息
    if (result.contains("faults") && result["faults"].is_array()) {
      for (auto& f : result["faults"]) {
        agx_bringup::msg::FaultBit fault;
        if (f.contains("item")) fault.item = f["item"];
        if (f.contains("value")) fault.value = f["value"];
        if (f.contains("text")) fault.text = f["text"];
        msg.faults.push_back(fault);
      }
    }
    
    // 发布自定义消息
    m_json_pub->publish(msg);
    
    if (m_node) {
      RCLCPP_DEBUG(m_node->get_logger(), "[BU] => 发布系统状态消息");
    }
  }
  catch (const std::exception& e)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析211帧失败: %s", e.what());
    }
  }
  catch (...)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析211帧失败: 未知错误");
    }
  }
}

double Frame211::get_vol(int8_t& high, int8_t& low)
{
  int16_t vol_raw = (static_cast<int16_t>(high) << 8) | (static_cast<uint16_t>(low) & 0xFF);
  return vol_raw / 100.0;
}

void Frame211::get_ctrl_mode(uint8_t& ctrl_mode, nlohmann::json& result)
{
  std::string text = "Unknown";
  if (ctrl_mode == agx::nav::CtrlMode::STANDBY) {
    text = "Standby Mode";
  }
  else if (ctrl_mode == agx::nav::CtrlMode::CAN) {
    text = "CAN Mode";
  }
  else if (ctrl_mode == agx::nav::CtrlMode::SERIAL) {
    text = "Serial Mode";
  }
  else if (ctrl_mode == agx::nav::CtrlMode::REMOTE_CTRL) {
    text = "Remote Control Mode";
  }
  get_value_text(ctrl_mode, text, result);
}

void Frame211::get_status(uint8_t status, nlohmann::json& result)
{
  std::string text = "Unknown";
  if (status == agx::nav::Status::NORMAL) {
    text = "Normal";
  }
  else if (status == agx::nav::Status::E_STOP) {
    text = "E-Stop Pressed";
  }
  else if (status == agx::nav::Status::ERROR) {
    text = "Error";
  }
  get_value_text(status, text, result);
}

void Frame211::get_fault(int8_t& f5, int8_t& f6, int8_t& f7, nlohmann::json& result)
{
  nlohmann::json faults;
  std::string item;
  int8_t value;
  std::string text;
  
  // 第5个字节
  for (int i = 0; i < 8; ++i) {
    nlohmann::json fault_item;
    switch(i) {
      case 0: item = "FrontRight Steering Zero Calibration"; break;
      case 1: item = "RearRight Steering Zero Calibration"; break;
      case 2: item = "RearLeft Steering Zero Calibration"; break;
      case 3: item = "FrontLeft Steering Zero Calibration"; break;
      case 4: item = "Steering Calibration Timeout"; break;
      case 5: item = "Reserved"; break;
      case 6: item = "Reserved"; break;
      case 7: item = "Reserved"; break;
    }
    value = (f5 >> i) & 1;
    get_text(value, text);
    get_item(item, value, text, fault_item);
    faults.push_back(fault_item);
  }
  
  // 第6个字节
  for (int i = 0; i < 8; ++i) {
    nlohmann::json fault_item;
    switch(i) {
      case 0: item = "Driver Status "; break;
      case 1: item = "Reserved"; break;
      case 2: item = "Motor 5 Driver "; break;
      case 3: item = "Motor 6 Driver "; break;
      case 4: item = "Motor 7 Driver "; break;
      case 5: item = "Motor 8 Driver "; break;
      case 6: item = "Over Temperature Protection"; break;
      case 7: item = "Over Current Protection"; break;
    }
    value = (f6 >> i) & 1;
    get_text(value, text);
    get_item(item, value, text, fault_item);
    faults.push_back(fault_item);
  }
  
  // 第7个字节
  for (int i = 0; i < 8; ++i) {
    nlohmann::json fault_item;
    switch(i) {
      case 0: item = "Battery Under Voltage "; break;
      case 1: item = "Over Voltage Protection"; break;
      case 2: item = "Remote Control Lost Connection"; break;
      case 3: item = "Motor 1 Driver Communication "; break;
      case 4: item = "Motor 2 Driver Communication "; break;
      case 5: item = "Motor 3 Driver Communication "; break;
      case 6: item = "Motor 4 Driver Communication "; break;
      case 7: item = "Emergency Stop"; break;
    }
    value = (f7 >> i) & 1;
    get_text(value, text);
    get_item(item, value, text, fault_item);
    faults.push_back(fault_item);
  }
  
  result["faults"] = faults;
}

void Frame211::get_value_text(uint8_t& value, std::string& text, nlohmann::json& result)
{
  result = {
    { "value", value },
    { "text", text }
  };
}

void Frame211::get_text(int8_t& value, std::string& text)
{
  if (value == 0) {
    text = "Normal";
  } else if (value == 1) {
    text = "Fault";
  } else {
    text = "Unknown";
  }
}

void Frame211::get_item(std::string& item, int8_t& value, std::string& text, nlohmann::json& result)
{
  result = {
    { "item", item },
    { "value", value },
    { "text", text }
  };
}

} // namespace agx::nav