#include "can/frame/feedback/base_frame.h"

namespace agx::nav {

BaseFrame::BaseFrame()
{}

BaseFrame::~BaseFrame()
{}

/**
 * @brief 获取两字节拼接数据
 * @param[in] high 高八位
 * @param[in] low  低八位
 */
int16_t BaseFrame::get_data(int8_t& high, int8_t& low)
{
  return (int16_t)((low & 0xff) | (high << 8));
}

/**
 * @brief 获取四字节拼接数据
 * @param[in] high_h 最高八位
 * @param[in] high_l 次低八位
 * @param[in] low_h  次低八位
 * @param[in] low_l  最低八位
 */
int32_t BaseFrame::get_data(int8_t& high_h, int8_t& high_l, int8_t& low_h, int8_t& low_l)
{
  return (int32_t)((low_l & 0xff) | (low_h << 8) | (high_l << 16) | (high_h << 24));
}

/**
 * @brief 电机驱动器高速信息反馈
 */
void BaseFrame::motor_driver_high(const struct can_frame& frame, json& result)
{
  int8_t rpm_l = static_cast<int8_t>(frame.data[0]);
  int8_t rpm_h = static_cast<int8_t>(frame.data[1]);
  int8_t current_l = static_cast<int8_t>(frame.data[2]);
  int8_t current_h = static_cast<int8_t>(frame.data[3]);
  int8_t pluse_7 = static_cast<int8_t>(frame.data[7]);
  int8_t pluse_6 = static_cast<int8_t>(frame.data[6]);
  int8_t pluse_5 = static_cast<int8_t>(frame.data[5]);
  int8_t pluse_4 = static_cast<int8_t>(frame.data[4]);
  result["rpm"] = get_data(rpm_l, rpm_h);   // 单位RPM
  result["current"] = get_data(current_l, current_h);   // 单位0.1A
  result["pulse"] = get_data(pluse_7, pluse_6, pluse_5, pluse_4);   // 单位脉冲数
}

/**
 * @brief 电机驱动器低速信息反馈
 */
void BaseFrame::motor_driver_low(const struct can_frame& frame, json& result)
{
  int8_t vol_l = static_cast<int8_t>(frame.data[0]);
  int8_t vol_h = static_cast<int8_t>(frame.data[1]);
  int8_t temp_l = static_cast<int8_t>(frame.data[2]);
  int8_t temp_h = static_cast<int8_t>(frame.data[3]);
  int8_t status = static_cast<int8_t>(frame.data[4]);
  result["driverVol"] = get_data(vol_l, vol_h);   // 驱动器电压 单位0.1V
  result["driverTemp"] = get_data(temp_l, temp_h);   // 驱动器温度 单位度
  result["motorTemp"] = static_cast<int8_t>(frame.data[4]);   // 电机温度 单位度
  
  // 驱动器状态
  json driver_status;
  json bit6;
  std::string bit6_item = "驱动器使能状态";
  int8_t bit6_value = (status >> 6) & 1;
  std::string bit6_text = ((status >> 6) & 1) == 0 ? "失能" : "使能";
  get_item(bit6_item, bit6_value, bit6_text, bit6);
  driver_status.push_back(bit6);

  json bit5;
  std::string bit5_item = "驱动器错误状态";
  int8_t bit5_value = (status >> 5) & 1;
  std::string bit5_text = ((status >> 5) & 1) == 0 ? "正常" : "错误";
  get_item(bit5_item, bit5_value, bit5_text, bit5);
  driver_status.push_back(bit5);

  json bit4;
  std::string bit4_item = "传感器状态";
  int8_t bit4_value = (status >> 4) & 1;
  std::string bit4_text = ((status >> 4) & 1) == 0 ? "正常" : "异常";
  get_item(bit4_item, bit4_value, bit4_text, bit4);
  driver_status.push_back(bit4);

  json bit3;
  std::string bit3_item = "驱动器是否过温";
  int8_t bit3_value = (status >> 3) & 1;
  std::string bit3_text = ((status >> 3) & 1) == 0 ? "正常" : "过温";
  get_item(bit3_item, bit3_value, bit3_text, bit3);
  driver_status.push_back(bit3);

  json bit2;
  std::string bit2_item = "驱动器是否过流";
  int8_t bit2_value = (status >> 2) & 1;
  std::string bit2_text = ((status >> 2) & 1) == 0 ? "正常" : "过流";
  get_item(bit2_item, bit2_value, bit2_text, bit2);
  driver_status.push_back(bit2);

  json bit1;
  std::string bit1_item = "电机是否过温";
  int8_t bit1_value = (status >> 1) & 1;
  std::string bit1_text = ((status >> 1) & 1) == 0 ? "正常" : "过温";
  get_item(bit1_item, bit1_value, bit1_text, bit1);
  driver_status.push_back(bit1);

  json bit0;
  std::string bit0_item = "电源电压是否过低";
  int8_t bit0_value = (status >> 0) & 1;
  std::string bit0_text = ((status >> 0) & 1) == 0 ? "正常" : "过低";
  get_item(bit0_item, bit0_value, bit0_text, bit0);
  driver_status.push_back(bit0);

  result["driverStatus"] = status;
}

void BaseFrame::get_item(const std::string& item, const int8_t& value,
    const std::string& text, json& result)
{
  result["item"] = item;
  result["value"] = value;
  result["text"] = text;
}

void BaseFrame::set_nav_detail_status(int status)
{
  if (!m_node) {
    return;
  }
  
  // 延迟创建发布者
  if (!m_pub_nav_detail_status) {
    m_pub_nav_detail_status = m_node->create_publisher<std_msgs::msg::String>(
      "/nav_detail_status", 10);
  }
  
  int64_t T = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
    
  json data = {
    {"status", status},
    {"T", T}
  };
  
  auto message = std_msgs::msg::String();
  message.data = data.dump();
  m_pub_nav_detail_status->publish(message);
}

}