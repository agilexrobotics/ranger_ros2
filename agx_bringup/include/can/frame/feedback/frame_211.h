#ifndef CAN_FRAME_FEEDBACK_FRAME_211_H
#define CAN_FRAME_FEEDBACK_FRAME_211_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/msg/system_status.hpp"
#include <memory>
#include "can/chassis_status.h"

namespace agx::nav {

class Frame211 : public BaseFrame
{
public:
  Frame211();
  ~Frame211() = default;
  virtual void parse(const struct can_frame& frame);

private:
  double get_vol(int8_t& high, int8_t& low);
  void get_ctrl_mode(uint8_t& ctrl_mode, nlohmann::json& result);
  void get_status(uint8_t status, nlohmann::json& result);
  void get_fault(int8_t& f5, int8_t& f6, int8_t& f7, nlohmann::json& result);
  void get_value_text(uint8_t& value, std::string& text, nlohmann::json& result);
  void get_text(int8_t& value, std::string& text);
  void get_item(std::string& item, int8_t& value, std::string& text, nlohmann::json& result);
  
  rclcpp::Publisher<agx_bringup::msg::SystemStatus>::SharedPtr m_json_pub;
  int m_max_count;
  bool m_publisher_initialized{false};
};

} // namespace agx::nav

#endif