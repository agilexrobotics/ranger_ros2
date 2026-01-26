#ifndef CAN_FRAME_FEEDBACK_FRAME_25X_H
#define CAN_FRAME_FEEDBACK_FRAME_25X_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/msg/motor_high_array.hpp"
#include <map>
#include <mutex>
#include <atomic>
#include <memory>

namespace agx::nav {

class Frame25x : public BaseFrame
{
public:
  Frame25x();
  ~Frame25x() = default;
  virtual void parse(const struct can_frame& frame);
private:
  void check_and_publish_complete_set();
  void reset_collection();
  void json_to_motor_high(const nlohmann::json& json_data, agx_bringup::msg::MotorHigh& motor_high);
  
  rclcpp::Publisher<agx_bringup::msg::MotorHighArray>::SharedPtr m_motor_array_pub;   
  int m_max_count;
  
  // 数据收集相关
  std::mutex m_data_mutex;
  std::map<int, nlohmann::json> m_motor_data;  
  std::atomic<int> m_received_count{0};
  bool m_publisher_initialized{false};
};

} // namespace agx::nav

#endif