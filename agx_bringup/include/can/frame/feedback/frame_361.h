#ifndef CAN_FRAME_FEEDBACK_FRAME_361_H
#define CAN_FRAME_FEEDBACK_FRAME_361_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/msg/bms_status.hpp"
#include <memory>

namespace agx::nav {

class Frame361 : public BaseFrame
{
public:
  Frame361();
  ~Frame361() = default;
  virtual void parse(const struct can_frame& frame);

private:
  double get_data(int8_t high, int8_t low);
  
  rclcpp::Publisher<agx_bringup::msg::BMSStatus>::SharedPtr m_bms_pub;
  int m_max_count;
  bool m_publisher_initialized{false};
};

} // namespace agx::nav

#endif