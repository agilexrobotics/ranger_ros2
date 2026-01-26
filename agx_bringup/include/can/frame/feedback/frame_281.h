#ifndef CAN_FRAME_FEEDBACK_FRAME_281_H
#define CAN_FRAME_FEEDBACK_FRAME_281_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/msg/wheel_speeds.hpp"
#include <memory>

namespace agx::nav {

class Frame281 : public BaseFrame
{
public:
  Frame281();
  ~Frame281() = default;
  virtual void parse(const struct can_frame& frame);

private:
  double get_data(int8_t high, int8_t low);
  
  rclcpp::Publisher<agx_bringup::msg::WheelSpeeds>::SharedPtr m_wheel_speeds_pub;
  int m_max_count;
  bool m_publisher_initialized{false};
};

} // namespace agx::nav

#endif