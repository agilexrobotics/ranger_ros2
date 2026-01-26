#ifndef CAN_FRAME_FEEDBACK_FRAME_221_H
#define CAN_FRAME_FEEDBACK_FRAME_221_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/msg/chassis_motion.hpp"
#include <memory>

namespace agx::nav {

class Frame221 : public BaseFrame
{
public:
  Frame221();
  ~Frame221() = default;
  virtual void parse(const struct can_frame& frame);

private:
  double get_data(int8_t high, int8_t low);
  
  rclcpp::Publisher<agx_bringup::msg::ChassisMotion>::SharedPtr m_chassis_pub; 
  int m_max_count;
  bool m_publisher_initialized{false};
};

} // namespace agx::nav

#endif