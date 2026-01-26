#ifndef CAN_FRAME_FEEDBACK_FRAME_271_H
#define CAN_FRAME_FEEDBACK_FRAME_271_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/msg/steering_angles.hpp"
#include <memory>

namespace agx::nav {

class Frame271 : public BaseFrame
{
public:
  Frame271();
  ~Frame271() = default;
  virtual void parse(const struct can_frame& frame);

private:
  double get_data(int8_t high, int8_t low);
  
  rclcpp::Publisher<agx_bringup::msg::SteeringAngles>::SharedPtr m_steering_pub;
  int m_max_count;
  bool m_publisher_initialized{false};
};

} // namespace agx::nav

#endif