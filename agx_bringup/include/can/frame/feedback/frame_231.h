#ifndef CAN_FRAME_FEEDBACK_FRAME_231_H
#define CAN_FRAME_FEEDBACK_FRAME_231_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/msg/light_control_status.hpp"
#include <memory>

namespace agx::nav {

class Frame231 : public BaseFrame
{
public:
  Frame231();
  ~Frame231() = default;
  virtual void parse(const struct can_frame& frame);

private:
  rclcpp::Publisher<agx_bringup::msg::LightControlStatus>::SharedPtr m_light_pub;
  int m_max_count;
  bool m_publisher_initialized{false};
};

} // namespace agx::nav

#endif