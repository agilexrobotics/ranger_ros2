#ifndef CAN_FRAME_FEEDBACK_FRAME_241_H
#define CAN_FRAME_FEEDBACK_FRAME_241_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/msg/remote_control_status.hpp"
#include <memory>

namespace agx::nav {

class Frame241 : public BaseFrame
{
public:
  Frame241();
  ~Frame241() = default;
  virtual void parse(const struct can_frame& frame);

private:
  rclcpp::Publisher<agx_bringup::msg::RemoteControlStatus>::SharedPtr m_remote_pub;
  int m_max_count;
  bool m_publisher_initialized{false};
};

} // namespace agx::nav

#endif