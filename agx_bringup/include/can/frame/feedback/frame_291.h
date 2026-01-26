#ifndef CAN_FRAME_FEEDBACK_FRAME_291_H
#define CAN_FRAME_FEEDBACK_FRAME_291_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/msg/motion_mode_feedback.hpp"
#include <memory>
#include <string>

namespace agx::nav {

class Frame291 : public BaseFrame
{
public:
  Frame291();
  ~Frame291() = default;
  virtual void parse(const struct can_frame& frame);

private:
  std::string motion_mode(uint8_t mode);
  std::string switching(uint8_t sw);
  std::string drive_mode(uint8_t drive_mode_val);
  
  rclcpp::Publisher<agx_bringup::msg::MotionModeFeedback>::SharedPtr m_motion_pub;
  int m_max_count;
  bool m_publisher_initialized{false};
};

} // namespace agx::nav

#endif