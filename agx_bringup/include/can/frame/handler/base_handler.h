#ifndef BASE_HANDLER_H
#define BASE_HANDLER_H

#include <linux/can.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include "can/socket_can.h"

namespace agx::nav
{

  constexpr int8_t zero = 0;

  class BaseHandler
  {
  public:
    BaseHandler();
    virtual ~BaseHandler();
    virtual void handle(const nlohmann::json &data) = 0;

  protected:
    void get_can_frame(struct can_frame &frame, uint16_t instruction_id,
                       int8_t bit0, int8_t bit1, int8_t bit2, int8_t bit3, int8_t bit4,
                       int8_t bit5, int8_t bit6, int8_t bit7);
  };

} // namespace agx::nav

#endif