#include "can/frame/handler/handler_461.h"

namespace agx::nav
{

  Handler461::Handler461()
  {
  }

  Handler461::~Handler461()
  {
  }

  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为{
   *   "hibernate":
   * }
   */
  void Handler461::handle(const nlohmann::json &data)
  {
    try
    {
      auto socket_can = SocketCan::get_instance();
      if (!socket_can || !socket_can->is_initialized())
      {
        RCLCPP_ERROR(rclcpp::get_logger("handler_111"), "SocketCan未初始化");
        return;
      }
      int32_t hibernate = data["hibernate"];
      int16_t hibernate_high = hibernate >> 8;
      int8_t hibernate_high_1 = hibernate_high >> 8;
      int8_t hibernate_high_2 = hibernate_high & 0xFF;
      int16_t hibernate_low = hibernate & 0xFF;
      int8_t hibernate_low_1 = hibernate_low >> 8;
      int8_t hibernate_low_2 = hibernate_low & 0xFF;

      struct can_frame frame = {0};
      int8_t bit0 = 0x02;
      int8_t bit1 = 0xAE;
      int8_t bit2 = 0;
      int8_t bit3 = 0x08;
      get_can_frame(frame, agx::nav::Instruction::HIBERNATE, bit0, bit1,
                    bit2, bit3, hibernate_high_1, hibernate_high_2, hibernate_low_1, hibernate_low_2);
      socket_can->send_frame(frame);
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("handler_461"), "写461帧失败");
    }
  }

}
