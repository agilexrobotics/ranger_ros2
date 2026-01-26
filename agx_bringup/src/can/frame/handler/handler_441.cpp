#include "can/frame/handler/handler_441.h"

namespace agx::nav
{

  Handler441::Handler441()
  {
  }

  Handler441::~Handler441()
  {
  }

  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为{
   *   "inst":
   * }
   */
  void Handler441::handle(const nlohmann::json &data)
  {
    try
    {
      auto socket_can = SocketCan::get_instance();
      if (!socket_can || !socket_can->is_initialized())
      {
        RCLCPP_ERROR(rclcpp::get_logger("handler_111"), "SocketCan未初始化");
        return;
      }
      if (!socket_can || !socket_can->is_initialized())
      {
        RCLCPP_ERROR(rclcpp::get_logger("handler_111"), "SocketCan未初始化");
        return;
      }
      int8_t inst = data["inst"];
      struct can_frame frame = {0};
      get_can_frame(frame, agx::nav::Instruction::CLEAR_FAULT, inst, zero, zero, zero, zero, zero, zero, zero);
      socket_can->send_frame(frame);
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("handler_441"), "写441帧失败");
    }
  }

}
