#include "can/frame/handler/handler_141.h"

namespace agx::nav
{

  Handler141::Handler141()
  {
  }

  Handler141::~Handler141()
  {
  }

  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为{
   *   "model":
   * }
   */
  void Handler141::handle(const nlohmann::json &data)
  {
    try
    {
      auto socket_can = SocketCan::get_instance();
      if (!socket_can || !socket_can->is_initialized())
      {
        RCLCPP_ERROR(rclcpp::get_logger("handler_111"), "SocketCan未初始化");
        return;
      }
      int8_t model = data["model"];
      struct can_frame frame = {0};
      get_can_frame(frame, agx::nav::Instruction::MOTION_MODEL, model, zero, zero, zero, zero, zero, zero, zero);
      socket_can->send_frame(frame);
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("handler_141"), "写141帧失败");
    }
  }

}
