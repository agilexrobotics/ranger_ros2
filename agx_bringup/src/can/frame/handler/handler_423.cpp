#include "can/frame/handler/handler_423.h"

namespace agx::nav
{

  Handler423::Handler423()
  {
  }

  Handler423::~Handler423()
  {
  }

  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为{
   *   "mode":     // 0电流 1电压
   * }
   */
  void Handler423::handle(const nlohmann::json &data)
  {
    try
    {
      auto socket_can = SocketCan::get_instance();
      if (!socket_can || !socket_can->is_initialized())
      {
        RCLCPP_ERROR(rclcpp::get_logger("handler_111"), "SocketCan未初始化");
        return;
      }
      int8_t mode = data["mode"];
      if (mode != 0 || mode != 1)
      {
        RCLCPP_ERROR(rclcpp::get_logger("handler_423"), "[BU] => 控制模式设置错误，不是0或1");
        return;
      }
      struct can_frame frame = {0};
      get_can_frame(frame, agx::nav::Instruction::CTRL_SWITCH, mode, zero, zero, zero, zero, zero, zero, zero);
      socket_can->send_frame(frame);
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("handler_423"), "写423帧失败");
    }
  }

}
