#include "can/frame/handler/handler_111.h"

namespace agx::nav
{

  Handler111::Handler111()
  {
  }

  Handler111::~Handler111()
  {
  }

  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为{
   *   "linear": ,
   *   "angular":
   * }
   */

  void Handler111::handle(const nlohmann::json &data)
  {
    try
    {
      auto socket_can = SocketCan::get_instance();
      if (!socket_can || !socket_can->is_initialized())
      {
        RCLCPP_ERROR(rclcpp::get_logger("handler_111"), "SocketCan未初始化");
        return;
      }
      double linear_x = data["linear_x"];
      double linear_y = data["linear_y"];

      double angular_d = data["angular"];
      double linear_d = 0.0f;
      double steering_angle = 0.0f;
      double radius = 0.0f;
      int8_t motion_mode = 0;
      // 根据指令内容选择运动模式
      if (linear_y != 0.0)
      {
        if (linear_x == 0.0)
        {
          motion_mode = 0x01; // 平移模式
        }
        else
        {
          motion_mode = 0x01; // 斜移模式
        }
      }
      else
      {
        steering_angle = CalculateSteeringAngle(data, radius);
        if (std::fabs(radius) < 0.5f)
        {
          motion_mode = 0x02; // 自旋模式
        }
        else
        {
          motion_mode = 0x00; // 阿克曼模式
        }
      }

      struct can_frame enable_frame = {0};
      enable_frame.can_id = 0x421;
      enable_frame.can_dlc = 1;
      enable_frame.data[0] = 1;

      socket_can->send_frame(enable_frame);

      struct can_frame mode_frame = {0};
      mode_frame.can_id = 0x141;
      mode_frame.can_dlc = 1;
      mode_frame.data[0] = motion_mode;

      socket_can->send_frame(mode_frame);

      switch (motion_mode)
      {
      case 0x00:
      {
        linear_d = linear_x;

        break;
      }
      case 0x01:
      {
        linear_d = sqrtf(linear_x * linear_x + linear_y * linear_y);
        steering_angle = atan(linear_y / linear_x);

        if (std::signbit(linear_x) && linear_x == 0.0)
        {
          steering_angle = -steering_angle;
        }
        else
        {
          steering_angle = steering_angle;
        }

        if (steering_angle > 1.571)
        {
          steering_angle = 1.571;
        }
        if (steering_angle < -1.571)
        {
          steering_angle = -1.571;
        }
        double vel = linear_x >= 0 ? 1.0 : -1.0;
        linear_d = vel * linear_d;
        break;
      }
      case 0x02:
      {

        break;
      }
      case 0x03:
      {
        break;
      }
      }
      int16_t linear = linear_d * 1000;   // 线速度
      int16_t angular = angular_d * 1000; // 角速度
      int16_t steer = steering_angle * 1000;
      int8_t linear_high = linear >> 8;
      int8_t linear_low = linear & 0xFF;
      int8_t angular_high = angular >> 8;
      int8_t angular_low = angular & 0xFF;
      int8_t steer_high = steer >> 8;
      int8_t steer_low = steer & 0xFF;

      struct can_frame frame = {0};
      get_can_frame(frame, agx::nav::Instruction::CTRL_MOTION, linear_high, linear_low,
                    angular_high, angular_low, zero, zero, steer_high, steer_low);
      socket_can->send_frame(frame);
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("handler_111"), "写111帧失败");
    }
  }

  double Handler111::CalculateSteeringAngle(const nlohmann::json &data,
                                            double &radius)
  {
    // 1. 提取速度值
    double linear = data["linear_x"];
    double angular = data["angular"];

    // 2. 计算转弯半径
    radius = std::fabs(linear / angular);

    // 3. 确定转向方向系数
    int k = (angular * linear) >= 0 ? 1.0 : -1.0;

    // 4. 获取机器人物理参数
    double l = 0.4f;  // 轴距
    double w = 0.36f; // 轮距

    // 5. 计算内侧车轮转向角度
    double phi_i = atan((l / 2) / (radius - w / 2));

    // 6. 调试信息输出

    // 7. 返回带方向的转向角度
    return k * phi_i;
  }

}
