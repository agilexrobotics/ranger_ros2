#include "can/frame/feedback/frame_291.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav
{

  Frame291::Frame291()
  {
    m_max_count = 50;
    m_publisher_initialized = false;
  }

  void Frame291::parse(const struct can_frame &frame)
  {
    try
    {
      // 延迟初始化发布者
      if (!m_publisher_initialized && m_node)
      {
        m_motion_pub = m_node->create_publisher<agx_bringup::msg::MotionModeFeedback>(
            "/motion_mode_feedback",
            rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
        m_publisher_initialized = true;

        if (m_node)
        {
          RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化运动模式反馈发布者");
        }
      }

      auto &gs = GlobalState::getInstance();

      // 解析运动模式数据
      uint8_t mode = static_cast<uint8_t>(frame.data[0]);
      uint8_t switch_status = static_cast<uint8_t>(frame.data[1]);
      uint8_t drive_mode_val = static_cast<uint8_t>(frame.data[2]);

      MotionMode gs_mode;
      switch (mode)
      {
      case 0:
        gs_mode = MotionMode::DUAL_ACKERMAN;
        break;
      case 1:
        gs_mode = MotionMode::PARALLEL;
        break;
      case 2:
        gs_mode = MotionMode::SPINNING;
        break;
      case 3:
        gs_mode = MotionMode::PARKING;
        break;
      case 4:
        gs_mode = MotionMode::SIDE_SLIP;
        break;
      default:
        gs_mode = MotionMode::DUAL_ACKERMAN;
      }
      gs.setMotionMode(gs_mode);

      // 创建运动模式反馈消息
      auto motion_msg = agx_bringup::msg::MotionModeFeedback();

      // 设置运动模式
      motion_msg.mode = mode;
      motion_msg.mode_text = motion_mode(mode);

      // 设置切换状态
      motion_msg.switch_status = switch_status;
      motion_msg.switch_text = switching(switch_status);

      // 设置驱动模式
      motion_msg.drive_mode = drive_mode_val;
      motion_msg.drive_mode_text = drive_mode(drive_mode_val);

      // 发布运动模式反馈消息
      m_motion_pub->publish(motion_msg);

      if (m_node)
      {
        RCLCPP_DEBUG(m_node->get_logger(),
                     "[BU] => 发布运动模式反馈: 模式=%d(%s), 切换状态=%d(%s), 驱动模式=%d(%s)",
                     mode, motion_msg.mode_text.c_str(),
                     switch_status, motion_msg.switch_text.c_str(),
                     drive_mode_val, motion_msg.drive_mode_text.c_str());
      }
    }
    catch (const std::exception &e)
    {
      if (m_node)
      {
        RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析291帧失败: %s", e.what());
      }
    }
    catch (...)
    {
      if (m_node)
      {
        RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析291帧失败: 未知错误");
      }
    }
  }

  // 底盘运动模式
  std::string Frame291::motion_mode(uint8_t mode)
  {
    switch (mode)
    {
    case 0:
      return "Front-Rear Ackermann";
    case 1:
      return "Crab Steering";
    case 2:
      return "Spin Turn";
    case 3:
      return "Parking Mode";
    default:
      return "Unknown";
    }
  }

  // 运动模型切换状态
  std::string Frame291::switching(uint8_t sw)
  {
    switch (sw)
    {
    case 0:
      return "Switch Completed";
    case 1:
      return "Switching";
    default:
      return "Unknown";
    }
  }

  // 驱动模式
  std::string Frame291::drive_mode(uint8_t drive_mode_val)
  {
    switch (drive_mode_val)
    {
    case 0:
      return "Current Drive";
    case 1:
      return "Voltage Drive";
    default:
      return "Unknown";
    }
  }

} // namespace agx::nav