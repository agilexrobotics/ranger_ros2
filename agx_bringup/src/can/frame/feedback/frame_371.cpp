#include "can/frame/feedback/frame_371.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav {

Frame371::Frame371() 
{
}

Frame371::~Frame371()
{}

void Frame371::parse(const struct can_frame& frame)
{
  try
  {
    // 充电状态 0未充电1桩充2线充
    int8_t status = static_cast<int8_t>(frame.data[0]);
    if (status == 1 || status == 2)
    {
      if (m_node) {
        RCLCPP_INFO(m_node->get_logger(), "正在充电中");
      }
      set_nav_detail_status(301);
    }
  }
  catch (const std::exception& e)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析371帧失败: %s", e.what());
    }
  }
  catch (...)
  {
    if (m_node) {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析371帧失败: 未知错误");
    }
  }
}

} // namespace agx::nav