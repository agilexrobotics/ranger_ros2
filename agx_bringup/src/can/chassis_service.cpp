#include "can/chassis_service.h"

namespace agx::nav
{

  ChassisService::ChassisService(rclcpp::Node* node): m_node(node),
                                                      m_can_handler(std::make_shared<CanHandler>())
  {
  }

  ChassisService::~ChassisService()
  {
  }

  void ChassisService::launch()
  {
    RCLCPP_INFO(m_node->get_logger(), "[BU] => 启动底盘操作服务");

    // 创建订阅者，使用回调函数
    m_sub_cmd_vel = m_node->create_subscription<geometry_msgs::msg::Twist>(
        "/sub_cmd_vel",
        1,
        std::bind(&ChassisService::sub_cmd_vel_callback, this, std::placeholders::_1));
  }

  /** 订阅发送给底盘的数据 */
  void ChassisService::sub_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    json data = {
        {"linear_x", msg->linear.x},
        {"linear_y", msg->linear.y},
        {"angular", msg->angular.z}};
    if (m_can_handler)
    {
      m_can_handler->handle(agx::nav::Instruction::CTRL_MOTION, data);
    }
    else
    {
      RCLCPP_ERROR(m_node->get_logger(), "[BU] => CanHandler未初始化");
    }
  }

}