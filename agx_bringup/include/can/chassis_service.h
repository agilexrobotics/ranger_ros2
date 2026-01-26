#ifndef CHASSIS_SERVICE_H_
#define CHASSIS_SERVICE_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include "can/frame/handler/can_handler.h"

using json = nlohmann::json;

namespace agx::nav
{

  /** 底盘操作服务类 */
  class ChassisService
  {
  public:
    ChassisService(rclcpp::Node* node);
    ~ChassisService();
    void launch();

  private:
    /** 订阅发送给底盘的数据 */
    void sub_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  private:
    rclcpp::Node* m_node;
    /** 订阅发送给底盘速度信息 */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub_cmd_vel;
    std::shared_ptr<CanHandler> m_can_handler;
  };

}

#endif