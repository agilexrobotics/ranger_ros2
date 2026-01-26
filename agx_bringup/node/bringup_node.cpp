#include "rclcpp/rclcpp.hpp"
#include "bring_up.h"
#include "global_state.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 使用无参构造函数
  auto bring_up = std::make_shared<agx::nav::BringUp>();

  // 初始化全局状态
  auto &global_state = GlobalState::getInstance();
  global_state.initialize(bring_up);

  RCLCPP_INFO(bring_up->get_logger(), "启动AGX底盘通信模块");

  rclcpp::spin(bring_up);
  rclcpp::shutdown();

  return 0;
}