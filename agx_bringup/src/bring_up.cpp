#include "bring_up.h"

namespace agx::nav
{

  BringUp::BringUp() : Node("agx_bringup")
  {
    // 使用 this Node 创建 ChassisService
    m_chassis_service = std::make_shared<ChassisService>(this);
    m_chassis_service->launch();

    // 启动底盘数据通信
    // 使用单例模式获取SocketCan实例，传入当前节点指针
    auto socket_can = SocketCan::get_instance(this);

    // 检查SocketCan是否初始化成功
    if (!socket_can || !socket_can->is_initialized())
    {
      RCLCPP_ERROR(this->get_logger(), "[BU] => SocketCan初始化失败，无法启动底盘服务");
      return;
    }
    m_socket_can = socket_can;
    m_socket_can->launch();
  }

  /**
   * @brief 析构函数，关闭底盘通信模块
   *
   * 在对象销毁时自动调用，负责清理底盘通信相关资源，
   * 并记录关闭日志信息。
   */
  BringUp::~BringUp()
  {
    RCLCPP_INFO(this->get_logger(), "[BU] => 关闭底盘通信模块");
  }

} // namespace agx::nav