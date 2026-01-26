#ifndef BRING_UP_H_
#define BRING_UP_H_

#include "rclcpp/rclcpp.hpp"
#include "can/chassis_service.h"
#include "can/socket_can.h"

namespace agx::nav
{

  class BringUp : public rclcpp::Node
  {
  public:
    BringUp();
    ~BringUp();

  private:
    std::shared_ptr<ChassisService> m_chassis_service;
    std::shared_ptr<SocketCan> m_socket_can;
  };

}

#endif