//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: handler_423.h
// Version: 1.0.00
//
// Create:  Jun.19 2025
// Author:  youguo.chu@agilex.ai
// Summary: 控制切换指令
//-----------------------------------------------------------------------------

#ifndef HANDLER_423_H_
#define HANDLER_423_H_

#include "can/chassis_status.h"
#include "can/frame/handler/base_handler.h"

namespace agx::nav {

/**
 * 电流控制、电压控制切换控制指令
 */
class Handler423 : public BaseHandler
{
public:
  Handler423();
  ~Handler423();
  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为{
   *   "mode": 
   * }
   */
  virtual void handle(const nlohmann::json& data);

};

}

#endif
