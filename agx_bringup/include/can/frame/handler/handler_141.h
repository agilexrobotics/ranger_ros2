//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: handler_141.h
// Version: 1.0.00
//
// Create:  Jun.19 2025
// Author:  youguo.chu@agilex.ai
// Summary: 运动模型切换指令
//-----------------------------------------------------------------------------

#ifndef HANDLER_141_H_
#define HANDLER_141_H_

#include "can/chassis_status.h"
#include "can/frame/handler/base_handler.h"

namespace agx::nav {

/**
 * 运动模型切换指令
 */
class Handler141 : public BaseHandler
{
public:
  Handler141();
  ~Handler141();
  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为{
   *   "model": 
   * }
   */
  virtual void handle(const nlohmann::json& data);

};

}

#endif
