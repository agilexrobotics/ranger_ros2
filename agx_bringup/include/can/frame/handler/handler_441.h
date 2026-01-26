//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: handler_441.h
// Version: 1.0.00
//
// Create:  Jun.19 2025
// Author:  youguo.chu@agilex.ai
// Summary: 清除故障指令
//-----------------------------------------------------------------------------

#ifndef HANDLER_441_H_
#define HANDLER_441_H_

#include "can/chassis_status.h"
#include "can/frame/handler/base_handler.h"

namespace agx::nav {

/**
 * 清除故障指令
 */
class Handler441 : public BaseHandler
{
public:
  Handler441();
  ~Handler441();
  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为{
   *   "inst": 
   * }
   */
  virtual void handle(const nlohmann::json& data);

};

}

#endif
