//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: handler_121.h
// Version: 1.0.00
//
// Create:  Jun.19 2025
// Author:  youguo.chu@agilex.ai
// Summary: 灯光控制指令
//-----------------------------------------------------------------------------

#ifndef HANDLER_121_H_
#define HANDLER_121_H_

#include "can/chassis_status.h"
#include "can/frame/handler/base_handler.h"

namespace agx::nav {

/**
 * 灯光控制指令
 */
class Handler121 : public BaseHandler
{
public:
  Handler121();
  ~Handler121();
  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为 {
   *   "enable":,   // 0无效1使能
   *   "value":     // 0常关1常开
   * }
   */
  virtual void handle(const nlohmann::json& data);

};

}

#endif
