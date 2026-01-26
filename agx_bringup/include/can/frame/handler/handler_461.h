//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: handler_461.h
// Version: 1.0.00
//
// Create:  Feb.24 2025
// Author:  youguo.chu@agilex.ai
// Summary: 休眠控制
//-----------------------------------------------------------------------------

#ifndef HANDLER_461_H_
#define HANDLER_461_H_

#include "can/chassis_status.h"
#include "can/frame/handler/base_handler.h"

namespace agx::nav {

/**
 * 休眠控制帧
 */
class Handler461 : public BaseHandler
{
public:
  Handler461();
  ~Handler461();
  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为{
   *   "hibernate": 
   * }
   */
  virtual void handle(const nlohmann::json& data);

};

}

#endif
