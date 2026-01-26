//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: handler_111.h
// Version: 1.0.00
//
// Create:  Feb.21 2025
// Author:  youguo.chu@agilex.ai
// Summary: 运动控制
//-----------------------------------------------------------------------------

#ifndef HANDLER_111_H_
#define HANDLER_111_H_

#include "can/chassis_status.h"
#include "can/frame/handler/base_handler.h"

namespace agx::nav {

/**
 * 运动控制帧
 */
class Handler111 : public BaseHandler
{
public:
  Handler111();
  ~Handler111();
  /**
   * @brief 处理消息报文
   * @param[in] data 消息格式为{
   *   "linear": ,
   *   "angular":  
   * }
   */
  virtual void handle(const nlohmann::json& data);

  double CalculateSteeringAngle(const nlohmann::json &data,
                                double &radius);
};

}

#endif
