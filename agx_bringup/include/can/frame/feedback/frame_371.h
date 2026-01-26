//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: frame_371.h
// Version: 1.0.00
//
// Create:  Apr.11 2025
// Author:  youguo.chu@agilex.ai
// Summary: 371帧解析
//-----------------------------------------------------------------------------

#ifndef FRAME_371_H_
#define FRAME_371_H_

#include <nlohmann/json.hpp>
#include "can/chassis_status.h"
#include "can/frame/feedback/base_frame.h"
#include <memory>

using json = nlohmann::json;

namespace agx::nav {

/**
 * 371帧解析，充电状态数据反馈
 */
class Frame371: public BaseFrame
{
public:
  Frame371();
  ~Frame371();
  virtual void parse(const struct can_frame& frame);

private:
  std::string m_uri;
};

}

#endif