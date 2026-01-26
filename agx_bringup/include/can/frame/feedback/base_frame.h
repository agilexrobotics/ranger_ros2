//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: base_frame.h
// Version: 1.0.00
//
// Create:  Feb.20 2025
// Author:  youguo.chu@agilex.ai
// Summary: 基础报文解析
//-----------------------------------------------------------------------------

#ifndef BASE_FRAME_H_
#define BASE_FRAME_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <thread>
#include <map>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include "global_state.hpp"

using json = nlohmann::json;

namespace agx::nav {

/**
 * 基础报文解析
 */
class BaseFrame
{
public:
  BaseFrame();
  virtual ~BaseFrame();
  virtual void parse(const struct can_frame& frame) = 0;

  void set_node(rclcpp::Node *node) { m_node = node; }

protected:
  /**
   * @brief 获取两字节拼接数据
   * @param[in] high 高八位
   * @param[in] low  低八位
   */
  int16_t get_data(int8_t& high, int8_t& low);
  /**
   * @brief 获取四字节拼接数据
   * @param[in] high_h 最高八位
   * @param[in] high_l 次低八位
   * @param[in] low_h  次低八位
   * @param[in] low_l  最低八位
   */
  int32_t get_data(int8_t& high_h, int8_t& high_l, int8_t& low_h, int8_t& low_l);
  /**
   * @brief 电机驱动器高速信息反馈
   */
  void motor_driver_high(const struct can_frame& frame, json& result);
  /**
   * @brief 电机驱动器低速信息反馈
   */
  void motor_driver_low(const struct can_frame& frame, json& result);

  void get_item(const std::string& item, const int8_t& value,
      const std::string& text, json& result);

  void set_nav_detail_status(int status);

protected:
  uint8_t m_max_count;
  rclcpp::Node *m_node{nullptr};

private:
  uint8_t m_count = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub_nav_detail_status;
};

}

#endif