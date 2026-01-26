//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: chassis_status.h
// Version: 1.0.00
//
// Create:  Feb.21 2025
// Author:  youguo.chu@agilex.ai
// Summary: 底盘状态常量定义
//-----------------------------------------------------------------------------

#ifndef CHASSIS_STATUS_H_
#define CHASSIS_STATUS_H_

#include <cstdint>

namespace agx::nav {

/** 车体状态 */
class Status
{
public:
  static uint8_t NORMAL;
  static uint8_t E_STOP;
  static uint8_t ERROR;
};

/** 模式控制 */
class CtrlMode
{
public:
  static uint8_t STANDBY;
  static uint8_t CAN;
  static uint8_t SERIAL;
  static uint8_t REMOTE_CTRL;
};

/** can指令 */
class Instruction
{
public:
  static uint16_t SYS_STATUS;
  static uint16_t MOTION;
  static uint16_t ODOM_FRONT;
  static uint16_t ODOM_BACK;
  static uint16_t BMS_ALL;
  static uint16_t BMS_ALL_STATUS;
  static uint16_t CHARGER;
  static uint16_t CTRL_MOTION;
  static uint16_t ENABLE;
  static uint16_t HIBERNATE;
  static uint16_t CTRL_SWITCH;           // 电流、电压控制切换指令
  static uint16_t CLEAR_FAULT;           // 清除故障
  static uint16_t MOTION_MODEL;          // 运动模型切换指令
  static uint16_t LIGHT_CTRL;            // 灯光控制指令帧
  static uint16_t REMOTE_CONTROL_STATUS;
  static uint16_t LIGHT_STATUS;
  static uint16_t MOTION_MODE;
  static uint16_t TURN_SPEED;
  static uint16_t TURN_ANGLE;
  static uint16_t MOTOR_DRIVE_HIGH_1;    // 右前轮1号
  static uint16_t MOTOR_DRIVE_HIGH_2;    // 右后轮2号
  static uint16_t MOTOR_DRIVE_HIGH_3;    // 左后轮3号
  static uint16_t MOTOR_DRIVE_HIGH_4;    // 左前轮4号
  static uint16_t MOTOR_DRIVE_HIGH_5;    // 右前转向5号
  static uint16_t MOTOR_DRIVE_HIGH_6;    // 右后转向6号
  static uint16_t MOTOR_DRIVE_HIGH_7;    // 左后转向7号
  static uint16_t MOTOR_DRIVE_HIGH_8;    // 左前转向8号
  static uint16_t MOTOR_DRIVE_LOW_1;     // 右前轮1号
  static uint16_t MOTOR_DRIVE_LOW_2;     // 右后轮2号
  static uint16_t MOTOR_DRIVE_LOW_3;     // 左后轮3号
  static uint16_t MOTOR_DRIVE_LOW_4;     // 左前轮4号
  static uint16_t MOTOR_DRIVE_LOW_5;     // 右前转向5号
  static uint16_t MOTOR_DRIVE_LOW_6;     // 右后转向6号
  static uint16_t MOTOR_DRIVE_LOW_7;     // 左后转向7号
  static uint16_t MOTOR_DRIVE_LOW_8;     // 左前转向8号
};

}

#endif
