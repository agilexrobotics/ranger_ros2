#include "can/chassis_status.h"

namespace agx::nav {

/** 车体状态 */
uint8_t Status::NORMAL = 0;  // 系统正常
uint8_t Status::E_STOP = 1;  // 紧急停车
uint8_t Status::ERROR = 2;   // 系统异常

/** 模式控制 */
uint8_t CtrlMode::STANDBY = 0;     // 待机模式
uint8_t CtrlMode::CAN = 1;         // CAN模式
uint8_t CtrlMode::SERIAL = 2;      // 串口控制模式
uint8_t CtrlMode::REMOTE_CTRL = 3; // 遥控模式

/** can指令 */
uint16_t Instruction::SYS_STATUS = 0x211;
uint16_t Instruction::MOTION = 0x221;
uint16_t Instruction::ODOM_FRONT = 0x311;
uint16_t Instruction::ODOM_BACK = 0x312;
uint16_t Instruction::BMS_ALL = 0x361;
uint16_t Instruction::BMS_ALL_STATUS = 0x362;
uint16_t Instruction::CHARGER = 0x371;
uint16_t Instruction::CTRL_MOTION = 0x111;
uint16_t Instruction::ENABLE = 0x421;
uint16_t Instruction::HIBERNATE = 0x461;              // 休眠帧
uint16_t Instruction::CTRL_SWITCH = 0x423;            // 电流、电压控制切换指令
uint16_t Instruction::CLEAR_FAULT = 0x441;            // 清除故障指令
uint16_t Instruction::MOTION_MODEL = 0x141;           // 运动模型
uint16_t Instruction::LIGHT_CTRL = 0x121;             // 灯光控制指令帧
uint16_t Instruction::REMOTE_CONTROL_STATUS = 0x241;  // 遥控器状态反馈
uint16_t Instruction::LIGHT_STATUS = 0x231;           // 灯光状态反馈
uint16_t Instruction::MOTION_MODE = 0x291;            // 运动模式反馈
uint16_t Instruction::TURN_SPEED = 0x281;             // 四轮转速
uint16_t Instruction::TURN_ANGLE = 0x271;             // 四轮转角
uint16_t Instruction::MOTOR_DRIVE_HIGH_1 = 0x251;     // 右前轮1号
uint16_t Instruction::MOTOR_DRIVE_HIGH_2 = 0x252;     // 右后轮2号
uint16_t Instruction::MOTOR_DRIVE_HIGH_3 = 0x253;     // 左后轮3号
uint16_t Instruction::MOTOR_DRIVE_HIGH_4 = 0x254;     // 左前轮4号
uint16_t Instruction::MOTOR_DRIVE_HIGH_5 = 0x255;     // 右前转向5号
uint16_t Instruction::MOTOR_DRIVE_HIGH_6 = 0x256;     // 右后转向6号
uint16_t Instruction::MOTOR_DRIVE_HIGH_7 = 0x257;     // 左后转向7号
uint16_t Instruction::MOTOR_DRIVE_HIGH_8 = 0x258;     // 左前转向8号
uint16_t Instruction::MOTOR_DRIVE_LOW_1 = 0x261;      // 右前轮1号
uint16_t Instruction::MOTOR_DRIVE_LOW_2 = 0x262;      // 右后轮2号
uint16_t Instruction::MOTOR_DRIVE_LOW_3 = 0x263;      // 左后轮3号
uint16_t Instruction::MOTOR_DRIVE_LOW_4 = 0x264;      // 左前轮4号
uint16_t Instruction::MOTOR_DRIVE_LOW_5 = 0x265;      // 右前转向5号
uint16_t Instruction::MOTOR_DRIVE_LOW_6 = 0x266;      // 右后转向6号
uint16_t Instruction::MOTOR_DRIVE_LOW_7 = 0x267;      // 左后转向7号
uint16_t Instruction::MOTOR_DRIVE_LOW_8 = 0x268;      // 左前转向8号

}
