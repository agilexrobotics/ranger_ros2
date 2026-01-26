# MotorHigh.msg - 25x高速反馈数据
# 电机高速信息反馈
uint8 motor_id          # 电机ID (1-8)
uint16 can_id           # CAN ID
float32 rpm             # 转速 (单位: RPM)
float32 current         # 电流 (单位: A)
int32 pulse             # 脉冲计数