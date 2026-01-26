# MotorLow.msg - 26x低速反馈数据
# 电机低速信息反馈
uint8 motor_id          # 电机ID (1-8)
uint16 can_id           # CAN ID
float32 driver_vol      # 驱动器电压 (单位: V)
float32 driver_temp     # 驱动器温度 (单位: °C)
float32 motor_temp      # 电机温度 (单位: °C)
uint8 driver_status     # 驱动器状态字节
bool enable_status      # 使能状态
bool error_status       # 错误状态
bool sensor_status      # 传感器状态
bool over_temp_driver   # 驱动器过温
bool over_current       # 过流
bool over_temp_motor    # 电机过温
bool under_voltage      # 欠压