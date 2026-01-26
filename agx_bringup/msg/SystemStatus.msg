# SystemStatus.msg - 211帧系统状态数据
# 系统状态信息
uint8 status            # 系统状态码
string status_text      # 系统状态文本描述
uint8 ctrl_mode         # 控制模式码
string ctrl_mode_text   # 控制模式文本描述
float64 vol             # 系统电压 (单位: V)
FaultBit[] faults       # 故障信息数组