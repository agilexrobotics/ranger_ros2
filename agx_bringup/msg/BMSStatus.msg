# BMSStatus.msg - BMS状态反馈
# BMS状态信息
uint8 soc               # 电池剩余电量 (单位: %)
uint8 soh               # 电池健康度 (单位: %)
float32 voltage         # 电池电压 (单位: V)
float32 current         # 电池电流 (单位: A)
float32 temperature     # 电池温度 (单位: °C)
uint8 status            # BMS状态码
string status_text      # BMS状态文本描述