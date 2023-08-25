uint8 VEHICLE_STATE_NORMAL = 0
uint8 VEHICLE_STATE_ESTOP = 1
uint8 VEHICLE_STATE_EXCEPTION = 2

uint8 CONTROL_MODE_RC = 0
uint8 CONTROL_MODE_CAN = 1

uint8 MOTION_MODE_DUAL_ACKERMAN = 0
uint8 MOTION_MODE_PARALLEL = 1
uint8 MOTION_MODE_SPINNING = 2
uint8 MOTION_MODE_SIDE_SLIP = 3

########################################

std_msgs/Header header

# system state
uint8 vehicle_state
uint8 control_mode
uint16 error_code
float64 battery_voltage

# motion state
uint8 motion_mode
