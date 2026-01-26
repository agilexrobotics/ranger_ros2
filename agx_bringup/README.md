# AGX 启动包

## 概述
本 ROS2 包提供了启动 AGX 机器人系统can控制接收的启动文件和配置。

## 功能
- CAN 总线通信与反馈处理

## 安装
1. 将本仓库克隆到 ROS 工作空间的 `src` 文件夹中：
   ```bash
   cd ~/colcon_ws/src
   git clone https://github.com/agilexrobotics/ranger_ros2.git -b air_delta
   ```
2. 构建包：
   ```bash
   cd ~/colcon_ws
   colcon build
   ```
3. 配置工作空间：
   ```bash
   source install/setup.bash
   ```

## 使用
启动机器人系统：
```bash
ros2 launch agx_bringup robot.launch.py
```

## ROS 话题说明

### 1. /bms_alarm_warning
- **描述**：BMS（电池管理系统）报警和警告消息。

### 2. /bms_status
- **描述**：BMS（电池管理系统）状态反馈消息。

### 3. /chassis_motion_feedback
- **描述**：底盘运动数据反馈消息。

### 4. /light_control_status
- **描述**：灯光控制状态反馈消息。

### 5. /motion_mode_feedback
- **描述**：运动模式反馈消息。

### 6. /motor_high_speed_feedback
- **描述**：电机高速反馈数据消息。

### 7. /motor_low_speed_feedback
- **描述**：电机低速反馈数据消息。

### 8. /remote_control_status
- **描述**：遥控指令状态反馈消息。

### 9. /steering_angles
- **描述**：四轮转向角度反馈消息。

### 10. /system_status
- **描述**：系统状态反馈消息。

### 11. /front_wheel_odometry
- **描述**：前轮式里程计数据反馈消息。

### 12. /back_wheel_odometry
- **描述**：后轮式里程计数据反馈消息。

### 13. /wheel_speeds
- **描述**：四轮速度反馈消息。

### 14. /sub_cmd_vel
- **描述**： 订阅速度，驱动底盘，此话题名称可在launch文件里面重新映射。

### 15. /wheel/odom
- **描述**： 底盘解算后的轮式里程计，此话题名称可在launch文件里面重新映射。