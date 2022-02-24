# ROS2 Packages for Hunter Mobile Base

## Packages

* ranger_base: a ROS wrapper around Hunter SDK to monitor and control the robot
* ranger_msgs: ranger related message definitions


## Communication interface setup

Please refer to the [README](https://github.com/agilexrobotics/ugv_sdk#hardware-interface) of "ugv_sdk" package for setup of communication interfaces.

#### Note on CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS package


1. Clone the packages into your ROS2 workspace and compile

    (the following instructions assume your ROS2 workspace is at: ~/ros2_ws/src)

    ```
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/agilexrobotics/ugv_sdk.git
    $ git clone https://github.com/agilexrobotics/ranger_ros2.git
    $ cd ..
    $ colcon build
    ```
    
2. Setup CAN-To-USB adapter

* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    $ sudo modprobe gs_usb
    ```
    
* first time use ranger-ros2 package
   ```
   $ cd ~/your_ws/src/ugv_sdk/scripts/
   $ bash setup_can2usb.bash
   ```
   
* if not the first time use ranger-ros2 package(Run this command every time you turn off the power) 
   ```
   $ cd ~/catkin_ws/src/ugv_sdk/scripts/
   $ bash bringup_can2usb_500k.bash
   ```
   
* Testing command
    ```
    # receiving data from can0
    $ candump can0
    ```

3. Launch ROS nodes

* Start the base node for the real robot

    ```
    $ ros2 launch ranger_base ranger_base.launch.py
    ```
* Then you can send command to the robot
    ```
    $ ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 0.0" 

    ```
* Then you can send command to the changed ranger motion_mode
    ```
    $ ros2 topic pub /ranger_setting ranger_msgs/msg/RangerSetting "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
  motion_mode: 0" 
    ```
* The motion_mode:
```
motion_mode ： 0    前后阿克曼模式 MOTION_MODE_ACKERMAN = 0
motion_mode ： 1    斜移模式      MOTION_MODE_SLIDE = 1
motion_mode ： 2    自旋模式      MOTION_MODE_ROUND = 2
motion_mode ： 3    横移模式      MOTION_MODE_SLOPING = 3

```

**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 
