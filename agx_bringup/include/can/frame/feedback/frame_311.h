//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: frame_311.h
// Version: 1.0.00
//
// Create:  Feb.21 2025
// Author:  youguo.chu@agilex.ai
// Summary: 311帧解析
//-----------------------------------------------------------------------------

#ifndef FRAME_311_H_
#define FRAME_311_H_

#include <cmath>
#include <memory>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "can/chassis_status.h"
#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/msg/front_wheel_odometry.hpp"
/**
 * 311帧解析，里程计回馈指令帧
 */
namespace agx::nav
{

    class Frame311 : public BaseFrame
    {
    public:
        Frame311();
        ~Frame311() = default;
        virtual void parse(const struct can_frame &frame) override;

    private:
        rclcpp::Publisher<agx_bringup::msg::FrontWheelOdometry>::SharedPtr odom_pub_;
        double left_odom_;
        double right_odom_;
        bool m_publisher_initialized = false;
    };

} // namespace agx::nav

#endif