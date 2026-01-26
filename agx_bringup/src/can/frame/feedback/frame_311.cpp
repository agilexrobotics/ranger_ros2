#include "can/frame/feedback/frame_311.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav
{

    Frame311::Frame311()
    {
        // 初始化里程计值
        left_odom_ = 0.0;
        right_odom_ = 0.0;
        m_publisher_initialized = false;
    }

    void Frame311::parse(const struct can_frame &frame)
    {
        try
        {
            if (!m_publisher_initialized && m_node)
            {
                // 初始化发布器
                odom_pub_ = m_node->create_publisher<agx_bringup::msg::FrontWheelOdometry>(
                    "/front_wheel_odom",
                    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
            }
            // 处理里程计
            left_odom_ = (double)((int32_t)((frame.data[3] & 0xff) | (frame.data[2] << 8) | (frame.data[1] << 16) | (frame.data[0] << 24))) / 1000.0;
            right_odom_ = (double)((int32_t)((frame.data[7] & 0xff) | (frame.data[6] << 8) | (frame.data[5] << 16) | (frame.data[4] << 24))) / 1000.0;

            // 发布消息
            auto odom_msg = std::make_unique<agx_bringup::msg::FrontWheelOdometry>();
            odom_msg->left_wheel_odom = left_odom_;
            odom_msg->right_wheel_odom = right_odom_;

            odom_pub_->publish(std::move(odom_msg));

            // 使用ROS2的节流调试输出
            RCLCPP_DEBUG_THROTTLE(m_node->get_logger(), *m_node->get_clock(), 1000,
                                  "前轮里程计 - 左轮: %.3f m, 右轮: %.3f m",
                                  left_odom_, right_odom_);
        }
        catch (...)
        {
            RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析311帧失败");
        }
    }

} // namespace agx::nav