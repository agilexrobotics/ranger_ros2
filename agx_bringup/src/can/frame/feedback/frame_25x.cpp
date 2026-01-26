#include "can/frame/feedback/frame_25x.h"
#include <rclcpp/rclcpp.hpp>

namespace agx::nav
{

  Frame25x::Frame25x()
  {
    m_max_count = 50;

    // 初始化1-8号电机数据存储
    for (int i = 1; i <= 8; ++i)
    {
      m_motor_data[i] = nlohmann::json::object();
    }

    m_received_count = 0;
    m_publisher_initialized = false;
  }

  void Frame25x::parse(const struct can_frame &frame)
  {
    try
    {
      // 检查是否是0x251-0x258帧
      if (frame.can_id < 0x251 || frame.can_id > 0x258)
      {
        return;
      }

      // 计算电机编号 (0x251->1, 0x252->2, ..., 0x258->8)
      int motor_id = frame.can_id - 0x250;

      std::lock_guard<std::mutex> lock(m_data_mutex);

      nlohmann::json motor_result;
      motor_driver_high(frame, motor_result);

      // 添加电机基本信息
      motor_result["motor_id"] = motor_id;
      motor_result["can_id"] = frame.can_id;

      // 检查是否是新数据
      bool had_previous_data = false;
      if (!m_motor_data[motor_id].empty() &&
          m_motor_data[motor_id].contains("motor_id"))
      {
        had_previous_data = true;
      }

      // 存储JSON数据
      m_motor_data[motor_id] = motor_result;

      // 只有当这个位置之前没有数据时才增加计数
      if (!had_previous_data)
      {
        m_received_count++;
      }

      if (m_node)
      {
        RCLCPP_DEBUG(m_node->get_logger(), "[BU] => 25x电机%d数据，已收集: %d/8",
                     motor_id, m_received_count.load());
      }
      // 检查是否收集完整
      if (m_received_count >= 8)
      {
        check_and_publish_complete_set();
      }
    }
    catch (const std::exception &e)
    {
      if (m_node)
      {
        RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析25x帧0x%x失败: %s",
                     frame.can_id, e.what());
      }
    }
    catch (...)
    {
      if (m_node)
      {
        RCLCPP_ERROR(m_node->get_logger(), "[BU] => 解析25x帧0x%x失败: 未知错误",
                     frame.can_id);
      }
    }
  }

  void Frame25x::check_and_publish_complete_set()
  {
    if (m_received_count >= 8)
    {
      try
      {
        // 延迟初始化发布者
        if (!m_publisher_initialized && m_node)
        {
          m_motor_array_pub = m_node->create_publisher<agx_bringup::msg::MotorHighArray>(
              "/motor_high_speed_feedback",
              rclcpp::QoS(rclcpp::KeepLast(m_max_count)).reliable());
          m_publisher_initialized = true;

          if (m_node)
          {
            RCLCPP_INFO(m_node->get_logger(), "[BU] => 初始化电机高速反馈发布者");
          }
        }

        if (!m_motor_array_pub)
        {
          return;
        }

        auto motor_array = agx_bringup::msg::MotorHighArray();

        for (int i = 1; i <= 8; ++i)
        {
          if (!m_motor_data[i].empty())
          {
            agx_bringup::msg::MotorHigh motor_high;
            json_to_motor_high(m_motor_data[i], motor_high);
            motor_array.motors.push_back(motor_high);
          }
        }

        m_motor_array_pub->publish(motor_array);

        if (m_node)
        {
          RCLCPP_DEBUG(m_node->get_logger(), "[BU] => 发布25x电机高速数据，数量: %zu",
                       motor_array.motors.size());
        }
      }
      catch (const std::exception &e)
      {
        if (m_node)
        {
          RCLCPP_ERROR(m_node->get_logger(), "[BU] => 发布25x数据失败: %s", e.what());
        }
      }

      reset_collection();
    }
  }

  void Frame25x::json_to_motor_high(const nlohmann::json &json_data, agx_bringup::msg::MotorHigh &motor_high)
  {
    // 从JSON转换为自定义消息
    motor_high.motor_id = json_data.value("motor_id", 0);
    motor_high.can_id = json_data.value("can_id", 0);
    motor_high.rpm = json_data.value("rpm", 0.0f);
    motor_high.current = json_data.value("current", 0.0f) * 0.1f;
    motor_high.pulse = json_data.value("pulse", 0);
  }

  void Frame25x::reset_collection()
  {
    m_received_count = 0;
    for (int i = 1; i <= 8; ++i)
    {
      m_motor_data[i] = nlohmann::json::object();
    }
  }

} // namespace agx::nav