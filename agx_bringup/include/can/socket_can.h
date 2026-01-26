#pragma once

#include "can/chassis_status.h"
#include "can/frame/feedback/base_frame.h"
#include "can/frame/feedback/frame_25x.h"
#include "can/frame/feedback/frame_26x.h"
#include "can/frame/feedback/frame_211.h"
#include "can/frame/feedback/frame_221.h"
#include "can/frame/feedback/frame_231.h"
#include "can/frame/feedback/frame_241.h"
#include "can/frame/feedback/frame_271.h"
#include "can/frame/feedback/frame_281.h"
#include "can/frame/feedback/frame_291.h"
#include "can/frame/feedback/frame_311.h"
#include "can/frame/feedback/frame_312.h"
#include "can/frame/feedback/frame_361.h"
#include "can/frame/feedback/frame_362.h"
#include "can/frame/feedback/frame_371.h"

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <mutex>
#include <memory>
#include <thread>
#include <unordered_map>
#include <iostream>
#include <sstream>
#include <iomanip>

namespace agx::nav {

class SocketCan
{
public:
    // 使用智能指针单例模式
    static std::shared_ptr<SocketCan> get_instance(rclcpp::Node* node = nullptr);
    
    // 删除拷贝构造函数和赋值操作符
    SocketCan(const SocketCan&) = delete;
    SocketCan& operator=(const SocketCan&) = delete;
    
    virtual ~SocketCan();
    
    void launch();
    bool send_frame(const struct can_frame& frame);
    
    // 添加连接状态检查
    bool is_initialized() const { return m_initialized; }
    
    // 设置节点指针
    void set_node(rclcpp::Node* node) { m_node = node; }
    
private:
    SocketCan(rclcpp::Node* node = nullptr); // 私有构造函数
    
    void launch_thread();
    bool init_can();
    bool init_frame();
    struct can_frame receive_frame();
    void close_socket();
    void process_frame(const struct can_frame& frame);
    
    // 单例相关
    static std::shared_ptr<SocketCan> m_instance;
    static std::mutex m_instance_mutex;
    
    // 成员变量
    int m_sock;
    std::string m_interface;
    bool m_debug;
    bool m_running;
    bool m_initialized;
    
    std::unique_ptr<std::thread> m_socket_thread;
    std::mutex m_send_mutex;
    
    std::unordered_map<uint32_t, std::unique_ptr<BaseFrame>> m_frame_map;
    std::unique_ptr<Frame25x> m_frame_25x;
    std::unique_ptr<Frame26x> m_frame_26x;
    
    rclcpp::Node* m_node;
};

} // namespace agx::nav