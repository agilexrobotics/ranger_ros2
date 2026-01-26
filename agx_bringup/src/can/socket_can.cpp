#include "can/socket_can.h"
#include <cstring>
#include <stdexcept>
#include <chrono>

namespace agx::nav
{
    // 静态成员初始化
    std::shared_ptr<SocketCan> SocketCan::m_instance = nullptr;
    std::mutex SocketCan::m_instance_mutex;

    std::shared_ptr<SocketCan> SocketCan::get_instance(rclcpp::Node* node)
    {
        std::lock_guard<std::mutex> lock(m_instance_mutex);
        if (!m_instance) {
            m_instance = std::shared_ptr<SocketCan>(new SocketCan(node));
            if (!m_instance->m_initialized) {
                m_instance = nullptr;
            }
        } else if (node && !m_instance->m_node) {
            // 如果实例已存在但未设置节点，则设置节点
            m_instance->set_node(node);
        }
        return m_instance;
    }

    SocketCan::SocketCan(rclcpp::Node* node) 
        : m_node(node),
          m_sock(-1),
          m_interface("can0"),
          m_debug(false),
          m_running(false),
          m_initialized(false)
    {

        try
        {
            if (init_can() && init_frame()) {
                m_initialized = true;
                if (m_node) {
                    RCLCPP_INFO(m_node->get_logger(), "SocketCan初始化成功");
                }
            } else {
                if (m_node) {
                    RCLCPP_ERROR(m_node->get_logger(), "SocketCan初始化失败");
                }
            }
        }
        catch (const std::exception &e)
        {
            if (m_node) {
                RCLCPP_ERROR(m_node->get_logger(), "SocketCan初始化异常: %s", e.what());
            }
        }
    }

    SocketCan::~SocketCan()
    {
        m_running = false;
        if (m_socket_thread && m_socket_thread->joinable())
        {
            m_socket_thread->join();
            m_socket_thread.reset();
        }

        close_socket();
        if (m_node) {
            RCLCPP_INFO(m_node->get_logger(), "SocketCan销毁完成");
        }
    }

    void SocketCan::launch()
    {
        if (!m_initialized) {
            if (m_node) {
                RCLCPP_ERROR(m_node->get_logger(), "SocketCan未初始化，无法启动");
            }
            return;
        }
        
        if (!m_running)
        {
            m_running = true;
            m_socket_thread = std::make_unique<std::thread>(&SocketCan::launch_thread, this);
            if (m_node) {
                RCLCPP_INFO(m_node->get_logger(), "启动CAN通信模块");
            }
        }
        else
        {
            if (m_node) {
                RCLCPP_WARN(m_node->get_logger(), "CAN通信模块已在运行中");
            }
        }
    }

    void SocketCan::launch_thread()
    {
        if (m_node) {
            RCLCPP_INFO(m_node->get_logger(), "启动CAN采集线程");
        }

        while (rclcpp::ok() && m_running)
        {
            try
            {
                struct can_frame frame = receive_frame();
                
                if (frame.can_dlc > 0 && frame.can_dlc <= 8)
                {
                    if (m_debug && m_node)
                    {
                        std::stringstream ss;
                        ss << "Received CAN frame: ID=0x" << std::hex << frame.can_id
                           << ", DLC=" << std::dec << static_cast<int>(frame.can_dlc)
                           << ", Data=";
                        for (int i = 0; i < frame.can_dlc; i++)
                        {
                            ss << std::hex << std::setw(2) << std::setfill('0')
                               << static_cast<int>(frame.data[i]) << " ";
                        }
                        RCLCPP_DEBUG(m_node->get_logger(), "%s", ss.str().c_str());
                    }

                    process_frame(frame);
                }

            }
            catch (const std::exception &e)
            {
                if (m_node) {
                    RCLCPP_ERROR(m_node->get_logger(), "处理CAN帧时发生异常: %s", e.what());
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        if (m_node) {
            RCLCPP_INFO(m_node->get_logger(), "CAN采集线程退出");
        }
        m_running = false;
    }

    void SocketCan::process_frame(const struct can_frame& frame)
    {
        // 251-258帧由m_frame_25x处理
        if (frame.can_id >= agx::nav::Instruction::MOTOR_DRIVE_HIGH_1 &&
            frame.can_id <= agx::nav::Instruction::MOTOR_DRIVE_HIGH_8)
        {
            if (m_frame_25x)
            {
                m_frame_25x->parse(frame);
            }
            else if (m_debug && m_node)
            {
                RCLCPP_DEBUG(m_node->get_logger(), "未设置25x帧处理器，忽略ID=0x%X", frame.can_id);
            }
        }
        // 261-268帧由m_frame_26x处理
        else if (frame.can_id >= agx::nav::Instruction::MOTOR_DRIVE_LOW_1 &&
                 frame.can_id <= agx::nav::Instruction::MOTOR_DRIVE_LOW_8)
        {
            if (m_frame_26x)
            {
                m_frame_26x->parse(frame);
            }
            else if (m_debug && m_node)
            {
                RCLCPP_DEBUG(m_node->get_logger(), "未设置26x帧处理器，忽略ID=0x%X", frame.can_id);
            }
        }
        // 其他帧使用映射表处理
        else
        {
            auto it = m_frame_map.find(frame.can_id);
            if (it != m_frame_map.end())
            {
                it->second->parse(frame);
            }
            else if (m_debug && m_node)
            {
                RCLCPP_DEBUG(m_node->get_logger(), "未设置帧处理器，忽略ID=0x%X", frame.can_id);
            }
        }
    }

    bool SocketCan::send_frame(const struct can_frame &frame)
    {
        std::lock_guard<std::mutex> lock(m_send_mutex);

        if (m_sock < 0)
        {
            if (m_node) {
                RCLCPP_ERROR(m_node->get_logger(), "CAN套接字未初始化，无法发送帧");
            }
            return false;
        }

        if (frame.can_dlc > 8) {
            if (m_node) {
                RCLCPP_ERROR(m_node->get_logger(), "无效的CAN帧DLC: %d", frame.can_dlc);
            }
            return false;
        }

        ssize_t result = write(m_sock, &frame, sizeof(frame));
        if (result != sizeof(frame))
        {
            if (m_node) {
                RCLCPP_ERROR(m_node->get_logger(), "发送CAN帧失败，错误: %s", strerror(errno));
            }
            return false;
        }
        
        if (m_debug && m_node)
        {
            RCLCPP_DEBUG(m_node->get_logger(), "发送CAN帧: ID=0x%X, DLC=%d", frame.can_id, frame.can_dlc);
        }
        
        return true;
    }

    struct can_frame SocketCan::receive_frame()
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        if (m_sock < 0)
        {
            return frame;
        }

        // 使用select实现带超时的读取
        fd_set readfds;
        struct timeval timeout;

        FD_ZERO(&readfds);
        FD_SET(m_sock, &readfds);

        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100ms超时

        int ret = select(m_sock + 1, &readfds, nullptr, nullptr, &timeout);
        if (ret < 0)
        {
            if (errno != EINTR && m_node)
            {
                RCLCPP_ERROR(m_node->get_logger(), "select错误: %s", strerror(errno));
            }
            return frame;
        }
        else if (ret == 0)
        {
            // 超时，返回空帧
            return frame;
        }

        // 读取数据
        ssize_t nbytes = read(m_sock, &frame, sizeof(frame));
        if (nbytes < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK && m_node)
            {
                RCLCPP_ERROR(m_node->get_logger(), "读取CAN帧错误: %s", strerror(errno));
            }
            memset(&frame, 0, sizeof(frame));
        }
        else if (nbytes != sizeof(frame) && m_node)
        {
            RCLCPP_WARN(m_node->get_logger(), "读取的CAN帧长度不正确: %zd", nbytes);
            memset(&frame, 0, sizeof(frame));
        }

        return frame;
    }

    bool SocketCan::init_can()
    {
        // 创建套接字
        if ((m_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            if (m_node) {
                RCLCPP_ERROR(m_node->get_logger(), "创建CAN套接字失败: %s", strerror(errno));
            }
            return false;
        }

        // 设置非阻塞模式
        int flags = fcntl(m_sock, F_GETFL, 0);
        if (flags >= 0)
        {
            fcntl(m_sock, F_SETFL, flags | O_NONBLOCK);
        }

        // 指定CAN接口
        struct ifreq ifr;
        if (m_interface.length() >= IFNAMSIZ) {
            if (m_node) {
                RCLCPP_ERROR(m_node->get_logger(), "接口名称过长: %s", m_interface.c_str());
            }
            close(m_sock);
            m_sock = -1;
            return false;
        }
        
        strncpy(ifr.ifr_name, m_interface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(m_sock, SIOCGIFINDEX, &ifr) < 0)
        {
            if (m_node) {
                RCLCPP_ERROR(m_node->get_logger(), "获取接口索引失败: %s, 错误: %s",
                           m_interface.c_str(), strerror(errno));
            }
            close(m_sock);
            m_sock = -1;
            return false;
        }

        // 绑定套接字到CAN接口
        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(m_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            if (m_node) {
                RCLCPP_ERROR(m_node->get_logger(), "绑定套接字失败: %s", strerror(errno));
            }
            close(m_sock);
            m_sock = -1;
            return false;
        }

        if (m_node) {
            RCLCPP_INFO(m_node->get_logger(), "CAN接口 %s 初始化成功", m_interface.c_str());
        }
        return true;
    }

    bool SocketCan::init_frame()
    {
        try {
            // 创建聚合处理器并设置节点
            m_frame_25x = std::make_unique<Frame25x>();
            m_frame_25x->set_node(m_node);

            m_frame_26x = std::make_unique<Frame26x>();
            m_frame_26x->set_node(m_node);

            // 初始化其他帧处理器
            auto create_frame = [this](auto frame, uint32_t instruction) {
                if (frame) {
                    frame->set_node(m_node);
                    m_frame_map[instruction] = std::move(frame);
                    return true;
                }
                return false;
            };

            create_frame(std::make_unique<Frame211>(), agx::nav::Instruction::SYS_STATUS);
            create_frame(std::make_unique<Frame221>(), agx::nav::Instruction::MOTION);
            create_frame(std::make_unique<Frame231>(), agx::nav::Instruction::LIGHT_STATUS);
            create_frame(std::make_unique<Frame241>(), agx::nav::Instruction::REMOTE_CONTROL_STATUS);
            create_frame(std::make_unique<Frame271>(), agx::nav::Instruction::TURN_ANGLE);
            create_frame(std::make_unique<Frame281>(), agx::nav::Instruction::TURN_SPEED);
            create_frame(std::make_unique<Frame291>(), agx::nav::Instruction::MOTION_MODE);
            create_frame(std::make_unique<Frame311>(), agx::nav::Instruction::ODOM_FRONT);
            create_frame(std::make_unique<Frame312>(), agx::nav::Instruction::ODOM_BACK);
            create_frame(std::make_unique<Frame361>(), agx::nav::Instruction::BMS_ALL);
            create_frame(std::make_unique<Frame362>(), agx::nav::Instruction::BMS_ALL_STATUS);
            create_frame(std::make_unique<Frame371>(), agx::nav::Instruction::CHARGER);

            if (m_node) {
                RCLCPP_INFO(m_node->get_logger(), "CAN帧处理器初始化完成，共设置 %zu 个处理器",
                           m_frame_map.size() + 2);
            }
            return true;
        } catch (const std::exception& e) {
            if (m_node) {
                RCLCPP_ERROR(m_node->get_logger(), "初始化帧处理器失败: %s", e.what());
            }
            return false;
        }
    }

    void SocketCan::close_socket()
    {
        if (m_sock >= 0)
        {
            close(m_sock);
            m_sock = -1;
            if (m_node) {
                RCLCPP_INFO(m_node->get_logger(), "关闭CAN套接字");
            }
        }
    }

} // namespace agx::nav