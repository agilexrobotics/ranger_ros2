#include "can/frame/handler/can_handler.h"
#include "can/frame/handler/handler_111.h"
#include "can/frame/handler/handler_121.h"
#include "can/frame/handler/handler_141.h"
#include "can/frame/handler/handler_423.h"
#include "can/frame/handler/handler_441.h"
#include "can/frame/handler/handler_461.h"
#include "can/chassis_status.h"  // 包含Instruction枚举定义

namespace agx::nav {

CanHandler::CanHandler()
{
  // 使用现代C++语法
  m_handler_map[agx::nav::Instruction::CTRL_MOTION] = std::make_unique<Handler111>();
  m_handler_map[agx::nav::Instruction::LIGHT_CTRL] = std::make_unique<Handler121>();
  m_handler_map[agx::nav::Instruction::MOTION_MODEL] = std::make_unique<Handler141>();
  m_handler_map[agx::nav::Instruction::CTRL_SWITCH] = std::make_unique<Handler423>();
  m_handler_map[agx::nav::Instruction::CLEAR_FAULT] = std::make_unique<Handler441>();
  m_handler_map[agx::nav::Instruction::HIBERNATE] = std::make_unique<Handler461>();
}

CanHandler::~CanHandler()
{
  // 清理资源
  m_handler_map.clear();
}

void CanHandler::handle(uint16_t instruction, const nlohmann::json& data)
{
  auto it = m_handler_map.find(instruction);  
  if (it != m_handler_map.end() && it->second)
  {
    it->second->handle(data);
  }
}

}