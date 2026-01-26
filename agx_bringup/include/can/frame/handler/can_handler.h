#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "can/frame/handler/base_handler.h"
#include <map>
#include <memory>
#include <nlohmann/json.hpp>

namespace agx::nav {

class CanHandler
{
public:
  CanHandler();
  ~CanHandler();
  
  void handle(uint16_t instruction, const nlohmann::json& data);

private:
  std::map<uint16_t, std::unique_ptr<BaseHandler>> m_handler_map;
};

} // namespace agx::nav

#endif