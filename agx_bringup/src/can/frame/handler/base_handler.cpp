#include "can/frame/handler/base_handler.h"

namespace agx::nav {

BaseHandler::BaseHandler()
{}

BaseHandler::~BaseHandler()
{}

void BaseHandler::get_can_frame(struct can_frame& frame, uint16_t instruction_id,
      int8_t bit0, int8_t bit1, int8_t bit2, int8_t bit3, int8_t bit4,
      int8_t bit5, int8_t bit6, int8_t bit7)
{
  frame.can_id = instruction_id;
  frame.can_dlc = 8;
  frame.data[0] = bit0;
  frame.data[1] = bit1;
  frame.data[2] = bit2;
  frame.data[3] = bit3;
  frame.data[4] = bit4;
  frame.data[5] = bit5;
  frame.data[6] = bit6;
  frame.data[7] = bit7;
}

}