
#include "can_bus.h"

#ifdef CAN_ENABLED

//#include "can_routing.h"

void can_send_message (uint32_t id, uint8_t length, uint8_t* data) {
  hw_can_send_frame(id, length, data);
}

void can_message_received (uint32_t id, uint8_t length, uint8_t* data) {
  switch (id) {
    case 0x201: can_gpio_received(1, length, data); break;

  }
}

void can_digital_output (uint8_t pin_num, bool value) {
  uint8_t data = (uint8_t)value;
  switch (pin_num) {
    case 1: can_send_message(0x201, 1, &data); break;
  }
}

#endif
