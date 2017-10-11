#include "can_bus.h"

#ifndef CAN_ROUTING_ONCE
#define CAN_ROUTING_ONCE

extern void can_gpio_received (int pin_num, bool pin_value);

template <uint32_t

void can_message_received (uint32_t id, uint8_t length, uint8_t *data) {

}

#endif
