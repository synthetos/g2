
#ifndef CAN_H_ONCE
#define CAN_H_ONCE

#include "g2core.h"
#include "hardware.h"
#include "gpio.h"
#include "xio.h"

#ifdef CAN_ENABLED

typedef void(*ccb_t)(uint8_t*) ; // Can Callback Type

void can_send_message (uint32_t id, uint8_t length, uint8_t *data);

void can_digital_output(uint8_t, bool);

#endif
#endif
