
#ifndef CAN_H_ONCE
#define CAN_H_ONCE

#include "g2core.h"
#include "hardware.h"
#include "gpio.h"

#ifdef CAN_ENABLED

/*
typedef union {
  uint64_t value;

	struct {
		uint32_t low;
		uint32_t high;
	};

	struct {
		uint16_t s0;
		uint16_t s1;
		uint16_t s2;
		uint16_t s3;
  };

	uint8_t bytes[8];
} cnData_t;

typedef struct {
  uint32_t id;
  cnData_t data;
} cnMessage_t;
*/

typedef void(*ccb_t)(uint8_t*) ; // Can Callback Type
//
// template <uint32_t Tid, ccb_t Tcallback>
// struct CanRoute {
//     const uint32_t id = Tid;
//
//     void message_received (uint8_t *data) {
//       Tcallback(data);
//     }
// };
//
// /*
// class canDigitalInput : canRoute {
//     const uint8_t ext_pin_number;
//     d_in_t in;
//     bool inReset = false;
//
//     inline canDigitalInput (uint8_t id) : canRoute(id) {};
//
//     void reset();
//
//     void message_received (uint8_t *data);
// };*/
//
// extern CanRoute* canRoutes[];

extern void hw_can_init();
extern void hw_can_send_frame (uint32_t id, uint8_t length, uint8_t *data);

void can_message_received (uint32_t id, uint8_t length, uint8_t *data);
void can_send_message (uint32_t id, uint8_t length, uint8_t *data);

void can_digital_output(uint8_t, bool);

#endif
#endif
