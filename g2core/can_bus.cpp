
#include "can_bus.h"
// #define CAN_GPIO_INPUT_ADDRESS_1 0x42

#ifdef CAN_ENABLED

void can_send_message (uint32_t id, uint8_t length, uint8_t* data) {
  hw_can_send_frame(id, length, data);
}

void can_message_received (uint32_t id, uint8_t length, uint8_t* data) {
  switch (id) {
    //Generated with: perl -e 'for($i=1;$i<13 { print "#ifdef CAN_GPIO_INPUT_ADDRESS_${i}\ncase CAN_GPIO_INPUT_ADDRESS_${i}: can_gpio_received(${i}, length, data); break;\n#endif\n";}'
    #ifdef CAN_GPIO_INPUT_ADDRESS_1
    case CAN_GPIO_INPUT_ADDRESS_1: can_gpio_received(0, 1, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_2
    case CAN_GPIO_INPUT_ADDRESS_2: can_gpio_received(2, length, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_3
    case CAN_GPIO_INPUT_ADDRESS_3: can_gpio_received(3, length, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_4
    case CAN_GPIO_INPUT_ADDRESS_4: can_gpio_received(4, length, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_5
    case CAN_GPIO_INPUT_ADDRESS_5: can_gpio_received(5, length, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_6
    case CAN_GPIO_INPUT_ADDRESS_6: can_gpio_received(6, length, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_7
    case CAN_GPIO_INPUT_ADDRESS_7: can_gpio_received(7, length, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_8
    case CAN_GPIO_INPUT_ADDRESS_8: can_gpio_received(8, length, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_9
    case CAN_GPIO_INPUT_ADDRESS_9: can_gpio_received(9, length, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_10
    case CAN_GPIO_INPUT_ADDRESS_10: can_gpio_received(10, length, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_11
    case CAN_GPIO_INPUT_ADDRESS_11: can_gpio_received(11, length, data); break;
    #endif
    #ifdef CAN_GPIO_INPUT_ADDRESS_12
    case CAN_GPIO_INPUT_ADDRESS_12: can_gpio_received(12, length, data); break;
    #endif

  }

  //can_gpio_received(100, 1, data);
}

void can_digital_output (uint8_t pin_num, bool value) {
  uint8_t data = (uint8_t)value;
  switch (pin_num) {
    case 1: can_send_message(0x201, 1, &data); break;
  }
}

#endif
