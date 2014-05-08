//
//  SD.cpp
//  TinyG2
//
//  Created by Rob on 5/2/14.
//  Copyright (c) 2014 Rob Giseburt. All rights reserved.
//

#include "tinyg2.h"			// #1
#include "config.h"			// #2
//#include "text_parser.h"
//#include "canonical_machine.h"
//#include "plan_arc.h"
//#include "planner.h"
//#include "stepper.h"
//#include "encoder.h"
//#include "spindle.h"
//#include "report.h"
//#include "gpio.h"
//#include "switch.h"
//#include "hardware.h"
//#include "util.h"

#include "sd.h"
#include "MotatePins.h"
#include "MotateSPI.h"
#include "MotateTimers.h"

using Motate::SysTickTimer;


#ifdef BOARD_HAS_SD

/** GO_IDLE_STATE - init card in spi mode if CS low */
const uint8_t kSD_CMD_GO_IDLE_STATE = 0x00;
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
const uint8_t kSD_CMD_SEND_IF_COND = 0X08;


/** status for card in the ready state */
uint8_t const kSD_R1_READY_STATE = 0X00;
/** status for card in the idle state */
uint8_t const kSD_R1_IDLE_STATE = 0X01;
/** status bit for illegal command */
uint8_t const kSD_R1_ILLEGAL_COMMAND = 0X04;
/** start data token for read or write single block*/
uint8_t const kSD_DATA_START_BLOCK = 0XFE;
/** stop token for write multiple blocks*/
uint8_t const kSD_STOP_TRAN_TOKEN = 0XFD;
/** start data token for write multiple blocks*/
uint8_t const kSD_WRITE_MULTIPLE_TOKEN = 0XFC;
/** mask for data response tokens after a write block operation */
uint8_t const kSD_DATA_RES_MASK = 0X1F;
/** write data accepted token */
uint8_t const kSD_DATA_RES_ACCEPTED = 0X05;

Motate::SPI<Motate::kSD_ChipSelectPinNumber> sd_spi(2000000);

uint8_t sdType[30] = "TESTING";


void sd_write(uint8_t data) {
    while (sd_spi.write(data) == -1) {};
}

uint8_t sd_read() {
    int16_t ret = -1;
    do {
        ret = sd_spi.read();
    } while (ret == -1);

    return ret & 0xff;
}

bool sd_wait_until_available(uint32_t timeout = 300) {
    uint32_t timeout_time = SysTickTimer.getValue() + timeout;
    do {
        uint8_t v = sd_read();
        if (v == 0xff) {
            return true;
        }
    } while (SysTickTimer.getValue() < timeout_time);
    return false;
}

uint8_t sd_send_command(const uint8_t command, uint32_t args) {
    sd_spi.setChannel();
    sd_spi.setSelected(true);
    sd_wait_until_available();

    sd_write(command | 0x40);
    for (int8_t s = 24; s >= 0; s -= 8) {
        sd_write(args >> s);
    }

    switch (command) {
        case kSD_CMD_GO_IDLE_STATE:
            sd_write(0x95);
            break;

        case kSD_CMD_SEND_IF_COND:
            sd_write(0x87);
            break;

        default:
            sd_write(0xFF);
            break;
    }

    // read the response
    uint8_t retries_left = 0xff;
    do {
        uint8_t value = sd_read();
        if (value & 0x80) {
            return value;
        }
    } while (retries_left--);

    return 0xff;
}

void sd_init() {

//    sd_spi.flush();
    sd_spi.setAutoselect(false);
    sd_spi.setSelected(false);

    // Must supply min of 74 clock cycles with CS high.
    // NOT selected! NOT selected. Not, selected.
    for (uint8_t i = 0; i < 10; i++) sd_write(0Xff);

//    sd_spi.setSelected(true);

    uint8_t _status = 0x00;

    uint32_t timeout_time = SysTickTimer.getValue() + 1000;
    do {
        _status = sd_send_command(kSD_CMD_GO_IDLE_STATE, 0);
        if (SysTickTimer.getValue() > timeout_time) {
            strcpy(sdType, "INIT TIMEOUT");
            goto deselectAndReturn;
        }
    } while (_status != kSD_R1_READY_STATE);

    _status = sd_send_command(kSD_CMD_SEND_IF_COND, 0x1AA);
    if (_status & kSD_R1_ILLEGAL_COMMAND) {
        strcpy(sdType, "SD Type SD1");
        goto deselectAndReturn;
    }
    else {
        // read four bytes, keep the last
        for (uint8_t i = 0; i < 4; i++) {
            _status = sd_read();
        }
        if (_status == 0xaa) {
            strcpy(sdType, "SD Type SD2");
            goto deselectAndReturn;
        }

        strcpy(sdType, "SD Error");
        goto deselectAndReturn;
    }

   deselectAndReturn:
    sd_spi.setSelected(false);
}

stat_t get_sd_type(cmdObj_t *cmd)
{
    sd_init();

    ritorno(cmd_copy_string(cmd, sdType));
	cmd->objtype = TYPE_STRING;
	return (STAT_OK);
}

#endif //HAS_SD