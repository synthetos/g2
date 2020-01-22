/*
 * hardware.cpp - general hardware support functions
 * For: /board/gQuintic
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "g2core.h"  // #1
#include "config.h"  // #2
#include "hardware.h"
#include "controller.h"
#include "text_parser.h"
#include "board_xio.h"
#include "board_gpio.h"
#include "safety_manager.h"

#include "MotateUtilities.h"
#include "MotateUniqueID.h"
#include "MotatePower.h"

//Motate::ClockOutputPin<Motate::kExternalClock1_PinNumber> external_clk_pin {16000000}; // 16MHz optimally
Motate::OutputPin<Motate::kExternalClock1_PinNumber> external_clk_pin {Motate::kStartLow};

HOT_DATA SPI_CS_PinMux_used_t spiCSPinMux;
HOT_DATA SPIBus_used_t spiBus;

// HOT_DATA TWIBus_used_t twiBus;

// // Define Multiplexers
// HOT_DATA plex0_t plex0{twiBus, 0x0070L};
// HOT_DATA plex1_t plex1{twiBus, 0x0071L};


// HOT_DATA I2C_EEPROM eeprom{twiBus, 0b01010000};
// alignas(4) uint8_t eeprom_buffer[128] HOT_DATA = "TestinglyABCDEFGHIJKLmnop";
// alignas(4) uint8_t eeprom_in_buffer[128] HOT_DATA = "";


SafetyManager sm{};
SafetyManager *safety_manager = &sm;


#ifndef SPINDLE_ENABLE_OUTPUT_NUMBER
#warning SPINDLE_ENABLE_OUTPUT_NUMBER is defaulted to 4!
#warning SPINDLE_ENABLE_OUTPUT_NUMBER should be defined in settings or a board file!
#define SPINDLE_ENABLE_OUTPUT_NUMBER 4
#endif

#ifndef SPINDLE_DIRECTION_OUTPUT_NUMBER
#warning SPINDLE_DIRECTION_OUTPUT_NUMBER is defaulted to 5!
#warning SPINDLE_DIRECTION_OUTPUT_NUMBER should be defined in settings or a board file!
#define SPINDLE_DIRECTION_OUTPUT_NUMBER 5
#endif

#ifndef SPINDLE_PWM_NUMBER
#warning SPINDLE_PWM_NUMBER is defaulted to 6!
#warning SPINDLE_PWM_NUMBER should be defined in settings or a board file!
#define SPINDLE_PWM_NUMBER 6
#endif

#ifndef SPINDLE_SPEED_CHANGE_PER_MS
#warning SPINDLE_SPEED_CHANGE_PER_MS is defaulted to 5!
#warning SPINDLE_SPEED_CHANGE_PER_MS should be defined in settings or a board file!
#define SPINDLE_SPEED_CHANGE_PER_MS 5
#endif

#include "esc_spindle.h"
ESCSpindle esc_spindle {SPINDLE_PWM_NUMBER, SPINDLE_ENABLE_OUTPUT_NUMBER, SPINDLE_DIRECTION_OUTPUT_NUMBER, SPINDLE_SPEED_CHANGE_PER_MS};

#if HAS_LASER
#ifndef LASER_ENABLE_OUTPUT_NUMBER
#error LASER_ENABLE_OUTPUT_NUMBER should be defined in settings or a board file!
#endif

#ifndef LASER_FIRE_PIN_NUMBER
#error LASER_FIRE_PIN_NUMBER should be defined in settings or a board file!
#endif

#include "laser_toolhead.h"
HOT_DATA LaserTool_used_t laser_tool {LASER_ENABLE_OUTPUT_NUMBER, MOTOR_6};

// CartesianKinematics<AXES, MOTORS> cartesian_kinematics;
KinematicsBase<AXES, MOTORS> *kn = &laser_tool;
#endif

ToolHead *toolhead_for_tool(uint8_t tool) {
#if !HAS_LASER
    return &esc_spindle;
#else
    if (tool != LASER_TOOL) {
        return &esc_spindle;
    } else {
        return &laser_tool;
    }
#endif
}

/*
 * hardware_init() - lowest level hardware init
 */

void hardware_init()
{
    spiBus.init();
    // twiBus.init();
    board_hardware_init();
    external_clk_pin = 0; // Force external clock to 0 for now.

    esc_spindle.init();
#if HAS_LASER
    laser_tool.init();
#endif
    spindle_set_toolhead(toolhead_for_tool(1));
}

/*
 * hardware_periodic() - callback from the controller loop - TIME CRITICAL.
 */

// previous values of analog voltages
float ai_vv[A_IN_CHANNELS];
const float analog_change_threshold = 0.01;

float angle_0 = 0.0;
float angle_1 = 0.0;

// void read_encoder_0(bool worked /* = false*/, float angle /* = 0.0*/) {
//     if (worked) {
//         angle_0 = angle;
//     }
//     encoder_0.getAngleFraction();
// }

// void read_encoder_1(bool worked /* = false*/, float angle /* = 0.0*/) {
//     if (worked) {
//         angle_1 = angle;
//     }
//     encoder_1.getAngleFraction();
// }

// void first_part(bool worked = false);
// void second_part(bool worked = false);
// void third_part(bool worked = false);

// void first_part (bool worked /* = false*/ ) {
//     if (!worked) {
//         eeprom.store(0, eeprom_buffer, 4, first_part);
//     } else {
//         second_part();
//     }
// }

// void second_part (bool worked /* = false*/ ) {
//     if (!worked) {
//         eeprom.store(4, eeprom_buffer + 4, 5, second_part);
//     } else {
//         third_part();
//     }
// }

// void third_part(bool worked /* = false*/ ) {
//     if (!worked) {
//         eeprom.load(0, eeprom_in_buffer, 9, third_part);
//     } else {
//         // read_encoder_0();
//     }
// }


stat_t hardware_periodic()
{
    // for all of the analog inputs that are enabled, request status reports
    // when they change beyond the threshold
    #if TEMPERATURE_OUTPUT_ON
    for (uint8_t a = 0; a < A_IN_CHANNELS; a++) {
        if (a_in[a]->getEnabled() == IO_ENABLED) {
            float new_vv = a_in[a]->getValue();
            if (std::abs(ai_vv[a] - new_vv) >= analog_change_threshold) {
                ai_vv[a] = new_vv; // only record if goes past threshold!
                sr_request_status_report(SR_REQUEST_TIMED);
            }
        }
    }
    #endif

    // static uint8_t sent = false;
    // if (!sent) {
    //     sent = 2;
    //     encoder_0.getAngleFraction(read_encoder_0);
    //     encoder_1.getAngleFraction(read_encoder_1);
    // } else if (sent > 1 && sent < 80) {
    //     sent++;
    // }

    // if (sent == 3) {
    //     first_part(false);
    // }

    return STAT_OK;
}



/*
 * hw_hard_reset() - reset system now
 * hw_flash_loader() - enter flash loader to reflash board
 */

void hw_hard_reset(void)
{
    Motate::System::reset(/*boootloader: */ false); // arg=0 resets the system
}

void hw_flash_loader(void)
{
    Motate::System::reset(/*boootloader: */ true);  // arg=1 erases FLASH and enters FLASH loader
}

/*
 * _get_id() - get a human readable signature
 *
 *	Produce a unique deviceID based on the factory calibration data.
 *	Truncate to SYS_ID_DIGITS length
 */

void _get_id(char *id)
{
    char *p = id;
    const char *uuid = Motate::UUID;

    Motate::strncpy(p, uuid, strlen(uuid));
}

/***** END OF SYSTEM FUNCTIONS *****/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * hw_get_fb()  - get firmware build number
 * hw_get_fv()  - get firmware version number
 * hw_get_hp()  - get hardware platform string
 * hw_get_hv()  - get hardware version string
 * hw_get_fbs() - get firmware build string
 */

stat_t hw_get_fb(nvObj_t *nv) { return (get_float(nv, cs.fw_build)); }
stat_t hw_get_fv(nvObj_t *nv) { return (get_float(nv, cs.fw_version)); }
stat_t hw_get_hp(nvObj_t *nv) { return (get_string(nv, G2CORE_HARDWARE_PLATFORM)); }
stat_t hw_get_hv(nvObj_t *nv) { return (get_string(nv, G2CORE_HARDWARE_VERSION)); }
stat_t hw_get_fbs(nvObj_t *nv) { return (get_string(nv, G2CORE_FIRMWARE_BUILD_STRING)); }

/*
 * hw_get_fbc() - get configuration settings file
 */

stat_t hw_get_fbc(nvObj_t *nv)
{
    nv->valuetype = TYPE_STRING;
#ifdef SETTINGS_FILE
#define settings_file_string1(s) #s
#define settings_file_string2(s) settings_file_string1(s)
    ritorno(nv_copy_string(nv, settings_file_string2(SETTINGS_FILE)));
#undef settings_file_string1
#undef settings_file_string2
#else
    ritorno(nv_copy_string(nv, "<default-settings>"));
#endif

    return (STAT_OK);
}

/*
 * hw_get_id() - get device ID (signature)
 */

stat_t hw_get_id(nvObj_t *nv)
{
    char tmp[SYS_ID_LEN];
    _get_id(tmp);
    nv->valuetype = TYPE_STRING;
    ritorno(nv_copy_string(nv, tmp));
    return (STAT_OK);
}

/*
 * hw_flash() - invoke FLASH loader from command input
 */
stat_t hw_flash(nvObj_t *nv)
{
    hw_flash_loader();
    return(STAT_OK);
}

#if !HAS_LASER

// Stub in getSysConfig_3
// constexpr cfgItem_t sys_config_items_3[] = {};
constexpr cfgSubtableFromStaticArray sys_config_3{};
const configSubtable * const getSysConfig_3() { return &sys_config_3; }

#else

stat_t set_pulse_duration(nvObj_t *nv)
{
    laser_tool.set_pulse_duration_us(nv->valuetype == TYPE_FLOAT ? nv->value_flt : nv->value_int);
    return (STAT_OK);
}
stat_t get_pulse_duration(nvObj_t *nv)
{
    nv->value_int = laser_tool.get_pulse_duration_us();
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

stat_t get_min_s(nvObj_t *nv) {
    nv->value_flt = laser_tool.get_min_s();
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}
stat_t set_min_s(nvObj_t *nv) {
    laser_tool.set_min_s(nv->value_flt);
    return (STAT_OK);
}

stat_t get_max_s(nvObj_t *nv) {
    nv->value_flt = laser_tool.get_max_s();
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}
stat_t set_max_s(nvObj_t *nv) {
    laser_tool.set_max_s(nv->value_flt);
    return (STAT_OK);
}

stat_t get_min_ppm(nvObj_t *nv) {
    nv->value_flt = laser_tool.get_min_ppm();
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}
stat_t set_min_ppm(nvObj_t *nv) {
    laser_tool.set_min_ppm(nv->value_flt);
    return (STAT_OK);
}

stat_t get_max_ppm(nvObj_t *nv) {
    nv->value_flt = laser_tool.get_max_ppm();
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}
stat_t set_max_ppm(nvObj_t *nv) {
    laser_tool.set_max_ppm(nv->value_flt);
    return (STAT_OK);
}

constexpr cfgItem_t sys_config_items_3[] = {
    { "th2","th2pd", _iip,  0, tx_print_nul, get_pulse_duration, set_pulse_duration, nullptr, LASER_PULSE_DURATION },
    { "th2","th2mns", _fip,  0, tx_print_nul, get_min_s, set_min_s, nullptr, LASER_MIN_S },
    { "th2","th2mxs", _fip,  0, tx_print_nul, get_max_s, set_max_s, nullptr, LASER_MAX_S },
    { "th2","th2mnp", _fip,  0, tx_print_nul, get_min_ppm, set_min_ppm, nullptr, LASER_MIN_PPM },
    { "th2","th2mxp", _fip,  0, tx_print_nul, get_max_ppm, set_max_ppm, nullptr, LASER_MAX_PPM },
};

constexpr cfgSubtableFromStaticArray sys_config_3{sys_config_items_3};
const configSubtable * const getSysConfig_3() { return &sys_config_3; }

#endif

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

    static const char fmt_fb[] =  "[fb]  firmware build%18.2f\n";
    static const char fmt_fv[] =  "[fv]  firmware version%16.2f\n";
    static const char fmt_fbs[] = "[fbs] firmware build%34s\n";
    static const char fmt_fbc[] = "[fbc] firmware config%33s\n";
    static const char fmt_hp[] =  "[hp]  hardware platform%15s\n";
    static const char fmt_hv[] =  "[hv]  hardware version%13s\n";
    static const char fmt_id[] =  "[id]  g2core ID%37s\n";

    void hw_print_fb(nvObj_t *nv)  { text_print(nv, fmt_fb);}   // TYPE_FLOAT
    void hw_print_fv(nvObj_t *nv)  { text_print(nv, fmt_fv);}   // TYPE_FLOAT
    void hw_print_fbs(nvObj_t *nv) { text_print(nv, fmt_fbs);}  // TYPE_STRING
    void hw_print_fbc(nvObj_t *nv) { text_print(nv, fmt_fbc);}  // TYPE_STRING
    void hw_print_hp(nvObj_t *nv)  { text_print(nv, fmt_hp);}   // TYPE_STRING
    void hw_print_hv(nvObj_t *nv)  { text_print(nv, fmt_hv);}   // TYPE_STRING
    void hw_print_id(nvObj_t *nv)  { text_print(nv, fmt_id);}   // TYPE_STRING

#endif //__TEXT_MODE
