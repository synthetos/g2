/*
 * hardware.cpp - general hardware support functions
 * For: /board/g2v9
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2018 Robert Giseburt
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

#include "MotateUtilities.h"
#include "MotateUniqueID.h"
#include "MotatePower.h"
#include "board_spi.h"
#include "sd_persistence.h"

#include "board_gpio.h"

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

SPIBus_used_t spiBus;

Motate::SPIChipSelectPin<Motate::kSD_ChipSelectPinNumber> sdcs{};
SDCard_used_t sd_card{spiBus, sdcs};

#ifdef BANTAM

#include "bantam_safety_manager.h"

BantamSafetyManager sm{};
SafetyManager *safety_manager = &sm;

#else

#include "safety_manager.h"

SafetyManager sm{};
SafetyManager *safety_manager = &sm;

// Stub in getSysConfig_3
// constexpr cfgItem_t sys_config_items_3[] = {};
constexpr cfgSubtableFromStaticArray sys_config_3{};
const configSubtable * const getSysConfig_3() { return &sys_config_3; }

#endif

#include "esc_spindle.h"
ESCSpindle esc_spindle {SPINDLE_PWM_NUMBER, SPINDLE_ENABLE_OUTPUT_NUMBER, SPINDLE_DIRECTION_OUTPUT_NUMBER, SPINDLE_SPEED_CHANGE_PER_MS};

ToolHead *toolhead_for_tool(uint8_t tool) {
    return &esc_spindle;
}

/*
 * hardware_init() - lowest level hardware init
 */

void hardware_init()
{
    spiBus.init();
    sd_card.init();
    setup_sd_persistence();
    board_hardware_init();
    toolhead_for_tool(0)->init();
    spindle_set_toolhead(toolhead_for_tool(0));
    return;
}

/*
 * hardware_periodic() - callback from the controller loop - TIME CRITICAL.
 */

stat_t hardware_periodic()
{
    sd_card.periodicCheck();
    return STAT_OK;
}

/*
 * hw_hard_reset() - reset system now
 * hw_flash_loader() - enter flash loader to reflash board
 */

void hw_hard_reset(void)
{
    Motate::System::reset(/*bootloader: */ false); // arg=0 resets the system
}

void hw_flash_loader(void)
{
    Motate::System::reset(/*bootloader: */ true);  // arg=1 erases FLASH and enters FLASH loader
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

    Motate::strncpy(p, uuid, Motate::strlen(uuid)+1);
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
