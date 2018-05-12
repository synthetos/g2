/*
 * hardware.cpp - general hardware support functions
 * For: /board/printrboardg2
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
#include "planner.h"
#include "text_parser.h"
#include "board_xio.h"
#include "util.h"


#include "MotateUtilities.h"
#include "MotateUniqueID.h"
#include "MotatePower.h"

#include "neopixel.h"

namespace LEDs {
    NeoPixel<Motate::kLED_RGBWPixelPinNumber, 3> rgbw_leds {NeoPixelOrder::GRBW};

    RGB_Color_t display_color[3] {{1, 1, 1, 5}, {1, 1, 1, 5}, {1, 1, 1, 5}};

    bool alarm_red = false; // if we are in alarm, the tells us if we're going to red (pulsing)
    bool shutdown_white = false; // if we are in shutdown, the tells us if we're going to red (pulsing)
    cmMachineState last_see_machine_state;
}

/*
 * hardware_init() - lowest level hardware init
 */

void hardware_init()
{
    board_hardware_init();
    LEDs::last_see_machine_state = cm_get_machine_state();

    for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
        LEDs::display_color[pixel].startTransition(0, 0, 0, 0);
        LEDs::rgbw_leds.setPixel(pixel, LEDs::display_color[pixel]);
    }
    LEDs::rgbw_leds.update();
}

/*
 * hardware_periodic() - callback from the controller loop - TIME CRITICAL.
 */

stat_t hardware_periodic()
{
    // If we're very time constrained, get out. This isn't critical
    if (!mp_is_phat_city_time()) { return STAT_OK; }

    auto new_machine_state = cm_get_machine_state();

    // Handle machines states from most important to least
    if (new_machine_state == MACHINE_PANIC) {
        if ((LEDs::last_see_machine_state != new_machine_state))
        {
            for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
                LEDs::display_color[pixel].startTransition(5000, 1, 0, 0);
            }
        }
    } else if (new_machine_state == MACHINE_SHUTDOWN) {
        if ((LEDs::last_see_machine_state != new_machine_state) || LEDs::display_color[0].isTransitionDone())
        {
            if (LEDs::shutdown_white) {
                // set to black
                for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
                    LEDs::display_color[pixel].startTransition(2000, 0, 0, 0);
                }
            } else {
                // set to shutdown white
                for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
                    LEDs::display_color[pixel].startTransition(2000, 0.5, 0.5, 0.5);
                }
            }
            LEDs::shutdown_white = !LEDs::shutdown_white;
        }
    } else if (new_machine_state == MACHINE_ALARM) {
        if ((LEDs::last_see_machine_state != new_machine_state) || LEDs::display_color[0].isTransitionDone())
        {
            if (LEDs::alarm_red) {
                // set to black
                for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
                    LEDs::display_color[pixel].startTransition(500, 0, 0, 0);
                }
            } else {
                // set to red
                for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
                    LEDs::display_color[pixel].startTransition(500, 1, 0, 0);
                }
            }
            LEDs::alarm_red = !LEDs::alarm_red;
        }

//    // Transistion to HOMING state, fade to white
//    } else if ((LEDs::last_see_machine_state != MACHINE_CYCLE) && (new_machine_state == MACHINE_CYCLE)) {
//        // set to white
//        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
//            LEDs::display_color[pixel].startTransition(5000, 1, 1, 1);
//        }
    // catch transition from alarm to reset to black
    } else if ((LEDs::last_see_machine_state == MACHINE_ALARM) && (new_machine_state != MACHINE_ALARM)) {
        // set to black
        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
            LEDs::display_color[pixel].startTransition(2000, 0, 0, 0);
        }
        LEDs::alarm_red = false;

    } else if ((LEDs::last_see_machine_state == MACHINE_READY) && (new_machine_state != MACHINE_READY)) {
        // set to black
        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
            LEDs::display_color[pixel].startTransition(0, 0, 0, 0);
        }
//    } else if ((LEDs::last_see_machine_state == MACHINE_READY)) {
//        // set to white
//        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
//            LEDs::display_color[pixel].startTransition(0, 1, 1, 1);
//        }
    }

    LEDs::last_see_machine_state = new_machine_state;

    for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
        if (LEDs::display_color[pixel].update()) {
            LEDs::rgbw_leds.setPixel(pixel, LEDs::display_color[pixel]);
        }
    }

    LEDs::rgbw_leds.update();

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

    Motate::strncpy(p, uuid, Motate::strlen(uuid));
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

stat_t _get_leds(nvObj_t *nv)
{
    nv->valuetype = TYPE_INTEGER;

    float red, green, blue;

    LEDs::display_color[0].getRGB(red, green, blue);

    // black
    if (fp_EQ(red, 0.0) && fp_EQ(green, 0.0) && fp_EQ(blue, 0.0)) {
        nv->value_int = 0;
    // white
    } else if (fp_EQ(red, 1.0) && fp_EQ(green, 1.0) && fp_EQ(blue, 1.0)) {
        nv->value_int = 1;
    // red
    } else if (fp_EQ(red, 1.0) && fp_EQ(green, 0.0) && fp_EQ(blue, 0.0)) {
        nv->value_int = 2;
    // green
    } else if (fp_EQ(red, 0.0) && fp_EQ(green, 1.0) && fp_EQ(blue, 0.0)) {
        nv->value_int = 3;
    // blue
    } else if (fp_EQ(red, 0.0) && fp_EQ(green, 0.0) && fp_EQ(blue, 1.0)) {
        nv->value_int = 4;
    // orange
    } else if (fp_EQ(red, 1.0) && fp_EQ(green, 0.5) && fp_EQ(blue, 0.0)) {
        nv->value_int = 5;
    // yellow
    } else if (fp_EQ(red, 1.0) && fp_EQ(green, 1.0) && fp_EQ(blue, 0.0)) {
        nv->value_int = 6;
    }
    return (STAT_OK);
}

stat_t _set_leds(nvObj_t *nv)
{
    uint32_t value = nv->value_int;
//    if ((nv->value_int < 0) || (nv->value_int > 6)) {
    if ((value < 0) || (value > 6)) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }

    // black
    if (value == 0) {
        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
            LEDs::display_color[pixel].startTransition(100, 0, 0, 0);
        }
    // white
    } else if (value == 1) {
        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
            LEDs::display_color[pixel].startTransition(100, 1.0, 1.0, 1.0);
        }
    // red
    } else if (value == 2) {
        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
            LEDs::display_color[pixel].startTransition(100, 1.0, 0.0, 0.0);
        }
    // green
    } else if (value == 3) {
        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
            LEDs::display_color[pixel].startTransition(100, 0.0, 1.0, 0.0);
        }
    // blue
    } else if (value == 4) {
        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
            LEDs::display_color[pixel].startTransition(100, 0.0, 0.0, 1.0);
        }
    // orange
    } else if (value == 5) {
        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
            LEDs::display_color[pixel].startTransition(100, 1.0, 0.5, 0.0);
        }
    // yellow
    } else if (value == 6) {
        for (uint8_t pixel = 0; pixel < LEDs::rgbw_leds.count; pixel++) {
            LEDs::display_color[pixel].startTransition(100, 1.0, 1.0, 0.0);
        }
    }
    return (STAT_OK);
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
