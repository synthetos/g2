/*
 * dro.cpp - digital measurement and readout functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2014 Robert Giseburt
 * Copyright (c) 2014 Alden S. Hart, Jr.
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
/* 	This module provides the low-level stepper drivers and some related functions.
 *	See stepper.h for a detailed explanation of this module.
 */

#include "tinyg2.h"
#include "config.h"
#include "dro.h"
#include "encoder.h"
#include "planner.h"
#include "canonical_machine.h"
#include "kinematics.h"
#include "hardware.h"
#include "report.h"
#include "text_parser.h"
#include "util.h"

/**** Allocate structures ****/

stConfig_t st_cfg;
stPrepSingleton_t st_pre;

#ifdef DIGITAL_DRO

#ifdef __ARM
using namespace Motate;

OutputPin<kGRBL_CommonEnablePinNumber> common_enable;	 // shorter form of the above
OutputPin<kDebug1_PinNumber> dda_debug_pin1;
OutputPin<kDebug2_PinNumber> dda_debug_pin2;
OutputPin<kDebug3_PinNumber> dda_debug_pin3;

// Motor structures
template<pin_number step_num,			// Setup a stepper template to hold our pins
		 pin_number dir_num, 
		 pin_number enable_num, 
		 pin_number ms0_num, 
		 pin_number ms1_num, 
         pin_number ms2_num,
		 pin_number vref_num,
         uint8_t    stepper_number>
struct Stepper {
    // DIGITAL_DRO

    Pin<step_num>   _step_pin;
	Pin<dir_num>    _dir_pin;
	Pin<enable_num> _enable_pin;
	Pin<ms0_num>    _ms0_pin;
	Pin<ms1_num>    _ms1_pin;
	Pin<ms2_num>    _ms2_pin;
//	ADCPin<vref_num>     _vref_pin;

    volatile int32_t             _position;

    Stepper() : _step_pin(kInput),
                _dir_pin(kInput),
                _enable_pin(kInput),
                _ms0_pin(kInput),
                _ms1_pin(kInput),
                _ms2_pin(kInput),
                _position(0)
    {
        _enable_pin.setInterrupts(kPinInterruptOnChange);
        enableChanged();
    };

    bool isEnabled() {
        // Enable is active low
        return (0 == _enable_pin.getInputValue());
    };

    void enableChanged() {
        if (isEnabled()) {
            _step_pin.setInterrupts(kPinInterruptOnFallingEdge);
        }
    };

    void stepped() {
#ifdef READ_MICROSTEPS
        uint8_t microstep_size = 1 << (_ms0_pin.getInputValue() | (_ms1_pin.getInputValue() << 1) | (_ms2_pin.getInputValue() << 2));
        _position += (_dir_pin.getInputValue()) ? microstep_size : -microstep_size;
#else
//        uint8_t microstep_size = st_cfg.mot[stepper_number].microsteps;

        _position += (_dir_pin.getInputValue()) ? 1 : -1;
#endif

        mr.position_steps[stepper_number] = _position;
		kin_forward_kinematics(mr.target, mr.position_steps);

        sr_request_status_report(SR_TIMED_REQUEST);
    };
};

Stepper<kSocket1_StepPinNumber,
		kSocket1_DirPinNumber,
		kSocket1_EnablePinNumber,
		kSocket1_Microstep_0PinNumber,
		kSocket1_Microstep_1PinNumber,
        kSocket1_Microstep_2PinNumber,
		kSocket1_VrefPinNumber,
        MOTOR_1> motor_1;

namespace Motate {
    void Pin<kSocket1_EnablePinNumber>::interrupt() {
        motor_1.enableChanged();
    }

    void Pin<kSocket1_StepPinNumber>::interrupt() {
        motor_1.stepped();
    }
}

Stepper<kSocket2_StepPinNumber,
		kSocket2_DirPinNumber,
		kSocket2_EnablePinNumber,
		kSocket2_Microstep_0PinNumber,
		kSocket2_Microstep_1PinNumber,
        kSocket2_Microstep_2PinNumber,
        kSocket2_VrefPinNumber,
        MOTOR_2> motor_2;

namespace Motate {
    void Pin<kSocket2_EnablePinNumber>::interrupt() {
        motor_2.enableChanged();
    }

    void Pin<kSocket2_StepPinNumber>::interrupt() {
        motor_2.stepped();
    }
}

Stepper<kSocket3_StepPinNumber,
		kSocket3_DirPinNumber,
		kSocket3_EnablePinNumber,
		kSocket3_Microstep_0PinNumber,
		kSocket3_Microstep_1PinNumber,
        kSocket3_Microstep_2PinNumber,
        kSocket3_VrefPinNumber,
        MOTOR_3> motor_3;


namespace Motate {
    void Pin<kSocket3_EnablePinNumber>::interrupt() {
        motor_3.enableChanged();
    }

    void Pin<kSocket3_StepPinNumber>::interrupt() {
        motor_3.stepped();
    }
}

Stepper<kSocket4_StepPinNumber,
		kSocket4_DirPinNumber,
		kSocket4_EnablePinNumber,
		kSocket4_Microstep_0PinNumber,
		kSocket4_Microstep_1PinNumber,
        kSocket4_Microstep_2PinNumber,
        kSocket4_VrefPinNumber,
        MOTOR_4> motor_4;


namespace Motate {
    void Pin<kSocket4_EnablePinNumber>::interrupt() {
        motor_4.enableChanged();
    }

    void Pin<kSocket4_StepPinNumber>::interrupt() {
        motor_4.stepped();
    }
}

Stepper<kSocket5_StepPinNumber,
		kSocket5_DirPinNumber,
		kSocket5_EnablePinNumber,
		kSocket5_Microstep_0PinNumber,
		kSocket5_Microstep_1PinNumber,
        kSocket5_Microstep_2PinNumber,
        kSocket5_VrefPinNumber,
        MOTOR_5> motor_5;


namespace Motate {
    void Pin<kSocket5_EnablePinNumber>::interrupt() {
        motor_5.enableChanged();
    }

    void Pin<kSocket5_StepPinNumber>::interrupt() {
        motor_5.stepped();
    }
}

Stepper<kSocket6_StepPinNumber,
		kSocket6_DirPinNumber,
		kSocket6_EnablePinNumber,
		kSocket6_Microstep_0PinNumber,
		kSocket6_Microstep_1PinNumber,
        kSocket6_Microstep_2PinNumber,
        kSocket6_VrefPinNumber,
        MOTOR_6> motor_6;


namespace Motate {
    void Pin<kSocket6_EnablePinNumber>::interrupt() {
        motor_6.enableChanged();
    }

    void Pin<kSocket6_StepPinNumber>::interrupt() {
        motor_6.stepped();
    }
}


#endif // __ARM

/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/
/*
 * stepper_init() - initialize stepper motor subsystem 
 *
 *	Notes:
 *	  - This init requires sys_init() to be run beforehand
 * 	  - microsteps are setup during config_init()
 *	  - motor polarity is setup during config_init()
 */

void stepper_init()
{
}


/***********************************************************************************
 * STUB FUNCTIONS
 * Functions to make it seem like we have motor, when we're really just reading them
 * from somewhere else
 ***********************************************************************************/
void st_request_exec_move()
{

}


void st_prep_null()
{

}

void st_prep_dwell(float)
{

}

void st_deenergize_motors()
{

}

uint8_t stepper_isbusy()
{
    return false;
}

void st_motor_power_callback()
{

}


stat_t st_prep_line(float travel_steps[], float following_error[], float segment_time)
{
	return (STAT_OK);
}

stat_t stepper_test_assertions()
{
	return (STAT_OK);
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/* HELPERS
 * _get_motor() - helper to return motor number as an index or -1 if na
 */

static int8_t _get_motor(const index_t index)
{
	char_t *ptr;
	char_t motors[] = {"123456"};
	char_t tmp[TOKEN_LEN+1];

	strcpy_P(tmp, cfgArray[index].group);
	if ((ptr = strchr(motors, tmp[0])) == NULL) {
		return (-1);
	}
	return (ptr - motors);
}

/*
 * _set_motor_steps_per_unit() - what it says
 * This function will need to be rethought if microstep morphing is implemented
 */

static void _set_motor_steps_per_unit(cmdObj_t *cmd) 
{
	uint8_t m = _get_motor(cmd->index);
	st_cfg.mot[m].units_per_step = (st_cfg.mot[m].travel_rev * st_cfg.mot[m].step_angle) / (360 * st_cfg.mot[m].microsteps);
	st_cfg.mot[m].steps_per_unit = 1 / st_cfg.mot[m].units_per_step;
}

/* PER-MOTOR FUNCTIONS
 * st_set_sa() - set motor step angle
 * st_set_tr() - set travel per motor revolution
 * st_set_mi() - set motor microsteps
 * st_set_pm() - set motor power mode
 * st_set_pl() - set motor power level
 */

stat_t st_set_sa(cmdObj_t *cmd)			// motor step angle
{ 
	set_flt(cmd);
	_set_motor_steps_per_unit(cmd); 
	return(STAT_OK);
}

stat_t st_set_tr(cmdObj_t *cmd)			// motor travel per revolution
{ 
	set_flu(cmd);
	_set_motor_steps_per_unit(cmd); 
	return(STAT_OK);
}

stat_t st_set_mi(cmdObj_t *cmd)			// motor microsteps
{
	if (fp_NE(cmd->value,1) && fp_NE(cmd->value,2) && fp_NE(cmd->value,4) && fp_NE(cmd->value,8)) {
		cmd_add_conditional_message((const char_t *)"*** WARNING *** Setting non-standard microstep value");
	}
	set_ui8(cmd);						// set it anyway, even if it's unsupported
	_set_motor_steps_per_unit(cmd);
//	_set_hw_microsteps(_get_motor(cmd->index), (uint8_t)cmd->value);
	return (STAT_OK);
}

stat_t st_set_pm(cmdObj_t *cmd)			// motor power mode
{
#if 0
	if (cmd->value >= MOTOR_POWER_MODE_MAX_VALUE) return (STAT_INPUT_VALUE_UNSUPPORTED);
	set_ui8(cmd);

	if (fp_ZERO(cmd->value)) { // people asked this setting take effect immediately, hence:
		_energize_motor(_get_motor(cmd->index));
	} else {
		_deenergize_motor(_get_motor(cmd->index));
	}
#endif
	return (STAT_OK);
}

/*
 * st_set_pl() - set motor power level
 *
 *	Input value may vary from 0.000 to 1.000 The setting is scaled to allowable PWM range.
 *	This function sets both the scaled and dynamic power levels, and applies the 
 *	scaled value to the vref.
 */ 
stat_t st_set_pl(cmdObj_t *cmd)	// motor power level
{
#if 0
#ifdef __ARM
	if (cmd->value < (float)0.0) cmd->value = 0.0;
	if (cmd->value > (float)1.0) {
		if (cmd->value > (float)100) cmd->value = 1;
 		cmd->value /= 100;		// accommodate old 0-100 inputs
	}
	set_flt(cmd);	// set power_setting value in the motor config struct (st)
	
	uint8_t motor = _get_motor(cmd->index);
	st_cfg.mot[motor].power_level_scaled = (cmd->value * POWER_LEVEL_SCALE_FACTOR);
	st_run.mot[motor].power_level_dynamic = (st_cfg.mot[motor].power_level_scaled);
	_set_motor_power_level(motor, st_cfg.mot[motor].power_level_scaled);
#endif
#endif
	return(STAT_OK);
}

/* GLOBAL FUNCTIONS (SYSTEM LEVEL)
 *
 * st_set_mt() - set motor timeout in seconds
 * st_set_md() - disable motor power
 * st_set_me() - enable motor power
 *
 * Calling me or md with NULL will enable or disable all motors
 * Setting a value of 0 will enable or disable all motors
 * Setting a value from 1 to MOTORS will enable or disable that motor only
 */

stat_t st_set_mt(cmdObj_t *cmd)
{
#if 0
	st_cfg.motor_power_timeout = min(POWER_TIMEOUT_SECONDS_MAX, max(cmd->value, POWER_TIMEOUT_SECONDS_MIN));
#endif
	return (STAT_OK);
}

stat_t st_set_md(cmdObj_t *cmd)	// Make sure this function is not part of initialization --> f00
{
#if 0
	if (((uint8_t)cmd->value == 0) || (cmd->objtype == TYPE_NULL)) {
		st_deenergize_motors();
	} else {
		_deenergize_motor((uint8_t)cmd->value-1);
	}
#endif
	return (STAT_OK);
}

stat_t st_set_me(cmdObj_t *cmd)	// Make sure this function is not part of initialization --> f00
{
#if 0
	if (((uint8_t)cmd->value == 0) || (cmd->objtype == TYPE_NULL)) {
		st_energize_motors();
	} else {
		_energize_motor((uint8_t)cmd->value-1);
	}
#endif
	return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char msg_units0[] PROGMEM = " in";	// used by generic print functions
static const char msg_units1[] PROGMEM = " mm";
static const char msg_units2[] PROGMEM = " deg";
static const char *const msg_units[] PROGMEM = { msg_units0, msg_units1, msg_units2 };
#define DEGREE_INDEX 2

static const char fmt_me[] PROGMEM = "motors energized\n";
static const char fmt_md[] PROGMEM = "motors de-energized\n";
static const char fmt_mt[] PROGMEM = "[mt]  motor idle timeout%14.2f Sec\n";
static const char fmt_0ma[] PROGMEM = "[%s%s] m%s map to axis%15d [0=X,1=Y,2=Z...]\n";
static const char fmt_0sa[] PROGMEM = "[%s%s] m%s step angle%20.3f%s\n";
static const char fmt_0tr[] PROGMEM = "[%s%s] m%s travel per revolution%10.4f%s\n";
static const char fmt_0mi[] PROGMEM = "[%s%s] m%s microsteps%16d [1,2,4,8]\n";
static const char fmt_0po[] PROGMEM = "[%s%s] m%s polarity%18d [0=normal,1=reverse]\n";
static const char fmt_0pm[] PROGMEM = "[%s%s] m%s power management%10d [0=disabled,1=always on,2=in cycle,3=when moving]\n";
static const char fmt_0pl[] PROGMEM = "[%s%s] m%s motor power level%13.3f [0.000=minimum, 1.000=maximum]\n";

void st_print_mt(cmdObj_t *cmd) { text_print_flt(cmd, fmt_mt);}
void st_print_me(cmdObj_t *cmd) { text_print_nul(cmd, fmt_me);}
void st_print_md(cmdObj_t *cmd) { text_print_nul(cmd, fmt_md);}

static void _print_motor_ui8(cmdObj_t *cmd, const char *format)
{
	fprintf_P(stderr, format, cmd->group, cmd->token, cmd->group, (uint8_t)cmd->value);
}

static void _print_motor_flt_units(cmdObj_t *cmd, const char *format, uint8_t units)
{
	fprintf_P(stderr, format, cmd->group, cmd->token, cmd->group, cmd->value, GET_TEXT_ITEM(msg_units, units));
}

static void _print_motor_flt(cmdObj_t *cmd, const char *format)
{
	fprintf_P(stderr, format, cmd->group, cmd->token, cmd->group, cmd->value);
}

void st_print_ma(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0ma);}
void st_print_sa(cmdObj_t *cmd) { _print_motor_flt_units(cmd, fmt_0sa, DEGREE_INDEX);}
void st_print_tr(cmdObj_t *cmd) { _print_motor_flt_units(cmd, fmt_0tr, cm_get_units_mode(MODEL));}
void st_print_mi(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0mi);}
void st_print_po(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0po);}
void st_print_pm(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0pm);}
void st_print_pl(cmdObj_t *cmd) { _print_motor_flt(cmd, fmt_0pl);}

#endif // __TEXT_MODE

#endif // DIGITAL_DRO