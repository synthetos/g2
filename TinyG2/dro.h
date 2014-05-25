/*
 * dro.h - digital measurement and readout functions
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

#ifndef DRO_H_ONCE
#define DRO_H_ONCE

//#define DIGITAL_DRO
//#define READ_MICROSTEPS

// Motor config structure

typedef struct cfgMotor {				// per-motor configs
	// public
	uint8_t	motor_map;					// map motor to axis
	uint8_t microsteps;					// microsteps to apply for each axis (ex: 8)
	uint8_t polarity;					// 0=normal polarity, 1=reverse motor direction
	uint8_t power_mode;					// See cmMotorPowerMode for enum
	float power_level;					// set 0.000 to 1.000 for PMW vref setting
	float step_angle;					// degrees per whole step (ex: 1.8)
	float travel_rev;					// mm or deg of travel per motor revolution
	float steps_per_unit;				// microsteps per mm (or degree) of travel
	float units_per_step;				// mm or degrees of travel per microstep

	// private
	float power_level_scaled;			// scaled to internal range - must be between 0 and 1
} cfgMotor_t;

typedef struct stConfig {				// stepper configs
	float motor_power_timeout;			// seconds before setting motors to idle current (currently this is OFF)
	cfgMotor_t mot[MOTORS];				// settings for motors 1-N
} stConfig_t;


typedef struct stPrepMotor {
	// direction and direction change
	uint8_t direction;					// travel direction corrected for polarity (CW==0. CCW==1)
	uint8_t prev_direction;				// travel direction from previous segment run for this motor
	int8_t step_sign;					// set to +1 or -1 for encoders

	float corrected_steps;				// accumulated correction steps for the cycle (for diagnostic display only)
} stPrepMotor_t;

typedef struct stPrepSingleton {
	uint16_t magic_start;				// magic number to test memory integrity

	stPrepMotor_t mot[MOTORS];			// prep time motor structs

	uint16_t magic_end;
} stPrepSingleton_t;

extern stConfig_t st_cfg;				// config struct is used widely
extern stPrepSingleton_t st_pre;		// only used by config_app diagnostics

/**** FUNCTION PROTOTYPES ****/

stat_t st_set_mt(cmdObj_t *cmd);
stat_t st_set_md(cmdObj_t *cmd);
stat_t st_set_me(cmdObj_t *cmd);

stat_t st_set_sa(cmdObj_t *cmd);
stat_t st_set_tr(cmdObj_t *cmd);
stat_t st_set_mi(cmdObj_t *cmd);
stat_t st_set_pm(cmdObj_t *cmd);
stat_t st_set_pl(cmdObj_t *cmd);

#ifdef __TEXT_MODE

	void st_print_mt(cmdObj_t *cmd);
	void st_print_me(cmdObj_t *cmd);
	void st_print_md(cmdObj_t *cmd);
	void st_print_ma(cmdObj_t *cmd);
	void st_print_sa(cmdObj_t *cmd);
	void st_print_tr(cmdObj_t *cmd);
	void st_print_mi(cmdObj_t *cmd);
	void st_print_po(cmdObj_t *cmd);
	void st_print_pm(cmdObj_t *cmd);
	void st_print_pl(cmdObj_t *cmd);

#else

	#define st_print_mt tx_print_stub
	#define st_print_me tx_print_stub
	#define st_print_md tx_print_stub
	#define st_print_ma tx_print_stub
	#define st_print_sa tx_print_stub
	#define st_print_tr tx_print_stub
	#define st_print_mi tx_print_stub
	#define st_print_po tx_print_stub
	#define st_print_pm tx_print_stub
	#define st_print_pl tx_print_stub

#endif // __TEXT_MODE

#endif	// End of include guard: STEPPER_H_ONCE
