/*
 * hardware.h - system hardware configuration - this file is platform specific
 *			  - ARM version
 *
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 Alden S. Hart, Jr.
 * Copyright (c) 2013 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/> .
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
#ifndef HARDWARE_H_ONCE
#define HARDWARE_H_ONCE

////////////////////////////
/////// ARM VERSION ////////
////////////////////////////

#include "motatePins.h"
#include "motateTimers.h" // for Motate::timer_number

#ifdef __cplusplus
extern "C"{
#endif

/**** Global System Defines ****/

#undef F_CPU							// CPU clock - set for delays
#define F_CPU 84000000UL
#define MILLISECONDS_PER_TICK 1			// MS for system tick (systick * N)
#define SYS_ID_LEN 12					// length of system ID string from sys_get_id()

/************************************************************************************
 **** ARM SAM3X8E SPECIFIC HARDWARE *************************************************
 ************************************************************************************/

/**** Resource Assignment via Motate ****
 *
 * This section defines resource usage for pins, timers, PWM channels, communications
 * and other resources. Please refer to /motate/utility/SamPins.h, SamTimers.h and 
 * other files for pinouts and other configuration details.
 *
 * Commenting out or #ifdef'ing out definitions below will cause the compiler to 
 * drop references to these resources from the compiled code. This will reduce 
 * compiled code size and runtime CPU cycles. E.g. if you are compiling for a 3 motor, 
 * XYZ axis config commenting out the higher motors and axes here will remove them
 * from later code (using the motate .isNull() test).
 */

/* Interrupt usage and priority
 *
 * The following interrupts are defined w/indicated priorities
 *
 *	 0	DDA_TIMER (3) for step pulse generation
 *	 1	DWELL_TIMER (4) for dwell timing
 *	 2	LOADER software generated interrupt (STIR / SGI)
 *	 3	Serial read character interrupt
 *	 4	EXEC software generated interrupt (STIR / SGI)
 *	 5	Serial write character interrupt  
 */

/**** Stepper DDA and dwell timer settings ****/

//#define FREQUENCY_DDA		50000UL
#define FREQUENCY_DDA		100000UL
#define FREQUENCY_DWELL		1000UL
#define FREQUENCY_SGI		200000UL		// 200,000 Hz means software interrupts will fire 5 uSec after being called

/**** Motate Definitions ****/

// Timer definitions. See stepper.h and other headers for setup

Motate::timer_number dda_timer_num   = 2;	// stepper pulse generation in stepper.cpp
Motate::timer_number dwell_timer_num = 3;	// dwell timing in stepper.cpp
Motate::timer_number load_timer_num  = 4;	// request load timer in stepper.cpp
Motate::timer_number exec_timer_num  = 5;	// request exec timer in stepper.cpp

// Pin assignments

Motate::pin_number indicator_led_pin_num = 13;
static Motate::OutputPin<indicator_led_pin_num> IndicatorLed;

// Communications support
Motate::pin_number i2c_sda_pin_num = 20;
Motate::pin_number i2c_scl_pin_num = 21;
Motate::pin_number spi_ss1_pin_num = 10;
Motate::pin_number spi_ss2_pin_num = 47;
Motate::pin_number spi_ss3_pin_num = 52;
Motate::pin_number spi_ss4_pin_num = 48;
Motate::pin_number spi_ss5_pin_num = 49;
Motate::pin_number spi_ss6_pin_num = 50;
Motate::pin_number kinen_sync_pin_num = 53;

// grbl compatibility
Motate::pin_number grbl_reset_pin_num = 54;
Motate::pin_number grbl_feedhold_pin_num = 55;
Motate::pin_number grbl_cycle_start_pin_num = 56;

// Gcode support
Motate::pin_number motor_common_enable_pin_num = 8;
Motate::pin_number spindle_enable_pin_num = 12;
Motate::pin_number spindle_dir_pin_num	  = 13;
Motate::pin_number spindle_pwm_pin_num	  = 11;
Motate::pin_number secondary_pwm_pin_num  = 9;
Motate::pin_number coolant_enable_pin_num = 57;

// axes
Motate::pin_number axis_X_min_pin_num = 14;
Motate::pin_number axis_X_max_pin_num = 15;
Motate::pin_number axis_Y_min_pin_num = 16;
Motate::pin_number axis_Y_max_pin_num = 17;
Motate::pin_number axis_Z_min_pin_num = 18;
Motate::pin_number axis_Z_max_pin_num = 19;

Motate::pin_number axis_A_min_pin_num = 58;
Motate::pin_number axis_A_max_pin_num = 59;
Motate::pin_number axis_B_min_pin_num = 60;
Motate::pin_number axis_B_max_pin_num = 61;
Motate::pin_number axis_C_min_pin_num = 65;
Motate::pin_number axis_C_max_pin_num = 51;

// motors
Motate::pin_number motor_1_step_pin_num			= 2;
Motate::pin_number motor_1_dir_pin_num			= 5;
Motate::pin_number motor_1_enable_pin_num		= 22;
Motate::pin_number motor_1_microstep_0_pin_num	= 23;
Motate::pin_number motor_1_microstep_1_pin_num	= 24;
Motate::pin_number motor_1_vref_pin_num			= 34;

Motate::pin_number motor_2_step_pin_num			= 3;
Motate::pin_number motor_2_dir_pin_num			= 6;
Motate::pin_number motor_2_enable_pin_num		= 25;
Motate::pin_number motor_2_microstep_0_pin_num	= 26;
Motate::pin_number motor_2_microstep_1_pin_num	= 27;
Motate::pin_number motor_2_vref_pin_num			= 62;

#if (MOTORS >= 3)
Motate::pin_number motor_3_step_pin_num			= 4;
Motate::pin_number motor_3_dir_pin_num			= 7;
Motate::pin_number motor_3_enable_pin_num		= 28;
Motate::pin_number motor_3_microstep_0_pin_num	= 29;
Motate::pin_number motor_3_microstep_1_pin_num	= 30;
Motate::pin_number motor_3_vref_pin_num			= 63;
#else
Motate::pin_number motor_3_step_pin_num			= -1;
Motate::pin_number motor_3_dir_pin_num			= -1;
Motate::pin_number motor_3_enable_pin_num		= -1;
Motate::pin_number motor_3_microstep_0_pin_num	= -1;
Motate::pin_number motor_3_microstep_1_pin_num	= -1;
Motate::pin_number motor_3_vref_pin_num			= -1;
#endif

#if (MOTORS >= 4)
Motate::pin_number motor_4_step_pin_num			= 31;
Motate::pin_number motor_4_dir_pin_num			= 32;
Motate::pin_number motor_4_enable_pin_num		= 33;
Motate::pin_number motor_4_microstep_0_pin_num	= 35;
Motate::pin_number motor_4_microstep_1_pin_num	= 36;
Motate::pin_number motor_4_vref_pin_num			= 64;
#else
Motate::pin_number motor_4_step_pin_num			= -1;
Motate::pin_number motor_4_dir_pin_num			= -1;
Motate::pin_number motor_4_enable_pin_num		= -1;
Motate::pin_number motor_4_microstep_0_pin_num	= -1;
Motate::pin_number motor_4_microstep_1_pin_num	= -1;
Motate::pin_number motor_4_vref_pin_num			= -1;
#endif

#if (MOTORS >= 5)
Motate::pin_number motor_5_step_pin_num			= 37;
Motate::pin_number motor_5_dir_pin_num			= 38;
Motate::pin_number motor_5_enable_pin_num		= 39;
Motate::pin_number motor_5_microstep_0_pin_num	= 40;
Motate::pin_number motor_5_microstep_1_pin_num	= 41;
Motate::pin_number motor_5_vref_pin_num			= 66;
#else
Motate::pin_number motor_5_step_pin_num			= -1;
Motate::pin_number motor_5_dir_pin_num			= -1;
Motate::pin_number motor_5_enable_pin_num		= -1;
Motate::pin_number motor_5_microstep_0_pin_num	= -1;
Motate::pin_number motor_5_microstep_1_pin_num	= -1;
Motate::pin_number motor_5_vref_pin_num			= -1;
#endif

#if (MOTORS >= 6)
Motate::pin_number motor_6_step_pin_num			= 42;
Motate::pin_number motor_6_dir_pin_num			= 43;
Motate::pin_number motor_6_enable_pin_num		= 44;
Motate::pin_number motor_6_microstep_0_pin_num	= 45;
Motate::pin_number motor_6_microstep_1_pin_num	= 46;
Motate::pin_number motor_6_vref_pin_num			= 67;
#else
Motate::pin_number motor_6_step_pin_num			= -1;
Motate::pin_number motor_6_dir_pin_num			= -1;
Motate::pin_number motor_6_enable_pin_num		= -1;
Motate::pin_number motor_6_microstep_0_pin_num	= -1;
Motate::pin_number motor_6_microstep_1_pin_num	= -1;
Motate::pin_number motor_6_vref_pin_num			= -1;
#endif

/**** Motate Global Pin Allocations ****/

static Motate::OutputPin<spi_ss1_pin_num> spi_ss1_pin;
static Motate::OutputPin<spi_ss2_pin_num> spi_ss2_pin;
static Motate::OutputPin<spi_ss3_pin_num> spi_ss3_pin;
static Motate::OutputPin<spi_ss4_pin_num> spi_ss4_pin;
static Motate::OutputPin<spi_ss5_pin_num> spi_ss5_pin;
static Motate::OutputPin<spi_ss6_pin_num> spi_ss6_pin;
static Motate::OutputPin<kinen_sync_pin_num> kinen_sync_pin;

static Motate::OutputPin<grbl_reset_pin_num> grbl_reset_pin;
static Motate::OutputPin<grbl_feedhold_pin_num> grbl_feedhold_pin;
static Motate::OutputPin<grbl_cycle_start_pin_num> grbl_cycle_start_pin;

static Motate::OutputPin<motor_common_enable_pin_num> motor_common_enable_pin;
static Motate::OutputPin<spindle_enable_pin_num> spindle_enable_pin;
static Motate::OutputPin<spindle_dir_pin_num> spindle_dir_pin;
static Motate::OutputPin<spindle_pwm_pin_num> spindle_pwm_pin;
static Motate::OutputPin<secondary_pwm_pin_num> secondary_pwm_pin;
static Motate::OutputPin<coolant_enable_pin_num> coolant_enable_pin;

static Motate::InputPin<axis_X_min_pin_num> axis_X_min_pin(Motate::kPullUp);
static Motate::InputPin<axis_X_max_pin_num> axis_X_max_pin(Motate::kPullUp);
static Motate::InputPin<axis_Y_min_pin_num> axis_Y_min_pin(Motate::kPullUp);
static Motate::InputPin<axis_Y_max_pin_num> axis_Y_max_pin(Motate::kPullUp);
static Motate::InputPin<axis_Z_min_pin_num> axis_Z_min_pin(Motate::kPullUp);
static Motate::InputPin<axis_Z_max_pin_num> axis_Z_max_pin(Motate::kPullUp);

static Motate::InputPin<axis_A_min_pin_num> axis_A_min_pin(Motate::kPullUp);
static Motate::InputPin<axis_A_max_pin_num> axis_A_max_pin(Motate::kPullUp);
static Motate::InputPin<axis_B_min_pin_num> axis_B_min_pin(Motate::kPullUp);
static Motate::InputPin<axis_B_max_pin_num> axis_B_max_pin(Motate::kPullUp);
static Motate::InputPin<axis_C_min_pin_num> axis_C_min_pin(Motate::kPullUp);
static Motate::InputPin<axis_C_max_pin_num> axis_C_max_pin(Motate::kPullUp);

/*** function prototypes ***/

void hardware_init(void);			// master hardware init
void hw_request_hard_reset();
void hw_hard_reset(void);
stat_t hw_hard_reset_handler(void);

void hw_request_bootloader(void);
stat_t hw_bootloader_handler(void);
stat_t hw_run_boot(cmdObj_t *cmd);

stat_t hw_set_hv(cmdObj_t *cmd);
stat_t hw_get_id(cmdObj_t *cmd);

#ifdef __TEXT_MODE

	void hw_print_fb(cmdObj_t *cmd);
	void hw_print_fv(cmdObj_t *cmd);
	void hw_print_hp(cmdObj_t *cmd);
	void hw_print_hv(cmdObj_t *cmd);
	void hw_print_id(cmdObj_t *cmd);

#else

	#define hw_print_fb tx_print_stub
	#define hw_print_fv tx_print_stub
	#define hw_print_hp tx_print_stub
	#define hw_print_hv tx_print_stub
	#define hw_print_id tx_print_stub

#endif // __TEXT_MODE

#ifdef __cplusplus
}
#endif

#endif	// end of include guard: HARDWARE_H_ONCE
