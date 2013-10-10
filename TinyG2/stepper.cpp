/*
 * stepper.cpp - stepper motor controls
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart, Jr.
 * Copyright (c) 2013 Robert Giseburt
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
/* 	This module provides the low-level stepper drivers and some related
 * 	functions. It dequeues lines queued by the motor_queue routines.
 * 	This is some of the most heavily optimized code in the project.
 *
 *	Note that if you want to use this for something other than TinyG
 *	you may need to stretch the step pulses. They run about 1 uSec 
 *	which is fine for the TI DRV8811/DRV8818 chips in TinyG but may 
 *	not suffice for other stepper driver hardware.
 */
/* 
 * See stepper.h for a detailed explanation of this module
 */

#include "tinyg2.h"
#include "config.h"
#include "stepper.h"
#include "planner.h"
#include "hardware.h"
#include "text_parser.h"
#include "util.h"

//#define ENABLE_DIAGNOSTICS
#ifdef ENABLE_DIAGNOSTICS
#define INCREMENT_DIAGNOSTIC_COUNTER(motor) st_run.m[motor].step_count_diagnostic++;
#else
#define INCREMENT_DIAGNOSTIC_COUNTER(motor)	// choose this one to disable counters
#endif

/**** Allocate structures ****/

stConfig_t st;
static stRunSingleton_t st_run;
static stPrepSingleton_t st_prep;

/**** Setup local functions ****/

static void _load_move(void);
static void _request_load_move(void);
static void _clear_diagnostic_counters(void);

// handy macro
#define _f_to_period(f) (uint16_t)((float)F_CPU / (float)f)

/**** Setup motate ****/

using namespace Motate;

OutputPin<motor_common_enable_pin_num> common_enable;	 // shorter form of the above
OutputPin<31> dda_debug_pin1;
OutputPin<33> dda_debug_pin2;

// Example with prefixed name::
//Motate::Timer<dda_timer_num> dda_timer(kTimerUpToMatch, FREQUENCY_DDA);			// stepper pulse generation
Timer<dda_timer_num> dda_timer(kTimerUpToMatch, FREQUENCY_DDA);			// stepper pulse generation
Timer<dwell_timer_num> dwell_timer(kTimerUpToMatch, FREQUENCY_DWELL);	// dwell timer
Timer<load_timer_num> load_timer;		// triggers load of next stepper segment
Timer<exec_timer_num> exec_timer;		// triggers calculation of next+1 stepper segment

// Motor structures
template<pin_number step_num,			// Setup a stepper template to hold our pins
		 pin_number dir_num, 
		 pin_number enable_num, 
		 pin_number ms0_num, 
		 pin_number ms1_num, 
		 pin_number vref_num>

struct Stepper {
	OutputPin<step_num> step;
	OutputPin<dir_num> dir;
	OutputPin<enable_num> enable;
	OutputPin<ms0_num> ms0;
	OutputPin<ms1_num> ms1;
	OutputPin<vref_num> vref;
};

Stepper<motor_1_step_pin_num, 
		motor_1_dir_pin_num, 
		motor_1_enable_pin_num, 
		motor_1_microstep_0_pin_num, 
		motor_1_microstep_1_pin_num,
		motor_1_vref_pin_num> motor_1;

Stepper<motor_2_step_pin_num, 
		motor_2_dir_pin_num, 
		motor_2_enable_pin_num, 
		motor_2_microstep_0_pin_num, 
		motor_2_microstep_1_pin_num,
		motor_2_vref_pin_num> motor_2;

Stepper<motor_3_step_pin_num, 
		motor_3_dir_pin_num, 
		motor_3_enable_pin_num, 
		motor_3_microstep_0_pin_num, 
		motor_3_microstep_1_pin_num,
		motor_3_vref_pin_num> motor_3;

Stepper<motor_4_step_pin_num, 
		motor_4_dir_pin_num, 
		motor_4_enable_pin_num, 
		motor_4_microstep_0_pin_num, 
		motor_4_microstep_1_pin_num,
		motor_4_vref_pin_num> motor_4;

Stepper<motor_5_step_pin_num, 
		motor_5_dir_pin_num, 
		motor_5_enable_pin_num, 
		motor_5_microstep_0_pin_num, 
		motor_5_microstep_1_pin_num,
		motor_5_vref_pin_num> motor_5;
		
Stepper<motor_6_step_pin_num, 
		motor_6_dir_pin_num, 
		motor_6_enable_pin_num, 
		motor_6_microstep_0_pin_num, 
		motor_6_microstep_1_pin_num,
		motor_6_vref_pin_num> motor_6;

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
 *	  - high level interrupts must be enabled in main() once all inits are complete
 */

void stepper_init()
{
	memset(&st_run, 0, sizeof(st_run));		// clear all values, pointers and status
	st_run.magic_start = MAGICNUM;
	st_prep.magic_start = MAGICNUM;
	_clear_diagnostic_counters();

	// setup DDA timer (see FOOTNOTE)
	dda_timer.setInterrupts(kInterruptOnOverflow | kInterruptOnMatchA | kInterruptPriorityHighest);
	dda_timer.setDutyCycleA(0.25);

	// setup DWELL timer
	dwell_timer.setInterrupts(kInterruptOnOverflow | kInterruptPriorityHighest);

	// setup LOAD timer
	load_timer.setInterrupts(kInterruptOnSoftwareTrigger | kInterruptPriorityLow);

	// setup EXEC timer
	exec_timer.setInterrupts(kInterruptOnSoftwareTrigger | kInterruptPriorityLowest);

	st_prep.exec_state = PREP_BUFFER_OWNED_BY_EXEC;		// initial condition
}
/*	FOOTNOTE: This is the bare code that the Motate timer calls replace.
	NB: requires: #include <component_tc.h>

	REG_TC1_WPMR = 0x54494D00;			// enable write to registers
	TC_Configure(TC_BLOCK_DDA, TC_CHANNEL_DDA, TC_CMR_DDA);
	REG_RC_DDA = TC_RC_DDA;				// set frequency
	REG_IER_DDA = TC_IER_DDA;			// enable interrupts
	NVIC_EnableIRQ(TC_IRQn_DDA);
	pmc_enable_periph_clk(TC_ID_DDA);
	TC_Start(TC_BLOCK_DDA, TC_CHANNEL_DDA);
*/

static void _clear_diagnostic_counters()
{
	st_run.m[MOTOR_1].step_count_diagnostic = 0;
	st_run.m[MOTOR_2].step_count_diagnostic = 0;
	st_run.m[MOTOR_3].step_count_diagnostic = 0;
	st_run.m[MOTOR_4].step_count_diagnostic = 0;
	st_run.m[MOTOR_5].step_count_diagnostic = 0;
	st_run.m[MOTOR_6].step_count_diagnostic = 0;
}

/*
 * st_assertions() - test assertions, return error code if violation exists
 */
stat_t st_assertions()
{
	if (st_run.magic_start  != MAGICNUM) return (STAT_MEMORY_FAULT);
	if (st_prep.magic_start != MAGICNUM) return (STAT_MEMORY_FAULT);
	return (STAT_OK);
}

/*
 * stepper_isbusy() - return TRUE if motors are running or a dwell is running
 */
uint8_t stepper_isbusy()
{
	if (st_run.dda_ticks_downcount == 0) {
		return (false);
	} 
	return (true);
}

/*
 * Motor power management functions
 *
 * _energize_motor()			- apply power to a motor
 * _deenergize_motor()			- remove power from a motor
 * st_set_motor_power()			- set motor a specified power level
 * st_energize_motors()			- apply power to all motors
 * st_deenergize_motors()		- remove power from all motors
 * st_motor_power_callback()	- callback to manage motor power sequencing
 */

static void _energize_motor(const uint8_t motor)
{
	// Motors that are not defined are not compiled. Saves some ugly #ifdef code
	if (!motor_1.enable.isNull()) if (motor == MOTOR_1) motor_1.enable.clear();	// clear enables the motor
	if (!motor_2.enable.isNull()) if (motor == MOTOR_2) motor_2.enable.clear();
	if (!motor_3.enable.isNull()) if (motor == MOTOR_3) motor_3.enable.clear();
	if (!motor_4.enable.isNull()) if (motor == MOTOR_4) motor_4.enable.clear();
	if (!motor_5.enable.isNull()) if (motor == MOTOR_5) motor_5.enable.clear();
	if (!motor_6.enable.isNull()) if (motor == MOTOR_6) motor_6.enable.clear();

	st_run.m[motor].power_state = MOTOR_START_IDLE_TIMEOUT;
}

static void _deenergize_motor(const uint8_t motor)
{
	// Motors that are not defined are not compiled. Saves some ugly #ifdef code
	if (!motor_1.enable.isNull()) if (motor == MOTOR_1) motor_1.enable.set();	// set disables the motor
	if (!motor_2.enable.isNull()) if (motor == MOTOR_2) motor_2.enable.set();
	if (!motor_3.enable.isNull()) if (motor == MOTOR_3) motor_3.enable.set();
	if (!motor_4.enable.isNull()) if (motor == MOTOR_4) motor_4.enable.set();
	if (!motor_5.enable.isNull()) if (motor == MOTOR_5) motor_5.enable.set();
	if (!motor_6.enable.isNull()) if (motor == MOTOR_6) motor_6.enable.set();

	st_run.m[motor].power_state = MOTOR_OFF;
}

void st_set_motor_power(const uint8_t motor) { }	// code for PWM driven Vref goes here

void st_energize_motors()
{
	motor_1.enable.clear();			// clear enables the motor
	motor_2.enable.clear();			// any motor-N.enable defined as -1 will drop out of compile
	motor_3.enable.clear();
	motor_4.enable.clear();
	motor_5.enable.clear();
	motor_6.enable.clear();
	common_enable.clear();			// enable gShield common enable

	st_run.m[MOTOR_1].power_state = MOTOR_START_IDLE_TIMEOUT;
	st_run.m[MOTOR_2].power_state = MOTOR_START_IDLE_TIMEOUT;
	st_run.m[MOTOR_3].power_state = MOTOR_START_IDLE_TIMEOUT;
	st_run.m[MOTOR_4].power_state = MOTOR_START_IDLE_TIMEOUT;
	st_run.m[MOTOR_5].power_state = MOTOR_START_IDLE_TIMEOUT;
	st_run.m[MOTOR_6].power_state = MOTOR_START_IDLE_TIMEOUT;
}

void st_deenergize_motors()
{
	motor_1.enable.set();			// set disables the motor
	motor_2.enable.set();			// any motor-N.enable defined as -1 will drop out of compile
	motor_3.enable.set();
	motor_4.enable.set();
	motor_5.enable.set();
	motor_6.enable.set();
	common_enable.set();			// disable gShield common enable
}

stat_t st_motor_power_callback() 	// called by controller
{
	// manage power for each motor individually - facilitates advanced features
	for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {

		if (st.m[motor].power_mode == MOTOR_ENERGIZED_DURING_CYCLE) {

			switch (st_run.m[motor].power_state) {
				case (MOTOR_START_IDLE_TIMEOUT): {
//					st_run.m[motor].power_systick = SysTickTimer_getValue() + (uint32_t)(st.motor_idle_timeout * 1000);
					st_run.m[motor].power_systick = SysTickTimer.getValue() + (uint32_t)(st.motor_idle_timeout * 1000);
					st_run.m[motor].power_state = MOTOR_TIME_IDLE_TIMEOUT;
					break;
				}

				case (MOTOR_TIME_IDLE_TIMEOUT): {
//					if (SysTickTimer_getValue() > st_run.m[motor].power_systick ) {
					if (SysTickTimer.getValue() > st_run.m[motor].power_systick ) {
						st_run.m[motor].power_state = MOTOR_IDLE;
						_deenergize_motor(motor);
					}
					break;
				}
			}
			} else if(st.m[motor].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
			switch (st_run.m[motor].power_state) {
				case (MOTOR_START_IDLE_TIMEOUT): {
//					st_run.m[motor].power_systick = SysTickTimer_getValue() + (uint32_t)(IDLE_TIMEOUT_SECONDS * 1000);
					st_run.m[motor].power_systick = SysTickTimer.getValue() + (uint32_t)(IDLE_TIMEOUT_SECONDS * 1000);
					st_run.m[motor].power_state = MOTOR_TIME_IDLE_TIMEOUT;
					break;
				}

				case (MOTOR_TIME_IDLE_TIMEOUT): {
//					if (SysTickTimer_getValue() > st_run.m[motor].power_systick ) {
					if (SysTickTimer.getValue() > st_run.m[motor].power_systick ) {
						st_run.m[motor].power_state = MOTOR_IDLE;
						_deenergize_motor(motor);
					}
					break;
				}
			}

//		} else if(st_run.m[motor].power_mode == MOTOR_POWER_REDUCED_WHEN_IDLE) {	// future
			
//		} else if(st_run.m[motor].power_mode == DYNAMIC_MOTOR_POWER) {				// future
			
		}
	}
	return (STAT_OK);
}

/******************************
 * Interrupt Service Routines *
 ******************************/

/*
 * Dwell timer interrupt
 */
namespace Motate {			// Must define timer interrupts inside the Motate namespace
MOTATE_TIMER_INTERRUPT(dwell_timer_num) 
{
	dwell_timer.getInterruptCause(); // read SR to clear interrupt condition
	if (--st_run.dda_ticks_downcount == 0) {
		dwell_timer.stop();
		_load_move();
	}
}
} // namespace Motate

/****************************************************************************************
 * ISR - DDA timer interrupt routine - service ticks from DDA timer
 *
 *	This interrupt is really 2 interrupts. It fires on timer overflow and also on match.
 *	Overflow interrupts are used to set step pins, match interrupts clear step pins.
 *	This way the duty cycle of the stepper pulse can be controlled by setting the match value.
 *
 *	Note that the motor_N.step.isNull() tests are compile-time tests, not run-time tests. 
 *	If motor_N is not defined that if{} clause (i.e. that motor) drops out of the complied code.
 */
namespace Motate {			// Must define timer interrupts inside the Motate namespace
MOTATE_TIMER_INTERRUPT(dda_timer_num)
{
	uint32_t interrupt_cause = dda_timer.getInterruptCause();	// also clears interrupt condition

	if (interrupt_cause == kInterruptOnOverflow) {
		dda_debug_pin1 = 1;

		if (!motor_1.step.isNull() && (st_run.m[MOTOR_1].phase_accumulator += st_run.m[MOTOR_1].phase_increment) > 0) {
			st_run.m[MOTOR_1].phase_accumulator -= st_run.dda_ticks_X_substeps;
			motor_1.step.set();		// turn step bit on
			INCREMENT_DIAGNOSTIC_COUNTER(MOTOR_1);
		}
		if (!motor_2.step.isNull() && (st_run.m[MOTOR_2].phase_accumulator += st_run.m[MOTOR_2].phase_increment) > 0) {
			st_run.m[MOTOR_2].phase_accumulator -= st_run.dda_ticks_X_substeps;
			motor_2.step.set();
			INCREMENT_DIAGNOSTIC_COUNTER(MOTOR_2);
		}
		if (!motor_3.step.isNull() && (st_run.m[MOTOR_3].phase_accumulator += st_run.m[MOTOR_3].phase_increment) > 0) {
			st_run.m[MOTOR_3].phase_accumulator -= st_run.dda_ticks_X_substeps;
			motor_3.step.set();
			INCREMENT_DIAGNOSTIC_COUNTER(MOTOR_3);
		}
		if (!motor_4.step.isNull() && (st_run.m[MOTOR_4].phase_accumulator += st_run.m[MOTOR_4].phase_increment) > 0) {
			st_run.m[MOTOR_4].phase_accumulator -= st_run.dda_ticks_X_substeps;
			motor_4.step.set();
			INCREMENT_DIAGNOSTIC_COUNTER(MOTOR_4);
		}
		if (!motor_5.step.isNull() && (st_run.m[MOTOR_5].phase_accumulator += st_run.m[MOTOR_5].phase_increment) > 0) {
			st_run.m[MOTOR_5].phase_accumulator -= st_run.dda_ticks_X_substeps;
			motor_5.step.set();
			INCREMENT_DIAGNOSTIC_COUNTER(MOTOR_5);
		}
		if (!motor_6.step.isNull() && (st_run.m[MOTOR_6].phase_accumulator += st_run.m[MOTOR_6].phase_increment) > 0) {
			st_run.m[MOTOR_6].phase_accumulator -= st_run.dda_ticks_X_substeps;
			motor_6.step.set();
			INCREMENT_DIAGNOSTIC_COUNTER(MOTOR_6);
		}
		dda_debug_pin1 = 0;

	} else if (interrupt_cause == kInterruptOnMatchA) { // dda_timer.getInterruptCause() == kInterruptOnMatchA
		dda_debug_pin2 = 1;
		motor_1.step.clear();		// turn step bits off
		motor_2.step.clear();
		motor_3.step.clear();
		motor_4.step.clear();
		motor_5.step.clear();
		motor_6.step.clear();

		if (--st_run.dda_ticks_downcount == 0) {	// process end of move
			dda_timer.stop();						// turn it off or it will keep stepping out the last segment
			_load_move();							// load the next move at the current interrupt level
		}
		dda_debug_pin2 = 0;
	}
}
} // namespace Motate

/****************************************************************************************
 * Exec sequencing code - computes and prepares next load segment
 * st_request_exec_move()	- SW interrupt to request to execute a move
 * exec_timer interrupt		- interrupt handler for calling exec function
 */
void st_request_exec_move()
{
	if (st_prep.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {	// bother interrupting
		exec_timer.setInterruptPending();
	}
}

namespace Motate {	// Define timer inside Motate namespace
MOTATE_TIMER_INTERRUPT(exec_timer_num)			// exec move SW interrupt
{
	exec_timer.getInterruptCause();				// clears the interrupt condition
   	if (st_prep.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {
		if (mp_exec_move() != STAT_NOOP) {
			st_prep.exec_state = PREP_BUFFER_OWNED_BY_LOADER; // flip it back
			_request_load_move();
		}
	}
}

} // namespace Motate

/****************************************************************************************
 * Load sequencing code
 *
 * _request_load()		- fires a software interrupt (timer) to request to load a move
 *  load_mode interrupt	- interrupt handler for running the loader
 * _load_move() 		- load a move into steppers, load a dwell, or process a Null move
 */

static void _request_load_move()
{
	if (st_run.dda_ticks_downcount == 0) {	// bother interrupting
		load_timer.setInterruptPending();
	} 	// ...else don't bother to interrupt. 
		// You'll just trigger an interrupt and find out the loader is not ready
}

namespace Motate {	// Define timer inside Motate namespace
MOTATE_TIMER_INTERRUPT(load_timer_num)		// load steppers SW interrupt
{
	load_timer.getInterruptCause();			// read SR to clear interrupt condition
	_load_move();
}
} // namespace Motate

/*
 * _load_move() - Dequeue move and load into stepper struct
 *
 *	This routine can only be called be called from an ISR at the same or 
 *	higher level as the DDA or dwell ISR. A software interrupt has been 
 *	provided to allow a non-ISR to request a load (see st_request_load_move())
 *
 *	In aline() code:
 *	 - All axes must set steps and compensate for out-of-range pulse phasing.
 *	 - If axis has 0 steps the direction setting can be omitted
 *	 - If axis has 0 steps the motor must not be enabled to support power mode = 1
 */

void _load_move()
{
	// handle aline() loads first (most common case)  NB: there are no more lines, only alines()
	if (st_prep.move_type == MOVE_TYPE_ALINE) {
		st_run.dda_ticks_downcount = st_prep.dda_ticks;
		st_run.dda_ticks_X_substeps = st_prep.dda_ticks_X_substeps;
 
		st_run.m[MOTOR_1].phase_increment = st_prep.m[MOTOR_1].phase_increment;
		if (st_prep.reset_flag == true) {           // compensate for pulse phasing
			st_run.m[MOTOR_1].phase_accumulator = -(st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_1].phase_increment != 0) {	// motor is in this move
			if (st_prep.m[MOTOR_1].dir == 0) {
				motor_1.dir.clear();			// clear the bit for clockwise motion 
			} else {
				motor_1.dir.set();				// set the bit for CCW motion
			}
			motor_1.enable.clear();				// enable the motor (clear the ~Enable line)
			st_run.m[MOTOR_1].power_state = MOTOR_RUNNING;
		} else {								// motor is not in this move
			if (st.m[MOTOR_1].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
				motor_1.enable.clear();			// energize motor
				st_run.m[MOTOR_1].power_state = MOTOR_START_IDLE_TIMEOUT;
			}			
		}

		st_run.m[MOTOR_2].phase_increment = st_prep.m[MOTOR_2].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_2].phase_accumulator = -(st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_2].phase_increment != 0) {
			if (st_prep.m[MOTOR_2].dir == 0) motor_2.dir.clear(); else motor_2.dir.set();
			motor_2.enable.clear();
			st_run.m[MOTOR_2].power_state = MOTOR_RUNNING;
		} else {
			if (st.m[MOTOR_2].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
				motor_2.enable.clear();
				st_run.m[MOTOR_2].power_state = MOTOR_START_IDLE_TIMEOUT;
			}
		}

		st_run.m[MOTOR_3].phase_increment = st_prep.m[MOTOR_3].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_3].phase_accumulator = -(st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_3].phase_increment != 0) {
			if (st_prep.m[MOTOR_3].dir == 0) motor_3.dir.clear(); else motor_3.dir.set();
			motor_3.enable.clear();
			st_run.m[MOTOR_3].power_state = MOTOR_RUNNING;
		} else {
			if (st.m[MOTOR_3].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
				motor_3.enable.clear();
				st_run.m[MOTOR_3].power_state = MOTOR_START_IDLE_TIMEOUT;
			}
		}

		st_run.m[MOTOR_4].phase_increment = st_prep.m[MOTOR_4].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_4].phase_accumulator = (st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_4].phase_increment != 0) {
			if (st_prep.m[MOTOR_4].dir == 0) motor_4.dir.clear(); else motor_4.dir.set();
			motor_4.enable.clear();
			st_run.m[MOTOR_4].power_state = MOTOR_RUNNING;
		} else {
			if (st.m[MOTOR_4].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
				motor_4.enable.clear();
				st_run.m[MOTOR_4].power_state = MOTOR_START_IDLE_TIMEOUT;
			}
		}

		st_run.m[MOTOR_5].phase_increment = st_prep.m[MOTOR_5].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_5].phase_accumulator = (st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_5].phase_increment != 0) {
			if (st_prep.m[MOTOR_5].dir == 0) motor_5.dir.clear(); else motor_5.dir.set();
			motor_5.enable.clear();
			st_run.m[MOTOR_5].power_state = MOTOR_RUNNING;
		} else {
			if (st.m[MOTOR_5].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
				motor_5.enable.clear();
				st_run.m[MOTOR_5].power_state = MOTOR_START_IDLE_TIMEOUT;
			}
		}

		st_run.m[MOTOR_6].phase_increment = st_prep.m[MOTOR_6].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_6].phase_accumulator = (st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_6].phase_increment != 0) {
			if (st_prep.m[MOTOR_6].dir == 0) motor_6.dir.clear(); else motor_6.dir.set();
			motor_6.enable.clear();
			st_run.m[MOTOR_6].power_state = MOTOR_RUNNING;
		} else {
			if (st.m[MOTOR_6].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
				motor_6.enable.clear();
				st_run.m[MOTOR_6].power_state = MOTOR_START_IDLE_TIMEOUT;
			}
		}
		dda_timer.start();		// start the DDA timer if not already running

	// handle dwells
	} else if (st_prep.move_type == MOVE_TYPE_DWELL) {
		st_run.dda_ticks_downcount = st_prep.dda_ticks;
		dwell_timer.start();
	}

	// all cases drop to here - such as Null moves queued by MCodes
	st_prep_null();										// needed to shut off timers if no moves left
	st_prep.exec_state = PREP_BUFFER_OWNED_BY_EXEC;		// flip it back
	st_request_exec_move();								// compute and prepare the next move
}

/* 
 * st_prep_null() - Keeps the loader happy. Otherwise performs no action
 *
 *	Used by M codes, tool and spindle changes
 */
void st_prep_null()
{
	st_prep.move_type = MOVE_TYPE_NULL;
}

/* 
 * st_prep_dwell() 	 - Add a dwell to the move buffer
 */

void st_prep_dwell(float microseconds)
{
	st_prep.move_type = MOVE_TYPE_DWELL;
	st_prep.dda_ticks = (uint32_t)((microseconds/1000000) * FREQUENCY_DWELL); // ARM code
//	st_prep.dda_period = _f_to_period(F_DWELL);	// AVR code
}

/***********************************************************************************
 * st_prep_line() - Prepare the next move for the loader
 *
 *	This function does the math on the next pulse segment and gets it ready for 
 *	the loader. It deals with all the DDA optimizations and timer setups so that
 *	loading can be performed as rapidly as possible. It works in joint space 
 *	(motors) and it works in steps, not length units. All args are provided as 
 *	floats and converted to their appropriate integer types for the loader. 
 *
 * Args:
 *	steps[] are signed relative motion in steps (can be non-integer values)
 *	Microseconds - how many microseconds the segment should run 
 */

stat_t st_prep_line(float steps[], float microseconds)
{
	// *** defensive programming ***
	// trap conditions that would prevent queuing the line
	if (st_prep.exec_state != PREP_BUFFER_OWNED_BY_EXEC) { return (STAT_INTERNAL_ERROR);
	} else if (isfinite(microseconds) == false) { return (STAT_INPUT_EXCEEDS_MAX_LENGTH);
	} else if (microseconds < EPSILON) { return (STAT_MINIMUM_TIME_MOVE_ERROR);
	}
	st_prep.reset_flag = false;         // initialize accumulator reset flag for this move.

	// setup motor parameters
	for (uint8_t i=0; i<MOTORS; i++) {
		st_prep.m[i].dir = ((steps[i] < 0) ? 1 : 0) ^ st.m[i].polarity;
		st_prep.m[i].phase_increment = (uint32_t)fabs(steps[i] * DDA_SUBSTEPS);
	}
	st_prep.dda_ticks = (uint32_t)((microseconds/1000000) * FREQUENCY_DDA);
	st_prep.dda_ticks_X_substeps = st_prep.dda_ticks * DDA_SUBSTEPS;

	// FOOTNOTE: The above expression was previously computed as below but floating
	// point rounding errors caused subtle and nasty accumulated position errors:
	// sp.dda_ticks_X_substeps = (uint32_t)((microseconds/1000000) * f_dda * dda_substeps);

	// anti-stall measure in case change in velocity between segments is too great 
	if ((st_prep.dda_ticks * ACCUMULATOR_RESET_FACTOR) < st_prep.prev_ticks) {  // NB: uint32_t math
		st_prep.reset_flag = true;
	}
	st_prep.prev_ticks = st_prep.dda_ticks;
	st_prep.move_type = MOVE_TYPE_ALINE;
	return (STAT_OK);
}

/*
 * _set_hw_microsteps() - set microsteps in hardware
 *
 *	For now the microstep_mode is the same as the microsteps (1,2,4,8)
 *	This may change if microstep morphing is implemented.
 */

static void _set_hw_microsteps(const uint8_t motor, const uint8_t microstep_mode)
{
/*
	if (microstep_mode == 8) {
		hw.st_port[motor]->OUTSET = MICROSTEP_BIT_0_bm;
		hw.st_port[motor]->OUTSET = MICROSTEP_BIT_1_bm;
	} else if (microstep_mode == 4) {
		hw.st_port[motor]->OUTCLR = MICROSTEP_BIT_0_bm;
		hw.st_port[motor]->OUTSET = MICROSTEP_BIT_1_bm;
	} else if (microstep_mode == 2) {
		hw.st_port[motor]->OUTSET = MICROSTEP_BIT_0_bm;
		hw.st_port[motor]->OUTCLR = MICROSTEP_BIT_1_bm;
	} else if (microstep_mode == 1) {
		hw.st_port[motor]->OUTCLR = MICROSTEP_BIT_0_bm;
		hw.st_port[motor]->OUTCLR = MICROSTEP_BIT_1_bm;
	}
*/
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * _get_motor() - helper to return motor number as an index or -1 if na
 */

static int8_t _get_motor(const index_t index)
{
	char_t *ptr;
	char_t motors[] = {"1234"};
	char_t tmp[CMD_TOKEN_LEN+1];

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
	st.m[m].steps_per_unit = (360 / (st.m[m].step_angle / st.m[m].microsteps) / st.m[m].travel_rev);
}

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
	set_ui8(cmd);							// set it anyway, even if it's unsupported
	_set_motor_steps_per_unit(cmd);
	_set_hw_microsteps(_get_motor(cmd->index), (uint8_t)cmd->value);
	return (STAT_OK);
}

stat_t st_set_pm(cmdObj_t *cmd)			// motor power mode
{ 
	ritorno (set_01(cmd));
	if (fp_ZERO(cmd->value)) { // people asked this setting take effect immediately, hence:
		_energize_motor(_get_motor(cmd->index));
	} else {
		_deenergize_motor(_get_motor(cmd->index));
	}
	return (STAT_OK);
}

stat_t st_set_mt(cmdObj_t *cmd)
{
	st.motor_idle_timeout = min(IDLE_TIMEOUT_SECONDS_MAX, max(cmd->value, IDLE_TIMEOUT_SECONDS_MIN));
	return (STAT_OK);
}

stat_t st_set_md(cmdObj_t *cmd)	// Make sure this function is not part of initialization --> f00
{
	st_deenergize_motors();
	return (STAT_OK);
}

stat_t st_set_me(cmdObj_t *cmd)	// Make sure this function is not part of initialization --> f00
{
	st_energize_motors();
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

static const char fmt_mt[] PROGMEM = "[mt]  motor idle timeout%14.2f Sec\n";
static const char fmt_me[] PROGMEM = "motors energized\n";
static const char fmt_md[] PROGMEM = "motors de-energized\n";
static const char fmt_0ma[] PROGMEM = "[%s%s] m%s map to axis%15d [0=X,1=Y,2=Z...]\n";
static const char fmt_0sa[] PROGMEM = "[%s%s] m%s step angle%20.3f%s\n";
static const char fmt_0tr[] PROGMEM = "[%s%s] m%s travel per revolution%9.3f%s\n";
static const char fmt_0mi[] PROGMEM = "[%s%s] m%s microsteps%16d [1,2,4,8]\n";
static const char fmt_0po[] PROGMEM = "[%s%s] m%s polarity%18d [0=normal,1=reverse]\n";
static const char fmt_0pm[] PROGMEM = "[%s%s] m%s power management%10d [0=remain powered,1=power down when idle]\n";

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

void st_print_ma(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0ma);}
void st_print_sa(cmdObj_t *cmd) { _print_motor_flt_units(cmd, fmt_0sa, DEGREE_INDEX);}
void st_print_tr(cmdObj_t *cmd) { _print_motor_flt_units(cmd, fmt_0tr, cm_get_units_mode(MODEL));}
void st_print_mi(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0mi);}
void st_print_po(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0po);}
void st_print_pm(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0pm);}

#endif // __TEXT_MODE
