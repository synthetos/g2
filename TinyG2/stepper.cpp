/*
 * stepper.cpp - stepper motor controls
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
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
/* 
 * See stepper.h for a detailed explanation of this part of the code 
 */

#include "tinyg2.h"
#include "config.h"
#include "stepper.h"
#include "planner.h"
//#include "motatePins.h"		// defined in hardware.h   Not needed here
#include "motateTimers.h"
#include "hardware.h"
#include "util.h"

//#define ENABLE_DIAGNOSTICS
#ifdef ENABLE_DIAGNOSTICS
#define INCREMENT_DIAGNOSTIC_COUNTER(motor) st_run.m[motor].step_count_diagnostic++;
#else
#define INCREMENT_DIAGNOSTIC_COUNTER(motor)	// chose this one to disable counters
#endif

// Setup local resources

static void _load_move(void);
static void _request_load_move(void);
//static void _clear_diagnostic_counters(void);

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

/*
 * Stepper structures
 *
 *	There are 4 sets of structures involved in loading and running step pulses:
 *
 *	data structure:					 static to:		runs at:
 *	  mpBuffer planning buffers (bf)  planner.cpp	  main loop
 *	  mrRuntimeSingleton (mr)		  planner.cpp	  medium interrupt priority
 *	  stPrepSingleton (sps)			  stepper.cpp	  medium interrupt priority
 *	  stRunSingleton (st)			  stepper.cpp	  highest interrupt priority 
 * 
 *	Care has been taken to isolate actions on these structures to the execution 
 *	level in which they run and to use the minimum number of volatiles in these 
 *	structures. This allows the compiler to optimize the stepper inner-loops better.
 */

// Runtime structure. Used exclusively by step generation ISR (HI)

typedef struct stRunMotor { 			// one per controlled motor
	int32_t phase_increment;			// = total steps in axis times substep factor
	int32_t phase_accumulator;			// DDA phase angle accumulator for axis
	uint8_t polarity;					// 0=normal polarity, 1=reverse motor polarity
//	uint32_t step_count_diagnostic;		// diagnostic - comment out for normal use
} stRunMotor_t;

typedef struct stRunSingleton {			// Stepper static values and axis parameters
	uint16_t magic_start;				// magic number to test memory integrity	
	int32_t dda_ticks_downcount;		// tick down-counter (unscaled)
	int32_t dda_ticks_X_substeps;		// ticks multiplied by scaling factor
	uint32_t motor_idle_systick;		// sys_tick at which to idle the motors
	volatile uint8_t motor_stop_flags;	// bitfield for motor stop conditions
	stRunMotor_t m[MOTORS];				// runtime motor structures
} stRunSingleton_t;

// Prep-time structure. Used by exec/prep ISR (MED) and read-only during load 
// Must be careful about volatiles in this one

typedef struct stPrepMotor {
 	uint32_t phase_increment; 			// = total steps in axis times substep factor
	int8_t dir;							// direction
} stPrepMotor_t;

typedef struct stPrepSingleton {
	uint16_t magic_start;				// magic number to test memory integrity	
	uint8_t move_type;					// move type
	volatile uint8_t exec_state;		// move execution state 
	volatile uint8_t reset_flag;		// set TRUE if accumulator should be reset
	uint32_t prev_ticks;				// tick count from previous move
	uint16_t dda_period;				// DDA or dwell clock period setting (unused for Motate)
	uint32_t dda_ticks;					// DDA (or dwell) ticks for the move
	uint32_t dda_ticks_X_substeps;		// DDA ticks scaled by sub-step factor
	stPrepMotor_t m[MOTORS];			// per-motor structures
} stPrepSingleton_t;

// Allocate static structures
static stRunSingleton_t st_run;
static struct stPrepSingleton st_prep;

magic_t st_get_st_magic() { return (st_run.magic_start);}
magic_t st_get_sps_magic() { return (st_prep.magic_start);}

/*
 * stepper_init() - initialize stepper motor subsystem 
 *
 *	Notes:
 *	  - This init requires sys_init() to be run beforehand
 * 	  - microsteps are setup during cfg_init()
 *	  - motor polarity is setup during cfg_init()
 *	  - high level interrupts must be enabled in main() once all inits are complete
 */

void stepper_init()
{
	memset(&st_run, 0, sizeof(st_run));		// clear all values, pointers and status
	st_run.magic_start = MAGICNUM;
	st_prep.magic_start = MAGICNUM;
//	_clear_diagnostic_counters();

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

/*
static void _clear_diagnostic_counters()
{
	st_run.m[MOTOR_1].step_count_diagnostic = 0;
	st_run.m[MOTOR_2].step_count_diagnostic = 0;
	st_run.m[MOTOR_3].step_count_diagnostic = 0;
	st_run.m[MOTOR_4].step_count_diagnostic = 0;
	st_run.m[MOTOR_5].step_count_diagnostic = 0;
	st_run.m[MOTOR_6].step_count_diagnostic = 0;
}
*/

/* 
 * st_set_motor_idle_timeout() - set the timeout in the config
 */

void st_set_motor_idle_timeout(float seconds)
{
	cfg.motor_idle_timeout = min(IDLE_TIMEOUT_SECONDS_MAX, max(seconds, IDLE_TIMEOUT_SECONDS_MIN));
}

/* 
 * st_do_motor_idle_timeout()  - execute the timeout
 *
 *	Sets a point N seconds in the future when the motors will be idled (time out)
 *	Can be called at any time to extend N seconds from the current time
 */

void st_do_motor_idle_timeout()
{
	st_run.motor_idle_systick = SysTickTimer.getValue() + (uint32_t)(cfg.motor_idle_timeout * 1000);
}

/* 
 * st_energize_motor()	 - enable a motor
 * st_deenergize_motor() - disable a motor
 * st_set_motor_power()	 - set motor power level
 */

void st_energize_motor(const uint8_t motor)
{
	// Motors that are not defined are not compiled. Saves some ugly #ifdef code
	if (!motor_1.enable.isNull()) if (motor == MOTOR_1) motor_1.enable.clear();	// clear enables the motor
	if (!motor_2.enable.isNull()) if (motor == MOTOR_2) motor_2.enable.clear();
	if (!motor_3.enable.isNull()) if (motor == MOTOR_3) motor_3.enable.clear();
	if (!motor_4.enable.isNull()) if (motor == MOTOR_4) motor_4.enable.clear();
	if (!motor_5.enable.isNull()) if (motor == MOTOR_5) motor_5.enable.clear();
	if (!motor_6.enable.isNull()) if (motor == MOTOR_6) motor_6.enable.clear();
}

void st_deenergize_motor(const uint8_t motor)
{
	// Motors that are not defined are not compiled. Saves some ugly #ifdef code
	if (!motor_1.enable.isNull()) if (motor == MOTOR_1) motor_1.enable.set();	// set disables the motor
	if (!motor_2.enable.isNull()) if (motor == MOTOR_2) motor_2.enable.set();
	if (!motor_3.enable.isNull()) if (motor == MOTOR_3) motor_3.enable.set();
	if (!motor_4.enable.isNull()) if (motor == MOTOR_4) motor_4.enable.set();
	if (!motor_5.enable.isNull()) if (motor == MOTOR_5) motor_5.enable.set();
	if (!motor_6.enable.isNull()) if (motor == MOTOR_6) motor_6.enable.set();
}

void st_set_motor_power(const uint8_t motor)
{
	// code for PWM driven Vref goes here
}

/* 
 * st_energize_motors()   - apply power to all motors
 * st_deenergize_motors() - remove power from all motors
 * st_idle_motors()		  - set all motors to idle power level
 */

void st_energize_motors()
{
	motor_1.enable.clear();			// clear enables the motor
	motor_2.enable.clear();			// any motor-N.enable defined as -1 will drop out of compile
	motor_3.enable.clear();
	motor_4.enable.clear();
	motor_5.enable.clear();
	motor_6.enable.clear();
	common_enable.clear();			// enable gShield common enable

	st_do_motor_idle_timeout();
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

void st_idle_motors()
{
	st_deenergize_motors();			// for now idle is the same as de-energized
}

/*
 * st_motor_power_callback() - callback to manage motor power sequencing
 */

stat_t st_motor_power_callback() 	// called by controller
{
	if (st_run.motor_stop_flags != 0) {
		st_run.motor_stop_flags = 0;
		st_do_motor_idle_timeout();
	}
	if (SysTickTimer.getValue() < st_run.motor_idle_systick ) return (STAT_NOOP);

	common_enable.set();			// disable gShield common enable
	st_idle_motors();
	return (STAT_OK);
}

// Define the timer interrupts inside the Motate namespace
namespace Motate {

/*
 * Dwell timer interrupt
 */
MOTATE_TIMER_INTERRUPT(dwell_timer_num) 
{
	dwell_timer.getInterruptCause(); // read SR to clear interrupt condition
	if (--st_run.dda_ticks_downcount == 0) {
		dwell_timer.stop();
		_load_move();
	}
}

/****************************************************************************************
 * ISR - DDA timer interrupt routine - service ticks from DDA timer
 *
 *	This interrupt is really 2 interrupts. It fires on timer overflow and also on match.
 *	Overflow interrupts are used to set step pins, match interrupts clear step pins.
 *	This way the duty cycle of the stepper pulse can be controlled by setting the match value.
 *
 *	Also note that the motor_N.step.isNull() tests are compile-time tests, not run-time tests.
 *	If motor_N is not defined that entire if {} clause drops out of the complied code.
 */
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
			st_run.motor_stop_flags = ALL_MOTORS_STOPPED;
			_load_move();							// load the next move at the current interrupt level
		}
		dda_debug_pin2 = 0;
	}
}

} // namespace Motate

/****************************************************************************************
 * Exec sequencing code - computes and prepares next load segment
 *
 * st_request_exec_move()	- SW interrupt to request to execute a move
 * exec_timer interrupt		- interrupt handler for calling exec function
 */
void st_request_exec_move()
{
	if (st_prep.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {	// bother interrupting
		exec_timer.setInterruptPending();
	}
}

// Define the timers inside the Motate namespace
namespace Motate {

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

// Define the timers inside the Motate namespace
namespace Motate {

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
		if (st_run.m[MOTOR_1].phase_increment != 0) {
			if (st_prep.m[MOTOR_1].dir == 0) {
				motor_1.dir.clear();			// clear the bit for clockwise motion 
			} else {
				motor_1.dir.set();				// set the bit for CCW motion
			}
			motor_1.enable.clear();				// enable the motor (clear the ~Enable line)
		}

		st_run.m[MOTOR_2].phase_increment = st_prep.m[MOTOR_2].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_2].phase_accumulator = -(st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_2].phase_increment != 0) {
			if (st_prep.m[MOTOR_2].dir == 0) motor_2.dir.clear(); 
			else motor_2.dir.set();
			motor_2.enable.clear();
		}

		st_run.m[MOTOR_3].phase_increment = st_prep.m[MOTOR_3].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_3].phase_accumulator = -(st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_3].phase_increment != 0) {
			if (st_prep.m[MOTOR_3].dir == 0) motor_3.dir.clear();
			else motor_3.dir.set();
			motor_3.enable.clear();
		}

		st_run.m[MOTOR_4].phase_increment = st_prep.m[MOTOR_4].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_4].phase_accumulator = (st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_4].phase_increment != 0) {
			if (st_prep.m[MOTOR_4].dir == 0) motor_4.dir.clear();
			else motor_4.dir.set();
			motor_4.enable.clear();
		}

		st_run.m[MOTOR_5].phase_increment = st_prep.m[MOTOR_5].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_5].phase_accumulator = (st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_5].phase_increment != 0) {
			if (st_prep.m[MOTOR_5].dir == 0) motor_5.dir.clear();
			else motor_5.dir.set();
			motor_5.enable.clear();
		}

		st_run.m[MOTOR_6].phase_increment = st_prep.m[MOTOR_6].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_6].phase_accumulator = (st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_6].phase_increment != 0) {
			if (st_prep.m[MOTOR_6].dir == 0) motor_6.dir.clear();
			else motor_6.dir.set();
			motor_6.enable.clear();
		}
		st_energize_motors();	// apply power to the motors
		dda_timer.start();		// start the DDA timer if not already running

	// handle dwells
	} else if (st_prep.move_type == MOVE_TYPE_DWELL) {
		st_run.dda_ticks_downcount = st_prep.dda_ticks;
		dwell_timer.start();
	}

	// all cases drop to here - such as Null moves queued by MCodes
	st_prep_null();										// disable prep buffer, if only temporarily
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
	st_prep.dda_ticks = (uint32_t)((microseconds/1000000) * FREQUENCY_DWELL);
}

/***********************************************************************************
 * st_prep_line() - Prepare the next move for the loader
 *
 *	This function does the math on the next pulse segment and gets it ready for the 
 *	loader. It deals with all the DDA optimizations and timer setups so that loading 
 *	can be performed as rapidly as possible. It works in joint space (motors) and it 
 *	works in steps, not length units. All args are provided as floats and converted 
 *	to their appropriate integer types for the loader. 
 *
 * Args:
 *	steps[] are signed relative motion in steps (can be non-integer values)
 *	Microseconds - how many microseconds the segment should run 
 */

uint8_t st_prep_line(float steps[], float microseconds)
{
	// *** defensive programming ***
	// trap conditions that would prevent queuing the line
	if (st_prep.exec_state != PREP_BUFFER_OWNED_BY_EXEC) { return (STAT_INTERNAL_ERROR);
	} else if (isfinite(microseconds) == false) { return (STAT_MINIMUM_TIME_MOVE_ERROR);
	} else if (microseconds < EPSILON) { return (STAT_MINIMUM_TIME_MOVE_ERROR);
	}
	st_prep.reset_flag = false;         // initialize accumulator reset flag for this move.

	// setup motor parameters
	for (uint8_t i=0; i<MOTORS; i++) {
		st_prep.m[i].dir = ((steps[i] < 0) ? 1 : 0) ^ cfg.m[i].polarity;
		st_prep.m[i].phase_increment = (uint32_t)fabs(steps[i] * DDA_SUBSTEPS);
	}
	st_prep.dda_ticks = (uint32_t)((microseconds/1000000) * FREQUENCY_DDA);
	st_prep.dda_ticks_X_substeps = st_prep.dda_ticks * DDA_SUBSTEPS;	// see FOOTNOTE

	// anti-stall measure in case change in velocity between segments is too great 
	if ((st_prep.dda_ticks * ACCUMULATOR_RESET_FACTOR) < st_prep.prev_ticks) {  // NB: uint32_t math
		st_prep.reset_flag = true;
	}
	st_prep.prev_ticks = st_prep.dda_ticks;
	st_prep.move_type = MOVE_TYPE_ALINE;
	return (STAT_OK);
}
// FOOTNOTE: This expression was previously computed as below but floating 
// point rounding errors caused subtle and nasty accumulated position errors:
// sp.dda_ticks_X_substeps = (uint32_t)((microseconds/1000000) * f_dda * dda_substeps);

/****************************************************************************************
 * UTILITIES
 * st_isbusy()			- return TRUE if motors are running or a dwell is running
 * st_set_microsteps()	- set microsteps in hardware
 *
 *	For now the microstep_mode is the same as the microsteps (1,2,4,8)
 *	This may change if microstep morphing is implemented.
 */
uint8_t st_isbusy()
{
	if (st_run.dda_ticks_downcount == 0) {
		return (false);
	} 
	return (true);
}

void st_set_microsteps(const uint8_t motor, const uint8_t microstep_mode)
{
/*
	if (microstep_mode == 8) {
		device.st_port[motor]->OUTSET = MICROSTEP_BIT_0_bm;
		device.st_port[motor]->OUTSET = MICROSTEP_BIT_1_bm;
	} else if (microstep_mode == 4) {
		device.st_port[motor]->OUTCLR = MICROSTEP_BIT_0_bm;
		device.st_port[motor]->OUTSET = MICROSTEP_BIT_1_bm;
	} else if (microstep_mode == 2) {
		device.st_port[motor]->OUTSET = MICROSTEP_BIT_0_bm;
		device.st_port[motor]->OUTCLR = MICROSTEP_BIT_1_bm;
	} else if (microstep_mode == 1) {
		device.st_port[motor]->OUTCLR = MICROSTEP_BIT_0_bm;
		device.st_port[motor]->OUTCLR = MICROSTEP_BIT_1_bm;
	}
*/
}

// END NOTES
/*

This test code can be run from the init or the load

#ifdef TEST_CODE
    st_prep.move_type = true;

	st_prep.dda_ticks = 100000;
	st_prep.dda_ticks_X_substeps = 1000000;
	st_run.m[MOTOR_1].phase_increment = 90000;
	st_run.m[MOTOR_1].phase_accumulator = -st_prep.dda_ticks;
	st_run.dda_ticks_X_substeps = st_prep.dda_ticks_X_substeps;
	st_enable(); 
//    dda_timer.start();
    return;
#endif
*/
