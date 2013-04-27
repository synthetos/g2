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
/* 	This module provides the low-level stepper drivers and some related
 * 	functions. It dequeues lines queued by the motor_queue routines.
 * 	This is some of the most heavily optimized code in the project.
 *	Please refer to the end of the stepper.h file for a more complete explanation
 */
#include "tinyg2.h"
#include "config.h"
#include "hardware.h"
#include "planner.h"
#include "stepper.h"
//#include "motatePins.h"		// defined in hardware.h   Not needed here
#include "motateTimers.h"
#include "util.h"

/*
#ifdef __cplusplus
extern "C"{
#endif
*/
//#include <component_tc.h>		// deprecated - to be removed

using namespace Motate;

// Setup local resources
Motate::Timer<DDA_TIMER_NUM> dda_timer;			// stepper pulse generation
Motate::Timer<DWELL_TIMER_NUM> dwell_timer;		// dwell timer
Motate::Timer<LOAD_TIMER_NUM> load_timer;		// triggers load of next stepper segment
Motate::Timer<EXEC_TIMER_NUM> exec_timer;		// triggers calculation of next+1 stepper segment
Motate::Pin<31> proof_of_timer(kOutput);

// Setup a stepper template to hold our pins
template<pin_number step_num, pin_number dir_num, pin_number enable_num, 
		 pin_number ms0_num, pin_number ms1_num, pin_number vref_num>

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

OutputPin<motor_enable_pin_num> enable;

volatile long dummy;			// convenient register to read into

static void _load_move(void);
static void _exec_move(void);
static void _request_load_move(void);

enum prepBufferState {
	PREP_BUFFER_OWNED_BY_LOADER = 0,// staging buffer is ready for load
	PREP_BUFFER_OWNED_BY_EXEC		// staging buffer is being loaded
};

/*
 * Stepper structures
 *
 *	There are 4 sets of structures involved in this operation;
 *
 *	data structure:						static to:		runs at:
 *	  mpBuffer planning buffers (bf)	  planner.c		  main loop
 *	  mrRuntimeSingleton (mr)			  planner.c		  MED ISR
 *	  stPrepSingleton (sp)				  stepper.c		  MED ISR
 *	  stRunSingleton (st)				  stepper.c		  HI ISR
 *  
 *	Care has been taken to isolate actions on these structures to the 
 *	execution level in which they run and to use the minimum number of 
 *	volatiles in these structures. This allows the compiler to optimize
 *	the stepper inner-loops better.
 */

// Runtime structure. Used exclusively by step generation ISR (HI)
typedef struct stRunMotor { 		// one per controlled motor
	int32_t steps;					// total steps in axis
	int32_t counter;				// DDA counter for axis
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor polarity
} stRunMotor_t;

typedef struct stRunSingleton {		// Stepper static values and axis parameters
	uint16_t magic_start;			// magic number to test memory integity	
	int32_t timer_ticks_downcount;	// tick down-counter (unscaled)
	int32_t timer_ticks_X_substeps;	// ticks multiplied by scaling factor
	stRunMotor_t m[MOTORS];			// runtime motor structures
} stRunSingleton_t;

// Prep-time structure. Used by exec/prep ISR (MED) and read-only during load 
// Must be careful about volatiles in this one

typedef struct stPrepMotor {
 	uint32_t steps; 				// total steps in each direction
	int8_t dir;						// b0 = direction
} stPrepMotor_t;

typedef struct stPrepSingleton {
	uint16_t magic_start;			// magic number to test memory integity	
	uint8_t move_type;				// move type
	volatile uint8_t exec_state;	// move execution state 
	volatile uint8_t counter_reset_flag; // set TRUE if counter should be reset
	uint32_t prev_ticks;			// tick count from previous move
	uint16_t timer_period;			// DDA or dwell clock period setting
	uint32_t timer_ticks;			// DDA or dwell ticks for the move
	uint32_t timer_ticks_X_substeps;// DDA ticks scaled by substep factor
	float segment_velocity;		// +++++ record segment velocity for diagnostics
	stPrepMotor_t m[MOTORS];		// per-motor structs
} stPrepSingleton_t;

// Structure allocation
static stRunSingleton_t st;
static struct stPrepSingleton sps;

magic_t st_get_st_magic() { return (st.magic_start);}
magic_t st_get_sps_magic() { return (sps.magic_start);}

/*
 * stepper_init() - initialize stepper motor subsystem 
 *
 *	Notes:
 *	  - This init requires sys_init() to be run beforehand
 *		This init is a precursor for gpio_init()
 * 	  - microsteps are setup during cfg_init()
 *	  - motor polarity is setup during cfg_init()
 *	  - high level interrupts must be enabled in main() once all inits are complete
 */

void stepper_init()
{
	memset(&st, 0, sizeof(st));			// clear all values, pointers and status
	st.magic_start = MAGICNUM;
	sps.magic_start = MAGICNUM;

	// ***** Setup pins *****
	motor_1.step.setMode(kOutput);
	motor_1.dir.setMode(kOutput);
	motor_1.enable.setMode(kOutput);
	motor_1.ms0.setMode(kOutput);
	motor_1.ms1.setMode(kOutput);
	motor_1.vref.setMode(kOutput);

	motor_2.step.setMode(kOutput);
	motor_2.dir.setMode(kOutput);
	motor_2.enable.setMode(kOutput);
	motor_2.ms0.setMode(kOutput);
	motor_2.ms1.setMode(kOutput);
	motor_2.vref.setMode(kOutput);

	motor_3.step.setMode(kOutput);
	motor_3.dir.setMode(kOutput);
	motor_3.enable.setMode(kOutput);
	motor_3.ms0.setMode(kOutput);
	motor_3.ms1.setMode(kOutput);
	motor_3.vref.setMode(kOutput);

	motor_4.step.setMode(kOutput);
	motor_4.dir.setMode(kOutput);
	motor_4.enable.setMode(kOutput);
	motor_4.ms0.setMode(kOutput);
	motor_4.ms1.setMode(kOutput);
	motor_4.vref.setMode(kOutput);

	motor_5.step.setMode(kOutput);
	motor_5.dir.setMode(kOutput);
	motor_5.enable.setMode(kOutput);
	motor_5.ms0.setMode(kOutput);
	motor_5.ms1.setMode(kOutput);
	motor_5.vref.setMode(kOutput);

	motor_6.step.setMode(kOutput);
	motor_6.dir.setMode(kOutput);
	motor_6.enable.setMode(kOutput);
	motor_6.ms0.setMode(kOutput);
	motor_6.ms1.setMode(kOutput);
	motor_6.vref.setMode(kOutput);

	// ***** Setup timers *****
	// setup DDA timer
#ifdef BARE_CODE
	REG_TC1_WPMR = 0x54494D00;			// enable write to registers
	TC_Configure(TC_BLOCK_DDA, TC_CHANNEL_DDA, TC_CMR_DDA);
	REG_RC_DDA = TC_RC_DDA;				// set frequency
	REG_IER_DDA = TC_IER_DDA;			// enable interrupts
	NVIC_EnableIRQ(TC_IRQn_DDA);
	pmc_enable_periph_clk(TC_ID_DDA);
	TC_Start(TC_BLOCK_DDA, TC_CHANNEL_DDA);
#else
    dda_timer.setModeAndFrequency(kTimerUpToMatch, FREQUENCY_DDA);
    dda_timer.setInterrupts(kInterruptOnOverflow);
 //   dda_timer.start();
#endif

	// setup DWELL timer
	dwell_timer.setModeAndFrequency(kTimerUpToMatch, FREQUENCY_DWELL);
	dwell_timer.setInterrupts(kInterruptOnOverflow);
//	dwell_timer.start();

	// setup LOAD timer
	load_timer.setModeAndFrequency(kTimerUpToMatch, FREQUENCY_SGI);
	load_timer.setInterrupts(kInterruptOnOverflow);

	// setup EXEC timer
	exec_timer.setModeAndFrequency(kTimerUpToMatch, FREQUENCY_SGI);
	exec_timer.setInterrupts(kInterruptOnOverflow);

	sps.exec_state = PREP_BUFFER_OWNED_BY_EXEC;

//    _load_move();
}

/*
 * st_disable() - stop the steppers. (Requires re-init to recover -- Is this still true?)
 */
void st_disable()
{
	dda_timer.stop();
}

/*
 * Dwell timer interrupt
 */
MOTATE_TIMER_INTERRUPT(DWELL_TIMER_NUM) 
{
	dummy = DWELL_STATUS_REGISTER;				// read SR to clear interrupt condition
	if (--st.timer_ticks_downcount == 0) {
		dwell_timer.stop();
		_load_move();
	}
}

/****************************************************************************************
 * ISR - DDA timer interrupt routine - service ticks from DDA timer
 *
 *	Uses direct struct addresses and literal values for hardware devices -
 *	it's faster than using indexed timer and port accesses. I checked.
 *	Even when -0s or -03 is used.
 */
MOTATE_TIMER_INTERRUPT(DDA_TIMER_NUM)
{
    dummy = DDA_STATUS_REGISTER;	// read SR to clear interrupt condition
    proof_of_timer = 0;
    
    if (!motor_1.step.isNull() && (st.m[MOTOR_1].counter += st.m[MOTOR_1].steps) > 0) {
        st.m[MOTOR_1].counter -= st.timer_ticks_X_substeps;
        motor_1.step.set();		// turn step bit on
    }
    if (!motor_2.step.isNull() && (st.m[MOTOR_2].counter += st.m[MOTOR_2].steps) > 0) {
        st.m[MOTOR_2].counter -= st.timer_ticks_X_substeps;
        motor_2.step.set();
    }
    if (!motor_3.step.isNull() && (st.m[MOTOR_3].counter += st.m[MOTOR_3].steps) > 0) {
        st.m[MOTOR_3].counter -= st.timer_ticks_X_substeps;
        motor_3.step.set();
    }
    if (!motor_4.step.isNull() && (st.m[MOTOR_4].counter += st.m[MOTOR_4].steps) > 0) {
        st.m[MOTOR_4].counter -= st.timer_ticks_X_substeps;
        motor_4.step.set();
    }
    if (!motor_5.step.isNull() && (st.m[MOTOR_5].counter += st.m[MOTOR_5].steps) > 0) {
        st.m[MOTOR_5].counter -= st.timer_ticks_X_substeps;
        motor_5.step.set();
    }
    if (!motor_6.step.isNull() && (st.m[MOTOR_6].counter += st.m[MOTOR_6].steps) > 0) {
        st.m[MOTOR_6].counter -= st.timer_ticks_X_substeps;
        motor_6.step.set();
    }

    motor_1.step.clear();
    motor_2.step.clear();
    motor_3.step.clear();
    motor_4.step.clear();
    motor_5.step.clear();
    motor_6.step.clear();

    if (--st.timer_ticks_downcount == 0) {			// end move
        motor_1.enable.set();
        motor_2.enable.set();
        motor_3.enable.set();
        motor_4.enable.set();
        motor_5.enable.set();
        motor_6.enable.set();

        enable.set();								// disable DDA timer
/*
        // power-down motors if this feature is enabled
        if (cfg.m[MOTOR_1].power_mode == true) {
            PORT_MOTOR_1_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
        }
        if (cfg.m[MOTOR_2].power_mode == true) {
            PORT_MOTOR_2_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
        }
        if (cfg.m[MOTOR_3].power_mode == true) {
            PORT_MOTOR_3_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
        }
        if (cfg.m[MOTOR_4].power_mode == true) {
            PORT_MOTOR_4_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
        }
*/
        _load_move();						// load the next move
    }
    proof_of_timer = 1;
}

/****************************************************************************************
 * Exec sequencing code
 *
 * st_test_exec_state()		- return TRUE if exec/prep can run
 * st_request_exec_move()	- SW interrupt to request to execute a move
 * EXEC INTERRUPT			- interrupt handler for above
 * _exec_move() 			- Run a move from the planner and prepare it for loading
 *
 *	_exec_move() can only be called be called from an ISR at a level lower than DDA, 
 *	Only use st_request_exec_move() to call it.
 */

uint8_t st_test_exec_state()
{
	if (sps.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {
		return (true);
	}
	return (false);
}

void st_request_exec_move()
{
	if (sps.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {	// bother interrupting
		exec_timer.start();
		//TIMER_EXEC.PER = SWI_PERIOD;
		//TIMER_EXEC.CTRLA = STEP_TIMER_ENABLE;			// trigger a LO interrupt
	}
}

MOTATE_TIMER_INTERRUPT(EXEC_TIMER_NUM)			// exec move SW interrupt
{
	dummy = EXEC_STATUS_REGISTER;
	exec_timer.stop();							// disable SW interrupt timer
	_exec_move();
}
/* OLD CODE
ISR(TIMER_EXEC_ISR_vect) {						// exec move SW interrupt
	TIMER_EXEC.CTRLA = STEP_TIMER_DISABLE;		// disable SW interrupt timer
	_exec_move();
} */

static void _exec_move()
{
   	if (sps.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {
		if (mp_exec_move() != STAT_NOOP) {
			sps.exec_state = PREP_BUFFER_OWNED_BY_LOADER; // flip it back
			_request_load_move();
		}
	}
}

/****************************************************************************************
 * Load sequencing code
 *
 * _request_load()		- fires a software interrupt (timer) to request to load a move
 *  LOADER INTERRUPT	- interrupt handler for above
 * _load_move() 		- load a move into steppers, load a dwell, or process a Null move
 */

static void _request_load_move()
{
	if (st.timer_ticks_downcount == 0) {	// bother interrupting
		load_timer.start();
		//		TIMER_LOAD.PER = SWI_PERIOD;
		//		TIMER_LOAD.CTRLA = STEP_TIMER_ENABLE;
	} 	// else don't bother to interrupt. You'll just trigger an
	// interrupt and find out the load routine is not ready for you
}

MOTATE_TIMER_INTERRUPT(LOAD_TIMER_NUM)		// load steppers SW interrupt
{
	dummy = LOAD_STATUS_REGISTER;
	load_timer.stop();						// disable SW interrupt timer
	_load_move();
}
/* OLD CODE
ISR(TIMER_LOAD_ISR_vect) {					// load steppers SW interrupt
{
	TIMER_LOAD.CTRLA = STEP_TIMER_DISABLE;	// disable SW interrupt timer
	_load_move();
} */

/*
 * _load_move() - Dequeue move and load into stepper struct
 *
 *	This routine can only be called be called from an ISR at the same or 
 *	higher level as the DDA or dwell ISR. A software interrupt has been 
 *	provided to allow a non-ISR to request a load (see st_request_load_move())
 *
 *	In aline code:
 *	 - All axes must set steps and compensate for out-of-range pulse phasing.
 *	 - If axis has 0 steps the direction setting can be omitted
 *	 - If axis has 0 steps the motor must not be enabled to support power mode = 1
 */

void _load_move()
{
/*++++ TEST CODE
	sps.move_type = true;
	sps.timer_ticks = 100000;
	sps.timer_ticks_X_substeps = 1000000;
	sps.timer_period = 64000;
	
	st.m[MOTOR_1].steps = 90000;
	st.m[MOTOR_1].counter = -sps.timer_ticks;
	st.timer_ticks_X_substeps = sps.timer_ticks_X_substeps;
*/
	// handle aline loads first (most common case)  NB: there are no more lines, only alines
	if (sps.move_type == MOVE_TYPE_ALINE) {
		st.timer_ticks_downcount = sps.timer_ticks;
		st.timer_ticks_X_substeps = sps.timer_ticks_X_substeps;
//		TIMER_DDA.PER = sps.timer_period;
 
/* Old motor1 code - left in for comparison
		st.m[MOTOR_1].steps = sps.m[MOTOR_1].steps;			// set steps
		if (sps.counter_reset_flag == true) {				// compensate for pulse phasing
			st.m[MOTOR_1].counter = -(st.timer_ticks_downcount);
		}
		if (st.m[MOTOR_1].steps != 0) {
			// For ideal optimizations, only set or clear a bit at a time.
			if (sps.m[MOTOR_1].dir == 0) {
				PORT_MOTOR_1_VPORT.OUT &= ~DIRECTION_BIT_bm;// CW motion (bit cleared)
			} else {
				PORT_MOTOR_1_VPORT.OUT |= DIRECTION_BIT_bm;	// CCW motion
			}
			PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;	// enable motor
		}
*/
		st.m[MOTOR_1].steps = sps.m[MOTOR_1].steps;			// set steps
		if (sps.counter_reset_flag == true) {				// compensate for pulse phasing
			st.m[MOTOR_1].counter = -(st.timer_ticks_downcount);
		}
		if (st.m[MOTOR_1].steps != 0) {
			if (sps.m[MOTOR_1].dir == 0) {
				motor_1.dir.clear();	// clear bit for clockwise motion 
			} else {
				motor_1.dir.set();		// CCW motion
			}
			motor_1.enable.clear();		// enable motor
		}

		st.m[MOTOR_2].steps = sps.m[MOTOR_2].steps;
		if (sps.counter_reset_flag == true) {
			st.m[MOTOR_2].counter = -(st.timer_ticks_downcount);
		}
		if (st.m[MOTOR_2].steps != 0) {
			if (sps.m[MOTOR_2].dir == 0) motor_2.dir.clear(); 
			else motor_2.dir.set();
			motor_2.enable.clear();
		}

		st.m[MOTOR_3].steps = sps.m[MOTOR_3].steps;
		if (sps.counter_reset_flag == true) {
			st.m[MOTOR_3].counter = -(st.timer_ticks_downcount);
		}
		if (st.m[MOTOR_3].steps != 0) {
			if (sps.m[MOTOR_3].dir == 0) motor_3.dir.clear();
			else motor_3.dir.set();
			motor_3.enable.clear();
		}

		st.m[MOTOR_4].steps = sps.m[MOTOR_4].steps;
		if (sps.counter_reset_flag == true) {
			st.m[MOTOR_4].counter = (st.timer_ticks_downcount);
		}
		if (st.m[MOTOR_4].steps != 0) {
			if (sps.m[MOTOR_4].dir == 0) motor_4.dir.clear();
			else motor_4.dir.set();
			motor_4.enable.clear();
		}

		st.m[MOTOR_5].steps = sps.m[MOTOR_5].steps;
		if (sps.counter_reset_flag == true) {
			st.m[MOTOR_5].counter = (st.timer_ticks_downcount);
		}
		if (st.m[MOTOR_5].steps != 0) {
			if (sps.m[MOTOR_5].dir == 0) motor_5.dir.clear();
			else motor_5.dir.set();
			motor_5.enable.clear();
		}

		st.m[MOTOR_6].steps = sps.m[MOTOR_6].steps;
		if (sps.counter_reset_flag == true) {
			st.m[MOTOR_6].counter = (st.timer_ticks_downcount);
		}
		if (st.m[MOTOR_6].steps != 0) {
			if (sps.m[MOTOR_6].dir == 0) motor_6.dir.clear();
			else motor_6.dir.set();
			motor_6.enable.clear();
		}
		dda_timer.start();

	// handle dwells
	} else if (sps.move_type == MOVE_TYPE_DWELL) {
		st.timer_ticks_downcount = sps.timer_ticks;
		dwell_timer.start();
//		TIMER_DWELL.PER = sps.timer_period;						// load dwell timer period
//		TIMER_DWELL.CTRLA = STEP_TIMER_ENABLE;					// enable the dwell timer
	}

	// all other cases drop to here - such as Null moves queued by Mcodes 
	sps.exec_state = PREP_BUFFER_OWNED_BY_EXEC;					// flip it back
	st_request_exec_move();										// exec and prep next move
}

/****************************************************************************************
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

uint8_t st_prep_line(float steps[], float microseconds)
{
	uint8_t i;
	float f_dda = FREQUENCY_DDA;		// starting point for adjustment
	float dda_substeps = DDA_SUBSTEPS;
//	float major_axis_steps = 0;

	// *** defensive programming ***
	// trap conditions that would prevent queuing the line
	if (sps.exec_state != PREP_BUFFER_OWNED_BY_EXEC) { return (STAT_INTERNAL_ERROR);
	} else if (isfinite(microseconds) == false) { return (STAT_ZERO_LENGTH_MOVE);
	} else if (microseconds < EPSILON) { return (STAT_ZERO_LENGTH_MOVE);
	}
	sps.counter_reset_flag = false;		// initialize counter reset flag for this move.

	// setup motor parameters
	for (i=0; i<MOTORS; i++) {
		sps.m[i].dir = ((steps[i] < 0) ? 1 : 0) ^ cfg.m[i].polarity;
		sps.m[i].steps = (uint32_t)fabs(steps[i] * dda_substeps);
	}
	sps.timer_period = _f_to_period(f_dda);
	sps.timer_ticks = (uint32_t)((microseconds/1000000) * f_dda);
	sps.timer_ticks_X_substeps = sps.timer_ticks * dda_substeps;	// see FOOTNOTE

	// anti-stall measure in case change in velocity between segments is too great 
	if ((sps.timer_ticks * COUNTER_RESET_FACTOR) < sps.prev_ticks) {  // NB: uint32_t math
		sps.counter_reset_flag = true;
	}
	sps.prev_ticks = sps.timer_ticks;
	sps.move_type = MOVE_TYPE_ALINE;
	return (STAT_OK);
}
// FOOTNOTE: This expression was previously computed as below but floating 
// point rounding errors caused subtle and nasty accumulated position errors:
// sp.timer_ticks_X_substeps = (uint32_t)((microseconds/1000000) * f_dda * dda_substeps);

/* 
 * st_prep_null() - Keeps the loader happy. Otherwise performs no action
 *
 *	Used by M codes, tool and spindle changes
 */
void st_prep_null()
{
	sps.move_type = MOVE_TYPE_NULL;
}

/* 
 * st_prep_dwell() 	 - Add a dwell to the move buffer
 */

void st_prep_dwell(float microseconds)
{
	sps.move_type = MOVE_TYPE_DWELL;
	sps.timer_period = _f_to_period(FREQUENCY_DWELL);
	sps.timer_ticks = (uint32_t)((microseconds/1000000) * FREQUENCY_DWELL);
}

/****************************************************************************************
 * UTILITIES
 * st_isbusy()			- return TRUE if motors are running or a dwell is running
 * st_set_polarity()	- setter needed by the config system
 * st_set_microsteps()	- set microsteps in hardware
 *
 *	For now the microstep_mode is the same as the microsteps (1,2,4,8)
 *	This may change if microstep morphing is implemented.
 */
uint8_t st_isbusy()
{
	if (st.timer_ticks_downcount == 0) {
		return (false);
	} 
	return (true);
}

void st_set_polarity(const uint8_t motor, const uint8_t polarity)
{
	st.m[motor].polarity = polarity;
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
/*
#ifdef __cplusplus
}
#endif // __cplusplus
*/