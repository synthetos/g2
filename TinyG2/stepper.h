/* stepper.h - stepper motor interface
 * This file is part of TinyG2 project
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

/**** Line planning and execution ****
 *
 *	Move planning, execution and pulse generation takes place at 3 levels:
 *
 *	Move planning occurs in the main-loop. The canonical machine calls the planner to 
 *	generate lines, arcs, dwells and synchronous stop/starts. The planner module 
 *	generates blocks (bf's) that hold parameters for lines and the other move types. 
 *	The blocks are backplanned to join lines, and to take dwells and stops into account. 
 *	("plan" stage).
 *
 *	Arc movement is planned above the above the line planner. The arc planner generates 
 *	short lines that are passed to the line planner.
 *
 *	Move execution and load prep takes place at a LOW interrupt level. Move execution 
 *	generates the next acceleration, cruise, or deceleration segment for planned lines, or 
 *	just transfers parameters needed for dwells and stops. This layer also prepares moves 
 *	for loading by pre-calculating the values needed by the DDA, and converting the executed 
 *	move into parameters that can be directly loaded into the steppers ("exec" and "prep" stages).
 *
 *	Pulse train generation takes place at a HI interrupt level. The stepper DDA fires timer 
 *	interrupts that generate the stepper pulses. This level also transfers new stepper 
 *	parameters once each pulse train ("segment") is complete ("load" and "run" stages). 
 */
/* 	What happens when the pulse generator is done with the current pulse train (segment)
*	is a multi-stage "pull" that looks like this:
 *
 *	As long as the steppers are running the sequence of events is:
 *	  - The stepper interrupt (HI) runs the DDA to generate a pulse train for the current move. 
 *		  This runs for the length of the pulse train currently executing - the "segment", 
 *		  usually 5ms worth of pulses.
 *
 *	  - When the current segment is finished the stepper interrupt LOADs the next segment from
 *		  the prep buffer, reloads the timers, and starts the next segment. At the end of the 
 *		  load the stepper interrupt routine requests an "exec" of the next move in order to
 *		  prepare for the next load operation. It does this by calling the exec using a software 
 *		  interrupt (actually a timer, since that's all we've got).
 *
 *	  - As a result of the above, the EXEC handler fires at the LO interrupt level. It computes 
 *		  the next accel/decel segment for the current move (i.e. the move in the planner's 
 *		  runtime buffer) by calling back to the exec routine in planner.c. Or it gets and runs 
 *		  the next buffer in the planning queue - depending on the move_type and state. 
 *
 *	  - Once the segment has been computed the exec handler finishes up by running the PREP 
 *		  routine in stepper.c. This computes the DDA values and gets the segment into the 
 *		  prep buffer - and ready for the next LOAD operation.
 *
 *	  - The main loop runs in background to receive gcode blocks, parse them, and send them to 
 *		  the planner in order to keep the planner queue full so that when the planner's runtime 
 *		  buffer completes the next move(a gcode block or perhaps an arc segment) is ready to run.
 *
 *	If the steppers are not running the above is similar, except that the exec is invoked from the 
 *	main loop by the software interrupt, and the stepper load is invoked from the exec by another 
 *	software interrupt.
 */
/*	Control flow can be a bit confusing. This is a typical sequence for planning 
 *	executing, and running an acceleration planned line:
 *
 *	 1  planner.mp_aline() is called, which populates a planning buffer (bf) 
 *		and back-plans any pre-existing buffers.
 *
 *	 2  When a new buffer is added _mp_queue_write_buffer() tries to invoke
 *	    execution of the move by calling stepper.st_request_exec_move(). 
 *
 *	 3a If the steppers are running this request is ignored.
 *	 3b If the steppers are not running this will set a timer to cause an 
 *		EXEC "software interrupt" that will ultimately call st_exec_move().
 *
 *   4  At this point a call to _exec_move() is made, either by the 
 *		software interrupt from 3b, or once the steppers finish running 
 *		the current segment and have loaded the next segment. In either 
 *		case the call is initated via the EXEC software interrupt which 
 *		causes _exec_move() to run at the MEDium interupt level.
 *		 
 *	 5	_exec_move() calls back to planner.mp_exec_move() which generates 
 *		the next segment using the mr singleton.
 *
 *	 6	When this operation is complete mp_exec_move() calls the appropriate
 *		PREP routine in stepper.c to derive the stepper parameters that will 
 *		be needed to run the move - in this example st_prep_line().
 *
 *	 7	st_prep_line() generates the timer and DDA values and stages these into 
 *		the prep structure (sp) - ready for loading into the stepper runtime struct
 *
 *	 8	stepper.st_prep_line() returns back to planner.mp_exec_move(), which 
 *		frees the planning buffer (bf) back to the planner buffer pool if the 
 *		move is complete. This is done by calling _mp_request_finalize_run_buffer()
 *
 *	 9	At this point the MED interrupt is complete, but the planning buffer has 
 *		not actually been returned to the pool yet. The buffer will be returned
 *		by the main-loop prior to testing for an available write buffer in order
 *		to receive the next Gcode block. This handoff prevents possible data 
 *		conflicts between the interrupt and main loop.
 *
 *	10	The final step in the sequence is _load_move() requesting the next 
 *		segment to be executed and prepared by calling st_request_exec() 
 *		- control goes back to step 4.
 *
 *	Note: For this to work you have to be really careful about what structures
 *	are modified at what level, and use volatiles where necessary.
 */
/* Partial steps and phase angle compensation
 *
 *	The DDA accepts partial steps as input. Fractional steps are managed by the 
 *	sub-step value as explained elsewhere. The fraction initially loaded into 
 *	the DDA and the remainder left at the end of a move (the "residual") can
 *	be thought of as a phase angle value for the DDA accumulation. Each 360
 *	degrees of phase angle results in a step being generated. 
 */
#ifndef STEPPER_H_ONCE
#define STEPPER_H_ONCE
/*
#ifdef __cplusplus
extern "C"{
#endif
*/
void stepper_init(void);			// initialize stepper subsystem

void st_enable_motor(const uint8_t motor);
void st_enable_motors(void);
void st_disable_motor(const uint8_t motor);
void st_disable_motors(void);
void st_conditional_disable_motors(void);
void st_set_motor_disable_timeout(uint32_t seconds);
stat_t st_motor_disable_callback(void);

//void st_enable_motors(void);		// start the steppers
//void st_disable_motors(void);		// step the stoppers
stat_t st_stepper_disable_delay_callback(void);// callback to disable the steppers after a delay

uint8_t st_isbusy(void);			// return TRUE is any axis is running (F=idle)
void st_set_polarity(const uint8_t motor, const uint8_t polarity);
void st_set_microsteps(const uint8_t motor, const uint8_t microstep_mode);
void st_set_power_mode(const uint8_t motor, const uint8_t power_mode);

uint8_t st_test_prep_state(void);
void st_request_exec_move(void);
void st_prep_null(void);
void st_prep_dwell(float microseconds);
uint8_t st_prep_line(float steps[], float microseconds);

uint16_t st_get_stepper_run_magic(void);
uint16_t st_get_stepper_prep_magic(void);

//magic_t st_get_st_magic(void);
//magic_t st_get_sps_magic(void);

/*
 * Stepper configs and constants
 */

enum prepBufferState {
	PREP_BUFFER_OWNED_BY_LOADER = 0,	// staging buffer is ready for load
	PREP_BUFFER_OWNED_BY_EXEC			// staging buffer is being loaded
};

enum cmStepperPowerMode {
	ENABLE_AXIS_DURING_CYCLE =0,		// axis is fully powered during cycles
	DISABLE_AXIS_WHEN_IDLE				// power down motor immediately when idle
//	REDUCE_AXIS_POWER_WHEN_IDLE			// not implemented yet
};

/* Timer settings for stepper module. See hardware.h for overall timer assignments */

//#define FREQUENCY_DDA	50000UL
#define FREQUENCY_DDA	100000UL
#define FREQUENCY_DWELL	1000UL
#define FREQUENCY_SGI	200000UL	// 200,000 Hz means software interrupts will fire 5 uSec after being called
//#define _f_to_period(f) (uint16_t)((float)F_CPU / (float)f)		// handy macro

// DDA substepping
// 	DDA_SUBSTEPS sets the amount of fractional precision for substepping. Substepping 
//	is kind of like microsteps done in software to make interpolation more accurate.
//	Set to 1 to disable, but don't do this or you will lose a lot of accuracy.
#define DDA_SUBSTEPS 100000		// 100,000 accumulates substeps to 6 decimal places

// Accumulator resets
//	You want to reset the DDA accumulator if the new ticks value is way less than previous 
//	value, but otherwise you should leave the accumulator alone. Preserving the accumulator 
//	value from the previous segment aligns pulse phasing between segments. However, 
//	if the new accumulator value is going to be much less than the old counter you must 
//	reset it or risk motor stalls. 
#define ACCUMULATOR_RESET_FACTOR 2	// amount counter range can safely change


/*************************************************************************************
 *** END NOTES ***********************************************************************
 *************************************************************************************/

/**** Step Pulse Generation ****

  Coordinated motion (line drawing) is performed using a classic Bresenham DDA with 
  a number of enhancements to optimize line interpolation and pulse timing accuracy.

  - The DDA clock runs at a constant (maximum) frequency all the time regardless of 
	the pulse rate needed for the move segment. This minimizes aliasing error and also 
	simplifies the code. On the Xmega at 32MHz the DDA clock runs at 50 Khz. This consumes 
	roughly 50% of the Xmega cycles in step generation for 4 motors. While this is a lot, 
	it is manageable. On the SAM3X8E at 84 MHz running the DDA at 50 KHz consumes 
	approximately 8% of the CPU cycles for 6 motors. So much higher DDA clock rates 
	are supportable.

  - The DDA accepts and processes fractional motor steps. Steps are passed to the DDA 
	prep routine as floats, and do not need to be integer values. The DDA implements 
	fractional steps and interpolation by extending the counter range downward using
	the DDA_SUBSTEPS setting. 
	
  - The DDA is not used as a 'ramp' for acceleration management. Acceleration is 
	computed as 3rd order (controlled jerk) equations that generate accel/decel 
	segments to the DDA in much the same way arc drawing is approximated. 
 
  - Pulse phasing is preserved between segments if possible. This makes for smoother 
	motion, particularly at very low speeds and short segment lengths (avoids pulse 
	jitter). Phase continuity is achieved by simply not resetting the DDA counters 
	across segments. In some cases the differences between timer values across segments
	are too large for this to work, and you risk motor stalls due to pulse starvation. 
	These cases are detected and the counters are reset to prevent stalling.
 
  - Pulse phasing is also helped by minimizing the time spent loading the next move 
	segment. To this end as much as possible about that move is pre-computed during move 
	execution (prep phase). Also, all moves are loaded from the interrupt level, avoiding 
	the need for mutual exclusion locking or volatiles (which slow things down).

  - Pulses are "stretched" by using a 2 phase interrupt - i.e. the system is interrupted 
	twice for each DDA clock timer cycle (so a 50 Khz clock actually interrupts at a 
	100 Khz rate). The first phase (overflow) is used to set the step pulses that are 
	supposed to fire in that phase. The second phase (match) resets the pins. The duty
	cycle of the phases can be set but is typically 50%. The combined overhead of these 
	operations is about 8% of the SAM cpu cycles at 50 KHz.
*/
/**** Line planning and execution ****
 
   Move planning, execution and pulse generation takes place at 3 levels:

  - Move planning occurs in the main-loop. The canonical machine calls the planner to 
	generate lines, arcs, dwells and synchronous stop/starts. The planner module 
	generates blocks (bf's) that hold parameters for lines and the other move types. 
	The blocks are backplanned to join lines, and to take dwells and stops into account. 
	("plan" stage).
 
  - Arc movement is planned above the above the line planner. The arc planner generates 
	short lines that are passed to the line planner.

  - Move execution and load prep takes place at the LOW interrupt level. Move execution 
	generates the next acceleration, cruise, or deceleration segment for planned lines, 
	or just transfers parameters needed for dwells and stops. This layer also prepares 
	moves for loading by pre-calculating the values needed by the DDA, and converting the 
	executed move into parameters that can be directly loaded into the steppers 
	("exec" and "prep" stages).

  - Pulse train generation takes place at the HI interrupt level. The stepper DDA fires 
	timer interrupts that generate the stepper pulses. This level also transfers new stepper 
	parameters once each pulse train ("segment") is complete ("load" and "run" stages). 

  - What happens when the pulse generator is done with the current pulse train (segment) 
	is a multi-stage "pull" queue that looks like this:

    As long as the steppers are running the sequence of events is:
	
	  - The stepper interrupt (HI) runs the DDA to generate a pulse train for the current 
		move segment. This runs for the length of the segment currently executing; usually 
		about 3ms to 5ms worth of pulses.

	  - When the current segment is finished the stepper interrupt LOADs the next segment 
		from the prep buffer, reloads the timers, and starts the next segment. At the end 
		of the load the stepper interrupt routine requests an "exec" of the next move 
		segment in order to prepare for the next load operation. It does this by generating 
		a software interrupt using a timer. 
		
	  - The interrupt calls back to the planner's runtime code to generate the next segment, 
		which computes the next accel/cruise/decel segment for the current move. This occurs 
		at the interrupt level of the SW interrupt, so it won't be interrupted by main-loop
		functions.

	  - Once the segment has been computed the exec handler finishes up by running the PREP 
		routine in stepper.cpp. This computes the DDA values and gets the segment into the 
		prep buffer - and ready for the next LOAD operation.

	  - While this is all happening, the main loop runs in background to receive gcode blocks, 
		parse them, and send them to the planner in order to keep the planner queue full. 
		The time budget is such that the planner must be able to receive, parse, plan and queue 
		the next move in a worst case of 5ms (i.e. the previous move was a sinmle 5ms segment)

  - If the steppers are not running the above is similar, except that the exec is invoked from 
	the main loop by the software interrupt, and the stepper load is invoked from the exec by 
	another software interrupt.
 */

// DEPRECATED - REVIEW THIS AND EITHER TOSS OR UPDATE
/*	Control flow can be a bit confusing. This is a typical sequence for planning 
 *	executing, and running an acceleration planned line:
 *
 *	 1  planner.mp_aline() is called, which populates a planning buffer (bf) 
 *		and back-plans any pre-existing buffers.
 *
 *	 2  When a new buffer is added _mp_queue_write_buffer() tries to invoke
 *	    execution of the move by calling stepper.st_request_exec_move(). 
 *
 *	 3a If the steppers are running this request is ignored.
 *	 3b If the steppers are not running this will set a timer to cause an 
 *		EXEC "software interrupt" that will ultimately call st_exec_move().
 *
 *   4  At this point a call to _exec_move() is made, either by the 
 *		software interrupt from 3b, or once the steppers finish running 
 *		the current segment and have loaded the next segment. In either 
 *		case the call is initiated via the EXEC software interrupt which 
 *		causes _exec_move() to run at the MEDium interupt level.
 *		 
 *	 5	_exec_move() calls back to planner.mp_exec_move() which generates 
 *		the next segment using the mr singleton.
 *
 *	 6	When this operation is complete mp_exec_move() calls the appropriate
 *		PREP routine in stepper.c to derive the stepper parameters that will 
 *		be needed to run the move - in this example st_prep_line().
 *
 *	 7	st_prep_line() generates the timer and DDA values and stages these into 
 *		the prep structure (sp) - ready for loading into the stepper runtime struct
 *
 *	 8	stepper.st_prep_line() returns back to planner.mp_exec_move(), which 
 *		frees the planning buffer (bf) back to the planner buffer pool if the 
 *		move is complete. This is done by calling _mp_request_finalize_run_buffer()
 *
 *	 9	At this point the MED interrupt is complete, but the planning buffer has 
 *		not actually been returned to the pool yet. The buffer will be returned
 *		by the main-loop prior to testing for an available write buffer in order
 *		to receive the next Gcode block. This handoff prevents possible data 
 *		conflicts between the interrupt and main loop.
 *
 *	10	The final step in the sequence is _load_move() requesting the next 
 *		segment to be executed and prepared by calling st_request_exec() 
 *		- control goes back to step 4.
 *
 *	Note: For this to work you have to be really careful about what structures
 *	are modified at what level, and use volatiles where necessary.
 */
/*
#ifdef __cplusplus
}
#endif // __cplusplus
*/
#endif	// End of include guard: STEPPER_H_ONCE
