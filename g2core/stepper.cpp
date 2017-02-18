/*
 * stepper.cpp - stepper motor controls
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2016 Robert Giseburt
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
/*  This module provides the low-level stepper drivers and some related functions.
 *  See stepper.h for a detailed explanation of this module.
 */

#include "g2core.h"
#include "config.h"
#include "stepper.h"
#include "encoder.h"
#include "planner.h"
#include "hardware.h"
#include "text_parser.h"
#include "util.h"
#include "controller.h"
#include "xio.h"

/**** Debugging output with semihosting ****/

#include "MotateDebug.h"

// Unless debugging, this should always read "#if 0 && ..."
// DON'T COMMIT with anything else!
#if 0 && (IN_DEBUGGER == 1)
template<int32_t len>
void stepper_debug(const char (&str)[len]) { Motate::debug.write(str); };
#else
template<int32_t len>
void stepper_debug(const char (&str)[len]) { ; };
#endif

/**** Allocate structures ****/

stConfig_t st_cfg;
stPrepSingleton_t st_pre;
static stRunSingleton_t st_run;

/**** Static functions ****/

static void _load_move(void);

// handy macro
//#define _f_to_period(f) (uint16_t)((float)F_CPU / (float)f)

/**** Setup motate ****/

using namespace Motate;
extern OutputPin<kDebug1_PinNumber> debug_pin1;
extern OutputPin<kDebug2_PinNumber> debug_pin2;
extern OutputPin<kDebug3_PinNumber> debug_pin3;
//extern OutputPin<kDebug4_PinNumber> debug_pin4;

dda_timer_type dda_timer     {kTimerUpToMatch, FREQUENCY_DDA};      // stepper pulse generation
exec_timer_type exec_timer;         // triggers calculation of next+1 stepper segment
fwd_plan_timer_type fwd_plan_timer; // triggers planning of next block

// SystickEvent for handling dweels (must be registered before it is active)
Motate::SysTickEvent dwell_systick_event {[&] {
    if (--st_run.dwell_ticks_downcount == 0) {
        SysTickTimer.unregisterEvent(&dwell_systick_event);
        _load_move();       // load the next move at the current interrupt level
    }
}, nullptr};


/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/
/*
 * stepper_init() - initialize stepper motor subsystem
 * stepper_reset() - reset stepper motor subsystem
 *
 *  Notes:
 *    - This init requires sys_init() to be run beforehand
 *    - microsteps are setup during config_init()
 *    - motor polarity is setup during config_init()
 *    - high level interrupts must be enabled in main() once all inits are complete
 */
/*  NOTE: This is the bare code that the Motate timer calls replace.
 *  NB: requires: #include <component_tc.h>
 *
 *  REG_TC1_WPMR = 0x54494D00;              // enable write to registers
 *  TC_Configure(TC_BLOCK_DDA, TC_CHANNEL_DDA, TC_CMR_DDA);
 *  REG_RC_DDA = TC_RC_DDA;                 // set frequency
 *  REG_IER_DDA = TC_IER_DDA;               // enable interrupts
 *  NVIC_EnableIRQ(TC_IRQn_DDA);
 *  pmc_enable_periph_clk(TC_ID_DDA);
 *  TC_Start(TC_BLOCK_DDA, TC_CHANNEL_DDA);
 */
void stepper_init()
{
    memset(&st_run, 0, sizeof(st_run));            // clear all values, pointers and status
    memset(&st_pre, 0, sizeof(st_pre));            // clear all values, pointers and status
    stepper_init_assertions();

    // setup DDA timer
    // Longer duty cycles stretch ON pulses but 75% is about the upper limit and about
    // optimal for 200 KHz DDA clock before the time in the OFF cycle is too short.
    // If you need more pulse width you need to drop the DDA clock rate
    dda_timer.setInterrupts(kInterruptOnOverflow | kInterruptPriorityHighest);

    // setup software interrupt exec timer & initial condition
    exec_timer.setInterrupts(kInterruptOnSoftwareTrigger | kInterruptPriorityHigh);
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;

    // setup software interrupt forward plan timer & initial condition
    fwd_plan_timer.setInterrupts(kInterruptOnSoftwareTrigger | kInterruptPriorityMedium);

    // setup motor power levels and apply power level to stepper drivers
    for (uint8_t motor=0; motor<MOTORS; motor++) {
        Motors[motor]->setPowerLevel(st_cfg.mot[motor].power_level_scaled);
        st_run.mot[motor].power_level_dynamic = st_cfg.mot[motor].power_level_scaled;
    }
    board_stepper_init();
    stepper_reset();                            // reset steppers to known state
}

/*
 * stepper_reset() - reset stepper internals
 *
 * Used to initialize stepper and also to halt movement
 */

void stepper_reset()
{
    dda_timer.stop();                                   // stop all movement
    st_run.dda_ticks_downcount = 0;                     // signal the runtime is not busy
    st_run.dwell_ticks_downcount = 0;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;    // set to EXEC or it won't restart

    for (uint8_t motor=0; motor<MOTORS; motor++) {
        st_pre.mot[motor].prev_direction = STEP_INITIAL_DIRECTION;
        st_pre.mot[motor].direction = STEP_INITIAL_DIRECTION;
        st_run.mot[motor].substep_accumulator = 0;      // will become max negative during per-motor setup;
        st_pre.mot[motor].corrected_steps = 0;          // diagnostic only - no action effect
    }
    mp_set_steps_to_runtime_position();                 // reset encoder to agree with the above
}

/*
 * stepper_init_assertions() - test assertions, return error code if violation exists
 * stepper_test_assertions() - test assertions, return error code if violation exists
 */

void stepper_init_assertions()
{
    st_run.magic_end = MAGICNUM;
    st_run.magic_start = MAGICNUM;
    st_pre.magic_end = MAGICNUM;
    st_pre.magic_start = MAGICNUM;
}

stat_t stepper_test_assertions()
{
    if ((BAD_MAGIC(st_run.magic_start)) || (BAD_MAGIC(st_run.magic_end)) ||
        (BAD_MAGIC(st_pre.magic_start)) || (BAD_MAGIC(st_pre.magic_end))) {
        return(cm_panic(STAT_STEPPER_ASSERTION_FAILURE, "stepper_test_assertions()"));
    }
    return (STAT_OK);
}

/*
 * st_runtime_isbusy() - return TRUE if runtime is busy:
 *
 *  Busy conditions:
 *  - motors are running
 *  - dwell is running
 */

bool st_runtime_isbusy()
{
    return (st_run.dda_ticks_downcount || st_run.dwell_ticks_downcount);    // returns false if down count is zero
}

/*
 * st_clc() - clear counters
 */

stat_t st_clc(nvObj_t *nv)    // clear diagnostic counters, reset stepper prep
{
    stepper_reset();
    return(STAT_OK);
}

/*
 * st_motor_power_callback() - callback to manage motor power sequencing
 *
 *  Handles motor power-down timing, low-power idle, and adaptive motor power
 */
stat_t st_motor_power_callback()     // called by controller
{
    if (!mp_is_phat_city_time()) {   // don't process this if you are time constrained in the planner
        return (STAT_NOOP);
    }

    bool have_actually_stopped = false;
    if ((!st_runtime_isbusy()) &&
        (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER) &&
        (cm_get_cycle_state() == CYCLE_OFF)
        )
    {    // if there are no moves to load...
        have_actually_stopped = true;
    }

    // manage power for each motor individually
    for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
        Motors[motor]->periodicCheck(have_actually_stopped);
    }
    return (STAT_OK);
}

/******************************
 * Interrupt Service Routines *
 ******************************/

/***** Stepper Interrupt Service Routine ************************************************
 * ISR - DDA timer interrupt routine - service ticks from DDA timer
 */

/*
 *  The DDA timer interrupt does this:
 *    - fire on overflow
 *    - clear interrupt condition
 *    - clear all step pins - this clears those the were set during the previous interrupt
 *    - if downcount == 0 and stop the timer and exit
 *    - run the DDA for each channel
 *    - decrement the downcount - if it reaches zero load the next segment
 *
 *  Note that the motor_N.step.isNull() tests are compile-time tests, not run-time tests.
 *  If motor_N is not defined that if{} clause (i.e. that motor) drops out of the complied code.
 */

namespace Motate {            // Must define timer interrupts inside the Motate namespace
template<>
void dda_timer_type::interrupt()
{
    dda_timer.getInterruptCause();  // clear interrupt condition

    // clear all steps from the previous interrupt
	// for (uint8_t motor=0; motor<MOTORS; motor++) {
	//	  Motors[motor]->stepEnd();
	// }
	// loop unrolled version (it's actually faster)
    motor_1.stepEnd();
    motor_2.stepEnd();
#if MOTORS > 2
    motor_3.stepEnd();
#endif
#if MOTORS > 3
    motor_4.stepEnd();
#endif
#if MOTORS > 4
    motor_5.stepEnd();
#endif
#if MOTORS > 5
    motor_6.stepEnd();
#endif

    // process last DDA tick after end of segment
    if (st_run.dda_ticks_downcount == 0) {
        dda_timer.stop(); // turn it off or it will keep stepping out the last segment
        return;
    }

//  The following code would work, but it's faster on the M3 to loop unroll it. Perhaps not on the M7
//    for (uint8_t motor=0; motor<MOTORS; motor++) {
//        if  ((st_run.mot[motor].substep_accumulator += st_run.mot[motor].substep_increment) > 0) {
//            Motors[motor]->stepStart();        // turn step bit on
//            st_run.mot[motor].substep_accumulator -= st_run.dda_ticks_X_substeps;
//            INCREMENT_ENCODER(motor);
//        }
//    }

    // process DDAs for each motor
        if  ((st_run.mot[MOTOR_1].substep_accumulator += st_run.mot[MOTOR_1].substep_increment) > 0) {
            motor_1.stepStart();        // turn step bit on
            st_run.mot[MOTOR_1].substep_accumulator -= st_run.dda_ticks_X_substeps;
            INCREMENT_ENCODER(MOTOR_1);
        }
        if ((st_run.mot[MOTOR_2].substep_accumulator += st_run.mot[MOTOR_2].substep_increment) > 0) {
            motor_2.stepStart();        // turn step bit on
            st_run.mot[MOTOR_2].substep_accumulator -= st_run.dda_ticks_X_substeps;
            INCREMENT_ENCODER(MOTOR_2);
        }
#if MOTORS > 2
        if ((st_run.mot[MOTOR_3].substep_accumulator += st_run.mot[MOTOR_3].substep_increment) > 0) {
            motor_3.stepStart();        // turn step bit on
            st_run.mot[MOTOR_3].substep_accumulator -= st_run.dda_ticks_X_substeps;
            INCREMENT_ENCODER(MOTOR_3);
        }
#endif
#if MOTORS > 3
        if ((st_run.mot[MOTOR_4].substep_accumulator += st_run.mot[MOTOR_4].substep_increment) > 0) {
            motor_4.stepStart();        // turn step bit on
            st_run.mot[MOTOR_4].substep_accumulator -= st_run.dda_ticks_X_substeps;
            INCREMENT_ENCODER(MOTOR_4);
        }
#endif
#if MOTORS > 4
        if ((st_run.mot[MOTOR_5].substep_accumulator += st_run.mot[MOTOR_5].substep_increment) > 0) {
            motor_5.stepStart();        // turn step bit on
            st_run.mot[MOTOR_5].substep_accumulator -= st_run.dda_ticks_X_substeps;
            INCREMENT_ENCODER(MOTOR_5);
        }
#endif
#if MOTORS > 5
        if ((st_run.mot[MOTOR_6].substep_accumulator += st_run.mot[MOTOR_6].substep_increment) > 0) {
            motor_6.stepStart();        // turn step bit on
            st_run.mot[MOTOR_6].substep_accumulator -= st_run.dda_ticks_X_substeps;
            INCREMENT_ENCODER(MOTOR_6);
        }
#endif

    // Process end of segment. 
    // One more interrupt will occur to turn of any pulses set in this pass.
    if (--st_run.dda_ticks_downcount == 0) {
        _load_move();       // load the next move at the current interrupt level
    }
} // MOTATE_TIMER_INTERRUPT
} // namespace Motate

/****************************************************************************************
 * Exec sequencing code   - computes and prepares next load segment
 * st_request_exec_move() - SW interrupt to request to execute a move
 * exec_timer interrupt   - interrupt handler for calling exec function
 */

void st_request_exec_move()
{
    stepper_debug("e");
    exec_timer.setInterruptPending();
    stepper_debug("!\n");
}

namespace Motate {    // Define timer inside Motate namespace
    template<>
    void exec_timer_type::interrupt()
    {
        exec_timer.getInterruptCause();                    // clears the interrupt condition
        if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_EXEC) {
            stepper_debug("E>");
            if (mp_exec_move() != STAT_NOOP) {
                stepper_debug("E+\n");
                st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER; // flip it back
                st_request_load_move();
                return;
            }
            stepper_debug("E-\n");
        }
    }
} // namespace Motate

void st_request_forward_plan()
{
    stepper_debug("p");
    fwd_plan_timer.setInterruptPending();
}

namespace Motate {    // Define timer inside Motate namespace
    template<>
    void fwd_plan_timer_type::interrupt()
    {
        fwd_plan_timer.getInterruptCause();     // clears the interrupt condition
        stepper_debug("P>");
        if (mp_forward_plan() != STAT_NOOP) {   // We now have a move to exec.
            stepper_debug("P+\n");
            st_request_exec_move();
            return;
        }
        stepper_debug("P-\n");
    }
} // namespace Motate

/****************************************************************************************
 * Loader sequencing code
 * st_request_load_move() - fires a software interrupt (timer) to request to load a move
 * load_move interrupt    - interrupt handler for running the loader
 *
 *  _load_move() can only be called be called from an ISR at the same or higher level as
 *  the DDA or dwell ISR. A software interrupt has been provided to allow a non-ISR to
 *  request a load (see st_request_load_move())
 */

void st_request_load_move()
{
    if (st_runtime_isbusy()) {                                      // don't request a load if the runtime is busy
        return;
    }
    stepper_debug("l");
    if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_LOADER) {       // bother interrupting
        stepper_debug("_");
        _load_move();
    }
}

/****************************************************************************************
 * _load_move() - Dequeue move and load into stepper runtime structure
 *
 *  This routine can only be called be called from an ISR at the same or
 *  higher level as the DDA or dwell ISR. A software interrupt has been
 *  provided to allow a non-ISR to request a load (st_request_load_move())
 *
 *  In aline() code:
 *   - All axes must set steps and compensate for out-of-range pulse phasing.
 *   - If axis has 0 steps the direction setting can be omitted
 *   - If axis has 0 steps the motor power must be set accord to the power mode
 */

static void _load_move()
{
    // Be aware that dda_ticks_downcount must equal zero for the loader to run.
    // So the initial load must also have this set to zero as part of initialization
    if (st_runtime_isbusy()) {
        return;                                                    // exit if the runtime is busy
    }
    if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER) {    // if there are no moves to load...

        if (cm.motion_state == MOTION_RUN)  {
#if IN_DEBUGGER == 1
//#warning debbugger REQUIRED for running this firmware!
//            __asm__("BKPT"); // attempted to _load_move with PREP_BUFFER_OWNED_BY_EXEC and cm.motion_state == MOTION_RUN
#endif
            st_request_exec_move();
            return;
        }

	// ...start motor power timeouts
	//	for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
	//		Motors[motor]->motionStopped();    
	//  }
	// loop unrolled version
        motor_1.motionStopped();    // ...start motor power timeouts
        motor_2.motionStopped();    // ...start motor power timeouts
#if (MOTORS > 2)
        motor_3.motionStopped();    // ...start motor power timeouts
#endif
#if (MOTORS > 3)
        motor_4.motionStopped();    // ...start motor power timeouts
#endif
#if (MOTORS > 4)
        motor_5.motionStopped();    // ...start motor power timeouts
#endif
#if (MOTORS > 5)
        motor_6.motionStopped();    // ...start motor power timeouts
#endif
        stepper_debug("•");
        return;
    } // if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER)

    stepper_debug("^");

    // handle aline loads first (most common case)  NB: there are no more lines, only alines
    if (st_pre.block_type == BLOCK_TYPE_ALINE) {

        //**** setup the new segment ****

        st_run.dda_ticks_downcount = st_pre.dda_ticks;
        st_run.dda_ticks_X_substeps = st_pre.dda_ticks_X_substeps;

        // INLINED VERSION: 4.3us
        //**** MOTOR_1 LOAD ****

        // These sections are somewhat optimized for execution speed. The whole load operation
        // is supposed to take < 5 uSec (Arm M3 core). Be careful if you mess with this.

        // the following if() statement sets the runtime substep increment value or zeroes it
        if ((st_run.mot[MOTOR_1].substep_increment = st_pre.mot[MOTOR_1].substep_increment) != 0) {

            // NB: If motor has 0 steps the following is all skipped. This ensures that state comparisons
            //     always operate on the last segment actually run by this motor, regardless of how many
            //     segments it may have been inactive in between.

            // Apply accumulator correction if the time base has changed since previous segment
            if (st_pre.mot[MOTOR_1].accumulator_correction_flag == true) {
                st_pre.mot[MOTOR_1].accumulator_correction_flag = false;
                st_run.mot[MOTOR_1].substep_accumulator *= st_pre.mot[MOTOR_1].accumulator_correction;
            }

            // Detect direction change and if so:
            //    Set the direction bit in hardware.
            //    Compensate for direction change by flipping substep accumulator value about its midpoint.

            if (st_pre.mot[MOTOR_1].direction != st_pre.mot[MOTOR_1].prev_direction) {
                st_pre.mot[MOTOR_1].prev_direction = st_pre.mot[MOTOR_1].direction;
                st_run.mot[MOTOR_1].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_1].substep_accumulator);
                motor_1.setDirection(st_pre.mot[MOTOR_1].direction);
            }

            // Enable the stepper and start/update motor power management
            motor_1.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_1, st_pre.mot[MOTOR_1].step_sign);

        } else {  // Motor has 0 steps; might need to energize motor for power mode processing
            motor_1.motionStopped();
        }
        // accumulate counted steps to the step position and zero out counted steps for the segment currently being loaded
        ACCUMULATE_ENCODER(MOTOR_1);

#if (MOTORS >= 2)
        if ((st_run.mot[MOTOR_2].substep_increment = st_pre.mot[MOTOR_2].substep_increment) != 0) {
            if (st_pre.mot[MOTOR_2].accumulator_correction_flag == true) {
                st_pre.mot[MOTOR_2].accumulator_correction_flag = false;
                st_run.mot[MOTOR_2].substep_accumulator *= st_pre.mot[MOTOR_2].accumulator_correction;
            }
            if (st_pre.mot[MOTOR_2].direction != st_pre.mot[MOTOR_2].prev_direction) {
                st_pre.mot[MOTOR_2].prev_direction = st_pre.mot[MOTOR_2].direction;
                st_run.mot[MOTOR_2].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_2].substep_accumulator);
                motor_2.setDirection(st_pre.mot[MOTOR_2].direction);
            }
            motor_2.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_2, st_pre.mot[MOTOR_2].step_sign);
        } else {
            motor_2.motionStopped();
        }
        ACCUMULATE_ENCODER(MOTOR_2);
#endif
#if (MOTORS >= 3)
        if ((st_run.mot[MOTOR_3].substep_increment = st_pre.mot[MOTOR_3].substep_increment) != 0) {
            if (st_pre.mot[MOTOR_3].accumulator_correction_flag == true) {
                st_pre.mot[MOTOR_3].accumulator_correction_flag = false;
                st_run.mot[MOTOR_3].substep_accumulator *= st_pre.mot[MOTOR_3].accumulator_correction;
            }
            if (st_pre.mot[MOTOR_3].direction != st_pre.mot[MOTOR_3].prev_direction) {
                st_pre.mot[MOTOR_3].prev_direction = st_pre.mot[MOTOR_3].direction;
                st_run.mot[MOTOR_3].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_3].substep_accumulator);
                motor_3.setDirection(st_pre.mot[MOTOR_3].direction);
            }
            motor_3.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_3, st_pre.mot[MOTOR_3].step_sign);
        } else {
            motor_3.motionStopped();
        }
        ACCUMULATE_ENCODER(MOTOR_3);
#endif
#if (MOTORS >= 4)
        if ((st_run.mot[MOTOR_4].substep_increment = st_pre.mot[MOTOR_4].substep_increment) != 0) {
            if (st_pre.mot[MOTOR_4].accumulator_correction_flag == true) {
                st_pre.mot[MOTOR_4].accumulator_correction_flag = false;
                st_run.mot[MOTOR_4].substep_accumulator *= st_pre.mot[MOTOR_4].accumulator_correction;
            }
            if (st_pre.mot[MOTOR_4].direction != st_pre.mot[MOTOR_4].prev_direction) {
                st_pre.mot[MOTOR_4].prev_direction = st_pre.mot[MOTOR_4].direction;
                st_run.mot[MOTOR_4].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_4].substep_accumulator);
                motor_4.setDirection(st_pre.mot[MOTOR_4].direction);
            }
            motor_4.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_4, st_pre.mot[MOTOR_4].step_sign);
        } else {
            motor_4.motionStopped();
        }
        ACCUMULATE_ENCODER(MOTOR_4);
#endif
#if (MOTORS >= 5)
        if ((st_run.mot[MOTOR_5].substep_increment = st_pre.mot[MOTOR_5].substep_increment) != 0) {
            if (st_pre.mot[MOTOR_5].accumulator_correction_flag == true) {
                st_pre.mot[MOTOR_5].accumulator_correction_flag = false;
                st_run.mot[MOTOR_5].substep_accumulator *= st_pre.mot[MOTOR_5].accumulator_correction;
            }
            if (st_pre.mot[MOTOR_5].direction != st_pre.mot[MOTOR_5].prev_direction) {
                st_pre.mot[MOTOR_5].prev_direction = st_pre.mot[MOTOR_5].direction;
                st_run.mot[MOTOR_5].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_5].substep_accumulator);
                motor_5.setDirection(st_pre.mot[MOTOR_5].direction);
            }
            motor_5.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_5, st_pre.mot[MOTOR_5].step_sign);
        } else {
            motor_5.motionStopped();
        }
        ACCUMULATE_ENCODER(MOTOR_5);
#endif
#if (MOTORS >= 6)
        if ((st_run.mot[MOTOR_6].substep_increment = st_pre.mot[MOTOR_6].substep_increment) != 0) {
            if (st_pre.mot[MOTOR_6].accumulator_correction_flag == true) {
                st_pre.mot[MOTOR_6].accumulator_correction_flag = false;
                st_run.mot[MOTOR_6].substep_accumulator *= st_pre.mot[MOTOR_6].accumulator_correction;
            }
            if (st_pre.mot[MOTOR_6].direction != st_pre.mot[MOTOR_6].prev_direction) {
                st_pre.mot[MOTOR_6].prev_direction = st_pre.mot[MOTOR_6].direction;
                st_run.mot[MOTOR_6].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_6].substep_accumulator);
                motor_6.setDirection(st_pre.mot[MOTOR_6].direction);
            }
            motor_6.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_6, st_pre.mot[MOTOR_6].step_sign);
        } else {
            motor_6.motionStopped();
        }
        ACCUMULATE_ENCODER(MOTOR_6);
#endif

        //**** do this last ****

        dda_timer.start();                              // start the DDA timer if not already running

    // handle dwells and commands
    } else if (st_pre.block_type == BLOCK_TYPE_DWELL) {
        st_run.dwell_ticks_downcount = st_pre.dwell_ticks;

        // We now use SysTick events to handle dwells
        SysTickTimer.registerEvent(&dwell_systick_event);

        // handle synchronous commands
    } else if (st_pre.block_type == BLOCK_TYPE_COMMAND) {
        mp_runtime_command(st_pre.bf);
        
    } // else null - which is okay in many cases

    // all other cases drop to here (e.g. Null moves after Mcodes skip to here)
    st_pre.block_type = BLOCK_TYPE_NULL;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;    // we are done with the prep buffer - flip the flag back
    st_request_exec_move();                             // exec and prep next move
}

/***********************************************************************************
 * st_prep_line() - Prepare the next move for the loader
 *
 *  This function does the math on the next pulse segment and gets it ready for
 *  the loader. It deals with all the DDA optimizations and timer setups so that
 *  loading can be performed as rapidly as possible. It works in joint space
 *  (motors) and it works in steps, not length units. All args are provided as
 *  floats and converted to their appropriate integer types for the loader.
 *
 * Args:
 *    - travel_steps[] are signed relative motion in steps for each motor. Steps are
 *      floats that typically have fractional values (fractional steps). The sign
 *      indicates direction. Motors that are not in the move should be 0 steps on input.
 *
 *    - following_error[] is a vector of measured errors to the step count. Used for correction.
 *
 *    - segment_time - how many minutes the segment should run. If timing is not
 *      100% accurate this will affect the move velocity, but not the distance traveled.
 *
 * NOTE:  Many of the expressions are sensitive to casting and execution order to avoid long-term
 *        accuracy errors due to floating point round off. One earlier failed attempt was:
 *          dda_ticks_X_substeps = (int32_t)((microseconds/1000000) * f_dda * dda_substeps);
 */

stat_t st_prep_line(float travel_steps[], float following_error[], float segment_time)
{
    stepper_debug("😶");
    // trap assertion failures and other conditions that would prevent queuing the line
    if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_EXEC) {     // never supposed to happen
        return (cm_panic(STAT_INTERNAL_ERROR, "st_prep_line() prep sync error"));
    } else if (isinf(segment_time)) {                           // never supposed to happen
        return (cm_panic(STAT_PREP_LINE_MOVE_TIME_IS_INFINITE, "st_prep_line()"));
    } else if (isnan(segment_time)) {                           // never supposed to happen
        return (cm_panic(STAT_PREP_LINE_MOVE_TIME_IS_NAN, "st_prep_line()"));
//    } else if (segment_time < EPSILON) {
//        return (STAT_MINIMUM_TIME_MOVE);
    }
    // setup segment parameters
    // - dda_ticks is the integer number of DDA clock ticks needed to play out the segment
    // - ticks_X_substeps is the maximum depth of the DDA accumulator (as a negative number)

    //st_pre.dda_period = _f_to_period(FREQUENCY_DDA);                // FYI: this is a constant
    st_pre.dda_ticks = (int32_t)(segment_time * 60 * FREQUENCY_DDA);// NB: converts minutes to seconds
    st_pre.dda_ticks_X_substeps = st_pre.dda_ticks * DDA_SUBSTEPS;

    // setup motor parameters

    float correction_steps;
    for (uint8_t motor=0; motor<MOTORS; motor++) {          // remind us that this is motors, not axes

        // Skip this motor if there are no new steps. Leave all other values intact.
        if (fp_ZERO(travel_steps[motor])) {
            st_pre.mot[motor].substep_increment = 0;        // substep increment also acts as a motor flag
            continue;
        }

        // Setup the direction, compensating for polarity.
        // Set the step_sign which is used by the stepper ISR to accumulate step position

        if (travel_steps[motor] >= 0) {                    // positive direction
            st_pre.mot[motor].direction = DIRECTION_CW ^ st_cfg.mot[motor].polarity;
            st_pre.mot[motor].step_sign = 1;
        } else {
            st_pre.mot[motor].direction = DIRECTION_CCW ^ st_cfg.mot[motor].polarity;
            st_pre.mot[motor].step_sign = -1;
        }

        // Detect segment time changes and setup the accumulator correction factor and flag.
        // Putting this here computes the correct factor even if the motor was dormant for some
        // number of previous moves. Correction is computed based on the last segment time actually used.

        if (fabs(segment_time - st_pre.mot[motor].prev_segment_time) > 0.0000001) { // highly tuned FP != compare
            if (fp_NOT_ZERO(st_pre.mot[motor].prev_segment_time)) {                    // special case to skip first move
                st_pre.mot[motor].accumulator_correction_flag = true;
                st_pre.mot[motor].accumulator_correction = segment_time / st_pre.mot[motor].prev_segment_time;
            }
            st_pre.mot[motor].prev_segment_time = segment_time;
        }

        // 'Nudge' correction strategy. Inject a single, scaled correction value then hold off
        // NOTE: This clause can be commented out to test for numerical accuracy and accumulating errors
        if ((--st_pre.mot[motor].correction_holdoff < 0) &&
            (fabs(following_error[motor]) > STEP_CORRECTION_THRESHOLD)) {

            st_pre.mot[motor].correction_holdoff = STEP_CORRECTION_HOLDOFF;
            correction_steps = following_error[motor] * STEP_CORRECTION_FACTOR;

            if (correction_steps > 0) {
                correction_steps = min3(correction_steps, fabs(travel_steps[motor]), STEP_CORRECTION_MAX);
            } else {
                correction_steps = max3(correction_steps, -fabs(travel_steps[motor]), -STEP_CORRECTION_MAX);
            }
            st_pre.mot[motor].corrected_steps += correction_steps;
            travel_steps[motor] -= correction_steps;
        }

        // Compute substeb increment. The accumulator must be *exactly* the incoming
        // fractional steps times the substep multiplier or positional drift will occur.
        // Rounding is performed to eliminate a negative bias in the uint32 conversion
        // that results in long-term negative drift. (fabs/round order doesn't matter)

        st_pre.mot[motor].substep_increment = round(fabs(travel_steps[motor] * DDA_SUBSTEPS));
    }
    st_pre.block_type = BLOCK_TYPE_ALINE;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;    // signal that prep buffer is ready
    stepper_debug("👍🏻");
    return (STAT_OK);
}

/*
 * st_prep_null() - Keeps the loader happy. Otherwise performs no action
 */

void st_prep_null()
{
    st_pre.block_type = BLOCK_TYPE_NULL;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;    // signal that prep buffer is empty
}

/*
 * st_prep_command() - Stage command to execution
 */

void st_prep_command(void *bf)
{
    st_pre.block_type = BLOCK_TYPE_COMMAND;
    st_pre.bf = (mpBuf_t *)bf;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;    // signal that prep buffer is ready
}

/*
 * st_prep_dwell()      - Add a dwell to the move buffer
 */

void st_prep_dwell(float microseconds)
{
    st_pre.block_type = BLOCK_TYPE_DWELL;
    // we need dwell_ticks to be at least 1
    st_pre.dwell_ticks = std::max((uint32_t)((microseconds/1000000) * FREQUENCY_DWELL), 1UL);
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;    // signal that prep buffer is ready
}

/*
 * st_request_out_of_band_dwell()
 * (only usable while exec isn't running, e.g. in feedhold or stopped states...)
 * add a dwell to the loader without going through the planner buffers
 */
void st_request_out_of_band_dwell(float microseconds)
{
    st_prep_dwell(microseconds);
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;    // signal that prep buffer is ready
    st_request_load_move();
}

/*
 * _set_hw_microsteps() - set microsteps in hardware
 */
static void _set_hw_microsteps(const uint8_t motor, const uint8_t microsteps)
{
    if (motor >= MOTORS) {return;}

    Motors[motor]->setMicrosteps(microsteps);
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
    char *ptr;
    char motors[] = {"123456"};
    char tmp[GROUP_LEN+1];

    strcpy(tmp, cfgArray[index].group);
    if ((ptr = strchr(motors, tmp[0])) == NULL) {
        return (-1);
    }
    return (ptr - motors);
}

/*
 * _set_motor_steps_per_unit() - what it says
 * This function will need to be rethought if microstep morphing is implemented
 */

static void _set_motor_steps_per_unit(nvObj_t *nv)
{
    uint8_t m = _get_motor(nv->index);
    st_cfg.mot[m].units_per_step = (st_cfg.mot[m].travel_rev * st_cfg.mot[m].step_angle) / (360 * st_cfg.mot[m].microsteps);
    st_cfg.mot[m].steps_per_unit = 1/st_cfg.mot[m].units_per_step;
}

/* PER-MOTOR FUNCTIONS
 * st_set_ma() - map motor to axis
 * st_set_sa() - set motor step angle
 * st_set_tr() - set travel per motor revolution
 * st_set_mi() - set motor microsteps
 * st_set_pm() - set motor power mode
 * st_get_pm() - get motor power mode
 * st_set_pl() - set motor power level
 */

stat_t st_set_ma(nvObj_t *nv)            // map motor to axis
{
    if (nv->value < 0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value >= AXES) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    set_ui8(nv);
    return(STAT_OK);
}

stat_t st_set_sa(nvObj_t *nv)            // motor step angle
{
    if (nv->value <= 0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value >= 360) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    set_flt(nv);
    _set_motor_steps_per_unit(nv);
    return(STAT_OK);
}

stat_t st_set_tr(nvObj_t *nv)            // motor travel per revolution
{
    if (nv->value <= 0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    set_flu(nv);
    _set_motor_steps_per_unit(nv);
    return(STAT_OK);
}

stat_t st_set_mi(nvObj_t *nv)            // motor microsteps
{
    if (nv->value <= 0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }

    uint8_t mi = (uint8_t)nv->value;
    if ((mi != 1) && (mi != 2) && (mi != 4) && (mi != 8) && (mi != 16) && (mi != 32)) {
        nv_add_conditional_message((const char *)"*** WARNING *** Setting non-standard microstep value");
    }
    set_ui8(nv);                        // set it anyway, even if it's unsupported
    _set_motor_steps_per_unit(nv);
    _set_hw_microsteps(_get_motor(nv->index), (uint8_t)nv->value);
    return (STAT_OK);
}

stat_t st_get_su(nvObj_t *nv)			// motor steps per unit (direct)
{
    uint8_t m = _get_motor(nv->index);
    nv->value = st_cfg.mot[m].steps_per_unit;
	nv->valuetype = TYPE_FLOAT;
    nv->precision = cfgArray[nv->index].precision;
    return(STAT_OK);
}

stat_t st_set_su(nvObj_t *nv)			// motor steps per unit (direct)
{
    // Don't set a zero or negative value - just calculate based on sa, tr, and mi
    // This way, if STEPS_PER_UNIT is set to 0 it is unused and we get the computed value
    uint8_t m = _get_motor(nv->index);
    if(nv->value <= 0) {
        nv->value = st_cfg.mot[m].steps_per_unit;
        _set_motor_steps_per_unit(nv);
        return(STAT_OK);
    }

    // Do unit conversion here because it's a reciprocal value (rather than process_incoming_float())
    if (cm_get_units_mode(MODEL) == INCHES) {
        if (cm_get_axis_type(nv->index) == AXIS_TYPE_LINEAR) {
            nv->value *= INCHES_PER_MM;
        }
    } 
    set_flt(nv);
    st_cfg.mot[m].units_per_step = 1.0/st_cfg.mot[m].steps_per_unit;

    // Scale TR so all the other values make sense
    // You could scale any one of the other values, but TR makes the most sense
    st_cfg.mot[m].travel_rev = (360.0*st_cfg.mot[m].microsteps)/(st_cfg.mot[m].steps_per_unit*st_cfg.mot[m].step_angle);
    return(STAT_OK);
}

stat_t st_set_pm(nvObj_t *nv)            // set motor power mode
{
    if (nv->value < 0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value >= MOTOR_POWER_MODE_MAX_VALUE) { 
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE); 
    }
    uint8_t motor = _get_motor(nv->index);
    if (motor > MOTORS) {
        nv->valuetype = TYPE_NULL;
        return STAT_INPUT_VALUE_RANGE_ERROR; 
    };

    // We do this *here* in order for this to take effect immediately.
    // setPowerMode() sets the value and also executes it.
    Motors[motor]->setPowerMode((stPowerMode)nv->value);
    return (STAT_OK);
}

stat_t st_get_pm(nvObj_t *nv)            // get motor power mode
{
    uint8_t motor = _get_motor(nv->index);
    if (motor > MOTORS) {
        nv->valuetype = TYPE_NULL;
        return STAT_INPUT_VALUE_RANGE_ERROR; 
    };

    nv->value = (float)Motors[motor]->getPowerMode();
    nv->valuetype = TYPE_INT;
    return (STAT_OK);
}

/*
 * st_set_pl() - set motor power level
 *
 *  Input value may vary from 0.000 to 1.000 The setting is scaled to allowable PWM range.
 *  This function sets both the scaled and dynamic power levels, and applies the
 *  scaled value to the vref.
 */
stat_t st_set_pl(nvObj_t *nv)    // motor power level
{
    if (nv->value < (float)0.0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE); 
    }
    if (nv->value > (float)1.0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    set_flt(nv);    // set power_setting value in the motor config struct (st)

    uint8_t motor = _get_motor(nv->index);
    st_cfg.mot[motor].power_level_scaled = (nv->value * POWER_LEVEL_SCALE_FACTOR);
    st_run.mot[motor].power_level_dynamic = (st_cfg.mot[motor].power_level_scaled);
    Motors[motor]->setPowerLevel(st_cfg.mot[motor].power_level_scaled);

    return(STAT_OK);
}

/*
 * st_get_pwr()	- get current motor power
 *
 *  Returns the current power level of the motor given it's enable/disable state
 *  Returns 0.0 if motor is de-energized or disabled
 *  Can be extended to report idle setback by changing getCurrentPowerLevel()
 */
stat_t st_get_pwr(nvObj_t *nv)
{
    // this is kind of a hack to extract the motor number from the table
    uint8_t motor = (cfgArray[nv->index].token[3] & 0x0F) - 1;
    if (motor > MOTORS) { return STAT_INPUT_VALUE_RANGE_ERROR; };

    nv->value = Motors[motor]->getCurrentPowerLevel(motor);
	nv->valuetype = TYPE_FLOAT;
    nv->precision = cfgArray[nv->index].precision;
	return (STAT_OK);
}

/* GLOBAL FUNCTIONS (SYSTEM LEVEL)
 *
 * st_set_mt() - set global motor timeout in seconds
 * st_set_me() - enable motor power
 * st_set_md() - disable motor power
 *
 * Calling me or md with NULL will enable or disable all motors
 * Setting a value of 0 will enable or disable all motors
 * Setting a value from 1 to MOTORS will enable or disable that motor only
 */

stat_t st_set_mt(nvObj_t *nv)
{
    if (nv->value < MOTOR_TIMEOUT_SECONDS_MIN) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value > MOTOR_TIMEOUT_SECONDS_MAX) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    st_cfg.motor_power_timeout = nv->value;
    return (STAT_OK);
}

// Make sure this function is not part of initialization --> f00
// nv->value is seconds of timeout
stat_t st_set_me(nvObj_t *nv)    
{
    for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
        Motors[motor]->enable(nv->value);   // nv->value is the timeout or 0 for default
    }
    return (STAT_OK);
}

// Make sure this function is not part of initialization --> f00
// nv-value is motor to disable, or 0 for all motors
stat_t st_set_md(nvObj_t *nv)    
{
    if (nv->value < 0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value > MOTORS) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }    
    // de-energize all motors
    if ((uint8_t)nv->value == 0) {      // 0 means all motors
        for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
            Motors[motor]->disable();
        }
    } else {                            // otherwise it's just one motor
         Motors[(uint8_t)nv->value -1]->disable();
    }
    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char msg_units0[] = " in";    // used by generic print functions
static const char msg_units1[] = " mm";
static const char msg_units2[] = " deg";
static const char *const msg_units[] = { msg_units0, msg_units1, msg_units2 };
#define DEGREE_INDEX 2

static const char fmt_me[] = "motors energized\n";
static const char fmt_md[] = "motors de-energized\n";
static const char fmt_mt[] = "[mt]  motor idle timeout%14.2f seconds\n";
static const char fmt_0ma[] = "[%s%s] m%s map to axis%15d [0=X,1=Y,2=Z...]\n";
static const char fmt_0sa[] = "[%s%s] m%s step angle%20.3f%s\n";
static const char fmt_0tr[] = "[%s%s] m%s travel per revolution%10.4f%s\n";
static const char fmt_0mi[] = "[%s%s] m%s microsteps%16d [1,2,4,8,16,32]\n";
static const char fmt_0su[] = "[%s%s] m%s steps per unit %17.5f steps per%s\n";
static const char fmt_0po[] = "[%s%s] m%s polarity%18d [0=normal,1=reverse]\n";
static const char fmt_0pm[] = "[%s%s] m%s power management%10d [0=disabled,1=always on,2=in cycle,3=when moving]\n";
static const char fmt_0pl[] = "[%s%s] m%s motor power level%13.3f [0.000=minimum, 1.000=maximum]\n";
static const char fmt_pwr[] = "[%s%s] Motor %c power level:%12.3f\n";

void st_print_me(nvObj_t *nv) { text_print(nv, fmt_me);}    // TYPE_NULL - message only
void st_print_md(nvObj_t *nv) { text_print(nv, fmt_md);}    // TYPE_NULL - message only
void st_print_mt(nvObj_t *nv) { text_print(nv, fmt_mt);}    // TYPE_FLOAT

static void _print_motor_int(nvObj_t *nv, const char *format)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, (int)nv->value);
    xio_writeline(cs.out_buf);
}

static void _print_motor_flt(nvObj_t *nv, const char *format)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->value);
    xio_writeline(cs.out_buf);
}

static void _print_motor_flt_units(nvObj_t *nv, const char *format, uint8_t units)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->value, GET_TEXT_ITEM(msg_units, units));
    xio_writeline(cs.out_buf);
}

static void _print_motor_pwr(nvObj_t *nv, const char *format)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->token[0], nv->value);
    xio_writeline(cs.out_buf);
}

void st_print_ma(nvObj_t *nv) { _print_motor_int(nv, fmt_0ma);}
void st_print_sa(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0sa, DEGREE_INDEX);}
void st_print_tr(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0tr, cm_get_units_mode(MODEL));}
void st_print_mi(nvObj_t *nv) { _print_motor_int(nv, fmt_0mi);}
void st_print_su(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0su, cm_get_units_mode(MODEL));}
void st_print_po(nvObj_t *nv) { _print_motor_int(nv, fmt_0po);}
void st_print_pm(nvObj_t *nv) { _print_motor_int(nv, fmt_0pm);}
void st_print_pl(nvObj_t *nv) { _print_motor_flt(nv, fmt_0pl);}
void st_print_pwr(nvObj_t *nv){ _print_motor_pwr(nv, fmt_pwr);}

#endif // __TEXT_MODE
