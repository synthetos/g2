/*
 * planner.cpp - Cartesian trajectory planning and motion execution
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 * Copyright (c) 2012 - 2015 Rob Giseburt
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
/* --- Planner Notes ----
 *
 *	The planner works below the canonical machine and above the motor mapping and stepper
 *	execution layers. A rudimentary multitasking capability is implemented for long-running
 *	commands such as lines, arcs, and dwells. These functions are coded as non-blocking
 *	continuations - which are simple state machines that are re-entered multiple times
 *	until a particular operation is complete. These functions have 2 parts - the initial call,
 *	which sets up the local context (closure), and callbacks (continuations) that are called
 *  from the main loop (in controller.c). These tasks only support a single instantiation
 *  and are therefore also not re-entrant - as they rely on singletons for closure.
 *
 *	One important concept is isolation of state at the three layers of the data model -
 *  the Gcode model (gm), motion planner model (bf queue & mm), and motion runtime model (mr).
 *  These are designated as "model", "planner" and "runtime" in function names.
 *
 *	The Gcode model is owned by the canonical machine and should only be accessed by cm_xxxx()
 *	functions. Data from the Gcode model is transferred to the motion planner by the mp_xxx()
 *	functions called by the canonical machine.
 *
 *	The planner should only use data in the planner model. When a move (block) is ready for
 *	execution the relevant data from the planner is transferred to the runtime model,
 *  which should also be isolated.
 *
 *	Models at different levels should never use data from other levels as the data may have
 *	changed or be out-of-sync and lead to unpredictable results.
 */
#include "tinyg2.h"
#include "config.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "kinematics.h"
#include "stepper.h"
#include "encoder.h"
#include "report.h"
#include "util.h"
#include "xio.h"    //+++++ DIAGNOSTIC - only needed if xio_writeline() direct prints are used

// Allocate planner structures
mpBufferPool_t mb;				// move buffer queue
mpMoveMasterSingleton_t mm;		// context for line planning
mpMoveRuntimeSingleton_t mr;	// context for line runtime

// Local Scope Data and Functions
#define _bump(a) ((a<PLANNER_BUFFER_POOL_SIZE-1)?(a+1):0) // buffer incr & wrap
#define spindle_speed move_time	// local alias for spindle_speed to the time variable
#define value_vector gm.target	// alias for vector of values

static void _planner_time_accounting();
static void _audit_buffers();

// Execution routines (NB: These are called from the LO interrupt)
static stat_t _exec_dwell(mpBuf_t *bf);
static stat_t _exec_command(mpBuf_t *bf);

/*
 * planner_init()
 * planner_reset()
 */
void planner_init()
{
// If you know all memory has been zeroed by a hard reset you don't need these next 2 lines
	memset(&mr, 0, sizeof(mr));	            // clear all values, pointers and status
	memset(&mm, 0, sizeof(mm));	            // clear all values, pointers and status
	planner_init_assertions();
	mp_init_buffers();

    // reasonable starting values
    mb.mfo_factor = 1.00;
    mb.planner_critical_time = PLANNER_CRITICAL_TIME;
}

void planner_reset()
{
    planner_init();
}

/*
 * planner_init_assertions()
 * planner_test_assertions() - test assertions, PANIC if violation exists
 */
void planner_init_assertions()
{
	mm.magic_start = MAGICNUM;      // Note: mb magic numbers set up by mp_init_buffers()
	mm.magic_end = MAGICNUM;
	mr.magic_start = MAGICNUM;
	mr.magic_end = MAGICNUM;
}

stat_t planner_test_assertions()
{
    if ((BAD_MAGIC(mm.magic_start)) || (BAD_MAGIC(mm.magic_end)) ||
        (BAD_MAGIC(mb.magic_start)) || (BAD_MAGIC(mb.magic_end)) ||
        (BAD_MAGIC(mr.magic_start)) || (BAD_MAGIC(mr.magic_end))) {
        return(cm_panic(STAT_PLANNER_ASSERTION_FAILURE, "planner_test_assertions()"));
    }
    return (STAT_OK);
}

/*
 * mp_halt_runtime() - stop runtime movement immediately
 */
void mp_halt_runtime()
{
    stepper_reset();                // stop the steppers and dwells
    planner_reset();                // reset the planner queues
}

/*
 * mp_flush_planner() - flush all moves in the planner and all arcs
 *
 *	Does not affect the move currently running in mr.
 *	Does not affect mm or gm model positions
 *	This function is designed to be called during a hold to reset the planner
 *	This function should not generally be called; call cm_queue_flush() instead
 */
void mp_flush_planner()
{
	cm_abort_arc();
	mp_init_buffers();
    mr.move_state = MOVE_OFF;   // invalidate mr buffer to prevent subsequent motion
}

/*
 * mp_set_planner_position() - set planner position for a single axis
 * mp_set_runtime_position() - set runtime position for a single axis
 * mp_set_steps_to_runtime_position() - set encoder counts to the runtime position
 *
 *	Since steps are in motor space you have to run the position vector through inverse
 *	kinematics to get the right numbers. This means that in a non-Cartesian robot changing
 *	any position can result in changes to multiple step values. So this operation is provided
 *	as a single function and always uses the new position vector as an input.
 *
 *	Keeping track of position is complicated by the fact that moves exist in several reference
 *	frames. The scheme to keep this straight is:
 *
 *	 - mm.position	- start and end position for planning
 *	 - mr.position	- current position of runtime segment
 *	 - mr.target	- target position of runtime segment
 *
 *	The runtime keeps a lot more data, such as waypoints, step vectors, etc.
 *  See struct mpMoveRuntimeSingleton for details.
 *
 *	Note that position is set immediately when called and may not be not an accurate representation
 *	of the tool position. The motors are still processing the action and the real tool position is
 *	still close to the starting point.
 */

void mp_set_planner_position(uint8_t axis, const float position) { mm.position[axis] = position; }
void mp_set_runtime_position(uint8_t axis, const float position) { mr.position[axis] = position; }

void mp_set_steps_to_runtime_position()
{
    float step_position[MOTORS];
    kn_inverse_kinematics(mr.position, step_position);      // convert lengths to steps in floating point
    for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
        mr.target_steps[motor] = step_position[motor];
        mr.position_steps[motor] = step_position[motor];
        mr.commanded_steps[motor] = step_position[motor];
        en_set_encoder_steps(motor, step_position[motor]);  // write steps to encoder register
        mr.encoder_steps[motor] = en_read_encoder(motor);

        // These must be zero:
        mr.following_error[motor] = 0;
        st_pre.mot[motor].corrected_steps = 0;
    }
}

/************************************************************************************
 * mp_queue_command() - queue a synchronous Mcode, program control, or other command
 * _exec_command()    - callback to execute command
 *
 *  How this works:
 *    - The command is called by the Gcode interpreter (cm_<command>, e.g. an M code)
 *    - cm_ function calls mp_queue_command which puts it in the planning queue (bf buffer).
 *      This involves setting some parameters and registering a callback to the
 *      execution function in the canonical machine.
 *    - the planning queue gets to the function and calls _exec_command()
 *    - ...which puts a pointer to the bf buffer in the prep struct (st_pre)
 *    - When the runtime gets to the end of the current activity (sending steps, counting a dwell)
 *      if executes mp_runtime_command...
 *    - ...which uses the callback function in the bf and the saved parameters in the vectors
 *    - To finish up mp_runtime_command() needs to free the bf buffer
 *
 *  Doing it this way instead of synchronizing on an empty queue simplifies the
 *  handling of feedholds, feed overrides, buffer flushes, and thread blocking,
 *  and makes keeping the queue full much easier - therefore avoiding Q starvation
 */

void mp_queue_command(void(*cm_exec)(float[], bool[]), float *value, bool *flag)
{
	mpBuf_t *bf;

	// Never supposed to fail as buffer availability was checked upstream in the controller
	if ((bf = mp_get_write_buffer()) == NULL) {
		cm_panic(STAT_FAILED_GET_PLANNER_BUFFER, "mp_queue_command()");
		return;
	}

	bf->move_type = MOVE_TYPE_COMMAND;
	bf->bf_func = _exec_command;      // callback to planner queue exec function
	bf->cm_func = cm_exec;            // callback to canonical machine exec function

	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		bf->value_vector[axis] = value[axis];
		bf->axis_flags[axis] = flag[axis];
	}
	mp_commit_write_buffer(MOVE_TYPE_COMMAND);			// must be final operation before exit
}

static stat_t _exec_command(mpBuf_t *bf)
{
	st_prep_command(bf);
	return (STAT_OK);
}

stat_t mp_runtime_command(mpBuf_t *bf)
{
	bf->cm_func(bf->value_vector, bf->axis_flags);		// 2 vectors used by callbacks
	if (mp_free_run_buffer()) {
		cm_cycle_end();									// free buffer & perform cycle_end if planner is empty
    }
	return (STAT_OK);
}

/*************************************************************************
 * mp_dwell() 	 - queue a dwell
 * _exec_dwell() - dwell execution
 *
 * Dwells are performed by passing a dwell move to the stepper drivers.
 * When the stepper driver sees a dwell it times the dwell on a separate
 * timer than the stepper pulse timer.
 */
stat_t mp_dwell(float seconds)
{
	mpBuf_t *bf;

	if ((bf = mp_get_write_buffer()) == NULL) {			// get write buffer or fail
		return(cm_panic(STAT_FAILED_GET_PLANNER_BUFFER, "mp_dwell()")); // not ever supposed to fail
	}
	bf->bf_func = _exec_dwell;							// register callback to dwell start
	bf->move_time = seconds;					        // in seconds, not minutes
	bf->move_state = MOVE_NEW;
	mp_commit_write_buffer(MOVE_TYPE_DWELL);			// must be final operation before exit
	return (STAT_OK);
}

static stat_t _exec_dwell(mpBuf_t *bf)
{
	st_prep_dwell((uint32_t)(bf->move_time * 1000000.0));// convert seconds to uSec
	if (mp_free_run_buffer()) {
        cm_cycle_end();			     // free buffer & perform cycle_end if planner is empty
    }
	return (STAT_OK);
}

//++++ stubbed ++++
void mp_request_out_of_band_dwell(float seconds)
{
//    mr.out_of_band_dwell_time = seconds;
}
//++++ stubbed ++++
stat_t mp_exec_out_of_band_dwell(void)
{
//    return _advance_dwell(mr.out_of_band_dwell_time);
    return 0;
}

/**********************************************************************************
 * Planner helpers
 *
 * mp_get_planner_buffers()   - return # of available planner buffers
 * mp_planner_is_full()       - true if planner has no room for a new block
 * mp_has_runnable_buffer()   - true if next buffer is runnable, indicating motion has not stopped.
 * mp_is_it_phat_city_time() - test if there is time for non-essential processes
 */
uint8_t mp_get_planner_buffers()
{
    return (mb.buffers_available);
}

bool mp_planner_is_full()
{
    return (mb.buffers_available < PLANNER_BUFFER_HEADROOM);
}

bool mp_has_runnable_buffer()
{
    return (mb.r->buffer_state);    // anything other than MP_BUFFER_EMPTY returns true
}

bool mp_is_phat_city_time()
{
	if(cm.hold_state == FEEDHOLD_HOLD) {
    	return true;
	}
//    return ((mb.time_total <= 0) || (PHAT_CITY_TIME < mb.time_total));
    return ((mb.plannable_time <= 0) || (PHAT_CITY_TIME < mb.plannable_time));
}

/**** Helper functions for mp_plan_buffer()
 * _stop_new_block_timer() - used to disable timer if blocks are not arriving
 * _reset_new_block_timer() - restart new block timer
 * _new_block_timeout() - detect a timeout
 * _set_planner_state() - set planner state optimistic or pessimistic
 */
static void _stop_new_block_timer()
{
    mb.new_block_timer = 0;
}

static void _reset_new_block_timer()
{
    mb.new_block_timer = SysTickTimer.getValue() + NEW_BLOCK_TIMEOUT_MS;
}

static bool _new_block_timeout()
{
    mb.new_block_timeout = false;
    if ((mp_planner_is_full()) || (mb.new_block_timer == 0)) {
        _reset_new_block_timer();
    } else if (mb.new_block_timer < SysTickTimer.getValue()) {
        mb.new_block_timeout = true;
    }
    return (mb.new_block_timeout);
}

/*
 * mp_plan_buffer()
 *
 *  mp_plan_buffer()'s job is to invoke planning intelligently. Items to note:
 *
 *    - At the start of a job the planner should fill up with unplanned blocks before
 *      motion starts. This eliminates an initial move that plans to zero and ensures
 *      the planner gets a "head start" on managing the time in the planner queue.
 *
 *    - The planner should attempt to operate in OPTIMISTIC mode whenever possible, i.e.
 *      It should not plan the last block and assume that the next block will arrive in
 *      time and be a continuation of the movement. When this case is no longer true the
 *      planner switches to PESSIMISTIC mode where all blocks are planned, and the last
 *      block is always planned to zero (the tail). This handles the "last line" case
 *      of a legitimate tail.
 *
 *      Planning optimistically minimizes "overplanning" - i.e. moves planned multiple
 *      times (particularly tails). This requires leaving one unplanned move at the
 *      end (N) of the buffer, so the N-1 block always has a valid exit velocity to
 *      plan to (the entry velocity of the Nth block).
 *
 *	  - An intermediate CAUTIOUS mode is also required if there is insufficient time in
 *		the plan to decelerate from the target velocity to zero. Cautious mode limits the
 *		target velocity so that deceleration is always possible
 *
 *    - It's important to distinguish between the case where the new block is actually a
 *      startup condition and where it's the first block after a stop or a stall. The
 *      planner wants to perform a STARTUP in the first case, but start planning
 *      immediately in the latter cases.
 *
     - A hard case is where a long block is immediately followed by a very short block
 *      at the same velocity, with no more blocks behind them. In this case the first block
 *      will run and  lock, and the second block may have insufficient distance to get to zero.
 *
 *  State Machine (See planner.h/ plannerState)
 *
 *         -------------                 --------------
 *        |             |  all stopped  |              |
 *        |     IDLE    |<--------------| PESSIMISTIC  |
 *        |             |       ------->|   [P]  [N]   |
 *         -------------       /         --------------
 *               |  new       /               ^  |
 *               | block     / timeout    (1) |  | (2)
 *               V          /                 |  V
 *         -------------   /             --------------
 *        |             |--             |              |
 *        |   STARTUP   |-------------->|  OPTIMISTIC  |
 *        |             | planner full  |              |
 *         -------------                 --------------
 *
 *  TRANSITIONS
 *    (1) time in planner is down to critical level or new blocks are not arriving fast enough
 *    (2) time in planner is back in safe region and/or new blocks are arriving fast enough
 *
 *  CRITICAL can be entered in one 2 conditions, or both (sub-states):
 *    [P] plannable_time < PLANNER_TIME_CRITICAL: planned time is dangerously low (about to starve)
 *    [N] new_block_timeout == true: buffer arrival rate is dangerously low (no new blocks)
 */
/*  Has to accurately detect end conditions to go pessimistic:
 *    Case 1: Single block arrives, followed by timeout
 *    Case 2: Two blocks arrive - both short, followed by timeout
 *    Case 3: Two blocks arrive - long followed by a short - w/o timeout
 *    Case 4: Two blocks arrive - long followed by a short - w timeout in between
 *    Case 5: Three blocks arrive - first 2 starts running, then a third block arrives after timeout, Tplan < critical on arrival of #3
 *    Case 6: Three blocks arrive - first 2 starts running, then a third block arrives after timeout, Tplan > critical on arrival of #3
 *    Case 7: Sequence of blocks plans optimistically, timeout, followed by a Nth block. Tplan < critical
 *    Case 8: Sequence of blocks plans optimistically, timeout, followed by a Nth block. Tplan > critical
 */
/*
 *	CAUTIOUS PLANNING occurs when the time in the planner is insufficient to plan to zero
 *	from the target velocity (Vt). Vt must be limited so that a deceleration to zero is
 *	possible in the available time.
 *
 *	Terms
 *	  - Head - blocks are removed from the head of the planner queue - inserted in the tail
 *	  - Running block - the block that is currently running (at the head)
 *    - Ondeck block - the next block to run. If nothing is running it's the head block N,
 *                     otherwise it's the block behind the running (head) block, N+1
 *
 *  Factors influencing cautious mode include:
 *	  - Vtarget (Vt) - the requested velocity of a block
 *	  - Vrun (Vr)    - the velocity of the running block
 *    - Vondeck (Vo) - the velocity of the ondeck block
 *	  - To-plan      - time in the planner measured from the ondeck block
 *    - Trun         - time remaining in the runtime (estimated)
 *    - Ttimeout     - block timeout time - e.g. 30 ms
 *    - Treplan      - time required to replan the queue (worst case, e.g. 10 ms)
 *	  - T_-decel     - time required to decelerate from a velocity to zero (Vt, Vr, Vo)
 *    - L_-decel     - the distance (length) required to decelerate from Vt, Vr, Vo
 *
 *  Lemma1: The following equality must always be true: To-decel < (To-plan - Ttimeout - Treplan)
 *          This requires Vo to be limited so this is always true.
 *
 *  Lemma2: Vo must equal the exit velocity of the run block
 */
stat_t mp_planner_callback()
{
    if (!mb.request_planning) {
        if ((cm.motion_state == MOTION_STOP) && (cm.hold_state == FEEDHOLD_OFF) &&
            (mb.buffers_available == PLANNER_BUFFER_POOL_SIZE)) {
            mb.planner_state = PLANNER_IDLE;
        }
        if ((mb.planner_state == PLANNER_PESSIMISTIC) && (!mb.new_block)) { // shortcut out of here
            return (STAT_NOOP);
        }
        if (mb.planner_state == PLANNER_IDLE) {
            if (!mb.new_block) {
                _stop_new_block_timer();
                return (STAT_NOOP);
            }
            mb.p = mb.r;                                // initialize planner pointer to run buffer
            mb.planner_state = PLANNER_STARTUP;
        }
    } else {
        mb.request_planning = false;
    }
    if (mb.new_block) {
        _reset_new_block_timer();
        mb.new_block = false;
    }

    // set planner state
    if (mb.planner_state == PLANNER_STARTUP) {          // set planner state for startup operation
        if (mp_planner_is_full()) {
//            mb.planner_state = PLANNER_OPTIMISTIC;      // start planning now
            mb.planner_state = PLANNER_PESSIMISTIC;      // start planning now
        } else if (_new_block_timeout()) {
            mb.planner_state = PLANNER_PESSIMISTIC;     // start planning now
        } else {
            return (STAT_OK);                           // accumulate new blocks until it's time to plan
        }
    } else {                                            // set planner state for normal operation
//        if (_new_block_timeout() || mb.plannable_time < PLANNER_CRITICAL_TIME) {
        if (_new_block_timeout() || mb.plannable_time < mb.planner_critical_time) {
            mb.planner_state = PLANNER_PESSIMISTIC;
        } else {
//            mb.planner_state = PLANNER_OPTIMISTIC;
            mb.planner_state = PLANNER_PESSIMISTIC;
        }
    }
    if ((mb.planner_state == PLANNER_OPTIMISTIC) &&     // skip last block if optimistic
        (mb.p->nx->buffer_state == MP_BUFFER_EMPTY)) {
        return (STAT_OK);
    }
    if (mb.p->buffer_state == MP_BUFFER_EMPTY) {          // unconditional exit condition
        return (STAT_OK);
    }
    mp_plan_block_list();                               // plan blocks optimistically or pessimistically
    return (STAT_OK);
}

/*
 *  mp_replan_queue() - reset the blocks in the planner queue and request a planner run
 */
void mp_replan_queue(mpBuf_t *bf)
{
    mb.p = bf;    // reset planner pointer to start replan from here

    do {
        if (bf->buffer_state == MP_BUFFER_EMPTY) {
            break;
        }
        bf->head_length = 0;
        bf->body_length = 0;
        bf->tail_length = 0;
        bf->head_time = 0;
        bf->body_time = 0;
        bf->tail_time = 0;
        bf->buffer_state = MP_BUFFER_NOT_PLANNED;
    } while ((bf = mp_get_next_buffer(bf)) != mb.p);

    mb.request_planning = true;
}

/*
 *  mp_start_feed_override() - gradually adjust existing and new buffers to target override percentage
 *  mp_end_feed_override() - gradually adjust existing and new buffers to no override percentage
 *
 *  Variables:
 *    - 'mfo_factor' is the override scaling factor normalized to 1.0 = 100%
 *      Values < 1.0 are speed decreases, > 1.0 are increases. Upper and lower limits are checked.
 *
 *    - 'ramp_time' is approximate, as the ramp dynamically changes move execution times
 *      The ramp will attempt to meet the time specified but it will not be exact.
 */
/*  Function:
 *  The override takes effect as close to real-time as possible. Practically, this mans about 20 or
 *  so behind the current running move. How it works:
 *
 *    - If the planner is idle just apply the override factor and be done with it. That's easy.
 *
 *    - Otherwise look for the "break point" at 20 ms.
 *
 *
 */

void mp_start_feed_override(const float ramp_time, const float mfo_factor)
{
    cm.mfo_state = MFO_REQUESTED;

    if (mb.planner_state == PLANNER_IDLE) {
        mb.mfo_factor = mfo_factor;             // that was easy
        return;
    }

    // Assume that the min and max values for override_factor have been validated upstream
    // SUVAT: V = U+AT ==> A = (V-U)/T
    mb.ramp_target = mfo_factor;
//    mb.ramp_dvdt = (mfo_factor - mb.current_mfo_factor) / ramp_time;
    mb.ramp_dvdt = (mfo_factor - mb.c->mfo_factor) / ramp_time;
    mb.mfo_active = true;

    if (fp_NOT_ZERO(mb.ramp_dvdt)) {    // do these things only if you actually have a ramp to run
        mb.p = mb.c;                    // re-position the planner pointer
        mb.ramp_active = true;
        mb.request_planning = true;
    }
}

void mp_end_feed_override(const float ramp_time)
{
    mp_start_feed_override (FEED_OVERRIDE_RAMP_TIME, 1.00);
}

/*
 * _planner_time_accounting() - gather time in planner and runtime
 *
 *  Record the move times in the runtime and planner for use in planning decisions
 */

static void _planner_time_accounting()
{
    // get run buffer and see if anything is running
    if (mb.r->buffer_state == MP_BUFFER_EMPTY || mb.r->buffer_state == MP_BUFFER_NOT_PLANNED) {
        mb.plannable_time = 0;
        return;
    }

    float plannable_time = 0;
    bool in_critical = true;                        // used to look for transition to critical region

    mpBuf_t *bf = mb.r;
    bf->plannable_time_ms = plannable_time;         // ++++++ which is 0 at this point

    // Step through the moves and add up the planner time
    while ((bf = mp_get_next_buffer(bf)) != mb.r) {
//        if (bf->buffer_state == MP_BUFFER_EMPTY) {
//            break;
//        }
        plannable_time += bf->move_time;              // total planner time, w/est's for non-planned blocks
        if (bf->buffer_state == MP_BUFFER_PLANNED) {
//          plannable_time += bf->move_time;
//            if (in_critical && (plannable_time >= PLANNER_CRITICAL_TIME)) {
            if (in_critical && (plannable_time >= mb.planner_critical_time)) {
                in_critical = false;
                mb.c = bf;                          // mark the first non-critical block
            }
            bf->plannable_time_ms = plannable_time * 60000; //+++++ DIAGNOSTIC
            continue;
        }
        break;
    }
    mb.plannable_time = plannable_time;

    //+++++ DIAGNOSTIC
    mb.plannable_time_ms = plannable_time * 60000;
    mb.run_time_remaining_ms = mb.run_time_remaining_ms * 60000;
}

/**** PLANNER BUFFER PRIMITIVES ************************************************************
 *
 *	Planner buffers are used to queue and operate on Gcode blocks. Each buffer contains
 *	one Gcode block which may be a move, an M code, or other command that must be
 *	executed synchronously with movement.
 *
 *	The planner queue (mb) is a circular queue of planner buffers (bf's). Each block has a
 *  pointer to the next block (nx), and one to the previous block (pv).
 *
 *  At the risk of being pedantic it's useful to get terms straight or it can get confusing.
 *
 *    - The "run" block is the block that is currently executing (i.e. in mr). Since
 *      it's a circular FIFO queue the running block is considered the "first block".
 *
 *    - The "write" block (aka "new" block) is the block that was just put on the queue.
 *      The new block is at the other end of the queue from the run block.
 *
 *    - Moving "forward" is advancing to the next block (nx), which is in the direction
 *      of the new block. Moving "backwards" backs up to the previous block (pv) in the
 *      direction of the running block. Since the queue is a doubly linked circular list
 *      the ends connect, and blocks "outside" of the running and new blocks may be empty.
 *
 *    - The "planning" block is the block currently pointed to by the planner. This
 *      starts out right next to the running block and advances towards the new block
 *      as planning executes. Planning executes predominantly in the forward direction.
 *
 *	New blocks are populated by (1) getting a write buffer, (2) populating the buffer,
 *	then (3) placing it in the queue (commit write buffer). If an exception occurs
 *	during step (2) you can unget the write buffer before queuing it, which returns
 *	it to the pool of available buffers. (NB: Unget is currently unused but left in.)
 *
 *	The RUN buffer may be retrieved once for simple commands, or multiple times for
 *  long-running commands such as moves that get called multiple times. The first
 *  retrieval (get run buffer) will return the new run buffer. Subsequent retrievals
 *  will return the same buffer until it's state changes to complete. When the command
 *  is complete the run buffer is returned to the pool by freeing it.
 *
 * Notes:
 *	The write buffer pointer only moves forward on mp_commit_write_buffer, and the
 *  run buffer pointer only moves forward on mp_free_run_buffer().
 *	Tests, gets and unget have no effect on the pointers.
 *
 * Functions Provided:
 * _clear_buffer(bf)        Zero the contents of a buffer
 *
 * mp_init_buffers()        Initialize or reset buffers
 *
 * mp_get_prev_buffer(bf)   Return pointer to the previous buffer in the linked list
 * mp_get_next_buffer(bf)   Return pointer to the next buffer in the linked list
 *
 * mp_get_write_buffer()    Get pointer to next available write buffer
 *                          Return pointer or NULL if no buffer available.
 *
 * mp_commit_write_buffer()	Commit the write buffer to the queue.
 *                          Advance write pointer & changes buffer state.
 *
 *                          *** WARNING *** The calling routine must NOT use the write
 *                          buffer once it has been committed as it may be processed
 *                          and freed (cleared) before the commit function returns.
 *
 * mp_get_run_buffer()      Get pointer to the next or current run buffer.
 *                          Return a new run buffer if prev buf was ENDed.
 *                          Return same buf if called again before ENDing.
 *                          Return NULL if no buffer available.
 *                          This behavior supports continuations (iteration).
 *
 * mp_free_run_buffer()     Release the run buffer & return to buffer pool.
 *                          Return true if queue is empty, false otherwise.
 *                          This is useful for doing queue empty / end move functions.
 * *
 * UNUSED BUT PROVIDED FOR REFERENCE:
 * mp_unget_write_buffer()  Free write buffer if you decide not to commit it.
 * mp_copy_buffer(bf,bp)	Copy the contents of bp into bf - preserves links.
 */

static inline void _clear_buffer(mpBuf_t *bf)
{
	// Note: bf->bf_func is the first address we wish to clear as we must preserve
    // the pointers and buffer number during interrupts

    // GCC did not like this line, so it's done differently
//	memset((void *)(&bf->bf_func), 0, sizeof(mpBuf_t) - (sizeof(void *) * 2) - sizeof(uint16_t));

    uint8_t buffer_number = bf->buffer_number;    //+++++ DIAGNOSTIC
	memset((void *)(&bf->bf_func), 0, sizeof(mpBuf_t) - (sizeof(void *) * 2));
    bf->buffer_number = buffer_number;            //+++++ DIAGNOSTIC
}

void mp_init_buffers(void)
{
	mpBuf_t *pv;
	uint8_t i;

	memset(&mb, 0, sizeof(mb));                     // clear all values, pointers and status
	mb.magic_start = MAGICNUM;
	mb.magic_end = MAGICNUM;

	mb.w = &mb.bf[0];                               // init all buffer pointers
	mb.r = &mb.bf[0];
	mb.p = &mb.bf[0];
	mb.c = &mb.bf[0];
	pv = &mb.bf[PLANNER_BUFFER_POOL_SIZE-1];
	for (i=0; i < PLANNER_BUFFER_POOL_SIZE; i++) {
        mb.bf[i].buffer_number = i;                 //+++++ number it for diagnostics only (otherwise not used)
		mb.bf[i].nx = &mb.bf[_bump(i)];
		mb.bf[i].pv = pv;                           // setup ring pointers
		pv = &mb.bf[i];
	}
	mb.buffers_available = PLANNER_BUFFER_POOL_SIZE;
}

/*
 * These GET functions are defined here but we use the macros in planner.h instead
 *
mpBuf_t * mp_get_prev_buffer(const mpBuf_t *bf)
{
    return (bf->pv);
}
mpBuf_t * mp_get_next_buffer(const mpBuf_t *bf)
{
    return (bf->nx);
}
*/

mpBuf_t * mp_get_write_buffer()     // get & clear a buffer
{
    if (mb.w->buffer_state == MP_BUFFER_EMPTY) {
        _clear_buffer(mb.w);
        mb.w->buffer_state = MP_BUFFER_NOT_PLANNED;
        mb.buffers_available--;
        return (mb.w);
    }
    // The no buffer condition always causes a panic - invoked by the caller
	rpt_exception(STAT_FAILED_TO_GET_PLANNER_BUFFER, "mp_get_write_buffer()");
	return (NULL);
}

/*** WARNING ***
* The function calling mp_commit_write_buffer() must NOT use the write buffer once it has
* been committed. Interrupts may use the buffer immediately, invalidating its contents.
*/

void mp_commit_write_buffer(const moveType move_type)
{
    mb.w->move_type = move_type;
    mb.w->move_state = MOVE_NEW;

    if (move_type == MOVE_TYPE_ALINE) {
        if (cm.motion_state == MOTION_STOP) {
            cm_set_motion_state(MOTION_PLANNING);
        }
    } else {
        if ((mb.planner_state > PLANNER_STARTUP) && (cm.hold_state == FEEDHOLD_OFF)) {
            // NB: BEWARE! the exec may result in the planner buffer being
            // processed IMMEDIATELY and then freed - invalidating the contents
            st_request_exec_move();	 // requests an exec if the runtime is not busy
        }
    }
    mb.new_block = true;				// got a new block to plan
    mb.w = mb.w->nx;                    // advance the write buffer pointer
    qr_request_queue_report(+1);        // request a QR and add to the "added buffers" count
}

// Note: mp_get_run_buffer() is only called by mp_exec_move(), which is within an interrupt
mpBuf_t * mp_get_run_buffer()
{
    // CASE: fresh buffer; becomes running if buffer planned
    if (mb.r->buffer_state == MP_BUFFER_PLANNED) {
        mb.r->buffer_state = MP_BUFFER_RUNNING;
    }

    // This is the one point where an accurate accounting of the total time in the
    // run and the planner is established. _plannable_time_accounting() also performs
    // the locking of planner buffers to ensure that sufficient "safe" time is reserved.
    _planner_time_accounting();

    // CASE: asking for the same run buffer for the Nth time
    if (mb.r->buffer_state == MP_BUFFER_RUNNING) {
        return (mb.r);					// return same buffer
    }
    return (NULL);						// CASE: no queued buffers. fail it.
}

// Note: mp_free_run_buffer() is only called from mp_exec_XXX, which are within an interrupt
bool mp_free_run_buffer()   // EMPTY current run buffer & advance to the next
{
    _audit_buffers();       // diagnostic audit for buffer chain integrity (only runs in DEBUG mode)

    mpBuf_t *r = mb.r;
    mb.r = mb.r->nx;					// advance to next run buffer

	_clear_buffer(r);                   // clear it out (& reset plannable and set MP_BUFFER_EMPTY)
//    r->buffer_state = MP_BUFFER_EMPTY;  // do a "light clear"
//    r->move_time = 0;

	mb.buffers_available++;
	qr_request_queue_report(-1);			// request a QR and add to the "removed buffers" count
	return (mb.w == mb.r); 	            // return true if the queue emptied
}

/* UNUSED FUNCTIONS - left in for completeness and for reference
void mp_unget_write_buffer()
{
    mb.w = mb.w->pv;						// queued --> write
    mb.w->buffer_state = MP_BUFFER_EMPTY; // not loading anymore
    mb.buffers_available++;
}

void mp_copy_buffer(mpBuf_t *bf, const mpBuf_t *bp)
{
    // copy contents of bp to by while preserving pointers in bp
    memcpy((void *)(&bf->bf_func), (&bp->bf_func), sizeof(mpBuf_t) - (sizeof(void *) * 2));
}
*/

/************************************************************************************
 * _planner_report()
 * _audit_buffers() - a DEBUG diagnostic
 */

#if 0
#ifdef DEBUG

#warning DEBUG TRAPS ENABLED


#pragma GCC optimize ("O0")


static void _planner_report(const char *msg)
{
    rpt_exception(STAT_PLANNER_ASSERTION_FAILURE, msg);

    for (uint8_t i=0; i<PLANNER_BUFFER_POOL_SIZE; i++) {
        printf("{\"er\":{\"stat\":%d, \"type\":%d, \"lock\":%d, \"plannable\":%d",
                mb.bf[i].buffer_state,
                mb.bf[i].move_type,
                mb.bf[i].locked,
                mb.bf[i].plannable);
        if (&mb.bf[i] == mb.r) {
            printf(", \"RUN\":t");}
        if (&mb.bf[i] == mb.w) {
            printf(", \"WRT\":t");}
        printf("}}\n");
    }
}

/*
 * _audit_buffers() - diagnostic to determine if buffers are sane
 */

static void _audit_buffers()
{
    __disable_irq();

    // Current buffer should be in the running state.
    if (mb.r->buffer_state != MP_BUFFER_RUNNING) {
        _planner_report("buffer audit1");
        _debug_trap();
    }

    // Check that the next from the previous is correct.
    if (mb.r->pv->nx != mb.r || mb.r->nx->pv != mb.r){
        _planner_report("buffer audit2");
        _debug_trap();
    }

    // Now check every buffer, in order we would execute them.
    mpBuf_t *bf = mb.r->nx;
    while (bf != mb.r) {
        // Check that the next from the previous is correct.
        if (bf->pv->nx != bf || bf->nx->pv != bf){
            _planner_report("buffer audit3");
            _debug_trap();
        }

        // Order should be:
        //  - MP_BUFFER_RUNNING
        //  - MP_BUFFER_PLANNED (zero or more)
        //  - MP_BUFFER_NOT_PLANNED (zero or more)
        //  - MP_BUFFER_EMPTY (zero or more up until mb.r)
        //  - no more

        // After RUNNING, we can see anything but PENDING, but prefer not to find PLANNING
        if (bf->pv->buffer_state == MP_BUFFER_RUNNING &&
                                    bf->buffer_state != MP_BUFFER_PLANNED &&
                                    bf->buffer_state != MP_BUFFER_EMPTY) {
            // Exception: PLANNING is allowed, but we may want to watch for it:
            if (bf->buffer_state == MP_BUFFER_NOT_PLANNED) {
                __NOP();
            } else {
                _planner_report("buffer audit4");
                _debug_trap();
            }
        }

        // After QUEUED, we can see QUEUED, PLANNING, or EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_PLANNED &&
                                    bf->buffer_state != MP_BUFFER_PLANNED &&
                                    bf->buffer_state != MP_BUFFER_NOT_PLANNED &&
                                    bf->buffer_state != MP_BUFFER_EMPTY) {
            _planner_report("buffer audit5");
            _debug_trap();
        }

        // After PLANNING, we can see PLANNING, or EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_NOT_PLANNED &&
                                    bf->buffer_state != MP_BUFFER_NOT_PLANNED &&
                                    bf->buffer_state != MP_BUFFER_PLANNED &&
                                    bf->buffer_state != MP_BUFFER_EMPTY) {
            _planner_report("buffer audit6");
            _debug_trap();
        }

        // After EMPTY, we should only see EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_EMPTY && bf->buffer_state != MP_BUFFER_EMPTY) {
            _planner_report("buffer audit7");
            _debug_trap();
        }
        // Now look at the next one.
        bf = bf->nx;
    }
    __enable_irq();
}

#pragma GCC reset_options

#endif // DEBUG

#else

static void _audit_buffers()
{
    // empty stub
}

#endif // 0


/****************************
 * END OF PLANNER FUNCTIONS *
 ****************************/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/
