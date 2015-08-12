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

using namespace Motate;
//extern OutputPin<kDebug1_PinNumber> plan_debug_pin1;
//extern OutputPin<kDebug2_PinNumber> plan_debug_pin2;
//extern OutputPin<kDebug3_PinNumber> plan_debug_pin3;
//extern OutputPin<kDebug4_PinNumber> plan_debug_pin4;

//extern OutputPin<-1> plan_debug_pin1;
//extern OutputPin<-1> plan_debug_pin2;
//extern OutputPin<-1> plan_debug_pin3;
//extern OutputPin<-1> plan_debug_pin4;


// Allocate planner structures

mpBufferPool_t mb;				// move buffer queue
mpMoveMasterSingleton_t mm;		// context for line planning
mpMoveRuntimeSingleton_t mr;	// context for line runtime

/*
 * Local Scope Data and Functions
 */
#define _bump(a) ((a<PLANNER_BUFFER_POOL_SIZE-1)?(a+1):0) // buffer incr & wrap
#define spindle_speed move_time	// local alias for spindle_speed to the time variable
#define value_vector gm.target	// alias for vector of values
//#define flag_vector unit		// alias for vector of flags

static void _planner_time_accounting();
static void _audit_buffers();

// execution routines (NB: These are called from the LO interrupt)
static stat_t _exec_dwell(mpBuf_t *bf);
static stat_t _exec_command(mpBuf_t *bf);

/*
 * planner_init()
 * planner_reset()
 */
void planner_init()
{
// If you know all memory has been zeroed by a hard reset you don't need these next 2 lines
	memset(&mr, 0, sizeof(mr));	// clear all values, pointers and status
	memset(&mm, 0, sizeof(mm));	// clear all values, pointers and status
	planner_init_assertions();
	mp_init_buffers();
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
        return(cm_panic(STAT_PLANNER_ASSERTION_FAILURE, "mp  magic numbers"));
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
		cm_panic(STAT_BUFFER_FULL_FATAL, "no write buffer in mp_queue_command");
		return;
	}

	bf->move_type = MOVE_TYPE_COMMAND;
	bf->bf_func = _exec_command;      // callback to planner queue exec function
	bf->cm_func = cm_exec;            // callback to canonical machine exec function
    bf->replannable = true;           // allow the normal planning to go backward past this zero-speed and zero-length "move"

	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		bf->value_vector[axis] = value[axis];
		bf->flag_vector[axis] = flag[axis];
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
	bf->cm_func(bf->value_vector, bf->flag_vector);		// 2 vectors used by callbacks
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
		return(cm_panic(STAT_BUFFER_FULL_FATAL, "no write buffer in mp_dwell")); // not ever supposed to fail
	}
	bf->bf_func = _exec_dwell;							// register callback to dwell start
    bf->replannable = true;  // +++ TEST allow the normal planning to go backward past this zero-speed and zero-length "move"
	bf->gm.move_time = seconds;							// in seconds, not minutes
	bf->move_state = MOVE_NEW;
	mp_commit_write_buffer(MOVE_TYPE_DWELL);			// must be final operation before exit
	return (STAT_OK);
}

static stat_t _exec_dwell(mpBuf_t *bf)
{
	st_prep_dwell((uint32_t)(bf->gm.move_time * 1000000.0));// convert seconds to uSec
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

/**** PLANNER BUFFER PRIMITIVES ************************************************************
 *
 *	Planner buffers are used to queue and operate on Gcode blocks. Each buffer contains
 *	one Gcode block which may be a move, and M code, or other command that must be
 *	executed synchronously with movement.
 *
 *	Buffers are in a circularly linked list managed by a WRITE pointer and a RUN pointer.
 *	New blocks are populated by (1) getting a write buffer, (2) populating the buffer,
 *	then (3) placing it in the queue (commit write buffer). If an exception occurs
 *	during step (2) you can unget the write buffer before queuing it, which returns
 *	it to the pool of available buffers. (NB: Unget is currently unused be left in)
 *
 *	The RUN buffer is the buffer currently executing. It may be retrieved once for
 *	simple commands, or multiple times for long-running commands like moves. The
 *  first retrieval (get run buffer) will return the new run buffer. Subsequent
 *  retrievals will return the same buffer until it's state changes to complete.
 *  When the command is complete the run buffer is returned to the pool by freeing it.
 *
 * Notes:
 *	The write buffer pointer only moves forward on mp_commit_write_buffer,
 *  and the run buffer pointer only moves forward on mp_free_run_buffer().
 *	Tests, gets and unget have no effect on the pointers.
 *
 * _clear_buffer(bf)        Zero the contents of the buffer
 *
 * mp_init_buffers()        Initialize or reset buffers
 *
 * mp_get_planner_buffers_available() Return # of available planner buffers
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
 * mp_has_runnable_buffer() Check to see if the next buffer is runnable, indicating that
 *                          we have not stopped.
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
 *
 * mp_get_prev_buffer(bf)   Return pointer to the previous buffer in the linked list
 * mp_get_next_buffer(bf)   Return pointer to the next buffer in the linked list
 * mp_get_first_buffer(bf)	Return pointer to first buffer, i.e. the running block
 *
 * UNUSED
 * mp_unget_write_buffer()  Free write buffer if you decide not to commit it.
 * mp_get_last_buffer(bf)	Return pointer to last buffer, i.e. last block.
 * mp_copy_buffer(bf,bp)	Copy the contents of bp into bf - preserves links.
 */

static inline void _clear_buffer(mpBuf_t *bf)
{
	// Note: bf->bf_func is the first address we wish to clear as
    // we must preserve the integrity of the pointers during interrupts
	memset((void *)(&bf->bf_func), 0, sizeof(mpBuf_t) - (sizeof(void *) * 2));
}

void mp_init_buffers(void)
{
	mpBuf_t *pv;
	uint8_t i;

	memset(&mb, 0, sizeof(mb));     // clear all values, pointers and status
	mb.magic_start = MAGICNUM;
	mb.magic_end = MAGICNUM;

	mb.w = &mb.bf[0];               // init write and read buffer pointers
	mb.q = &mb.bf[0];
	mb.r = &mb.bf[0];
	pv = &mb.bf[PLANNER_BUFFER_POOL_SIZE-1];
	for (i=0; i < PLANNER_BUFFER_POOL_SIZE; i++) { // setup ring pointers
		mb.bf[i].nx = &mb.bf[_bump(i)];
		mb.bf[i].pv = pv;
		pv = &mb.bf[i];
	}
	mb.buffers_available = PLANNER_BUFFER_POOL_SIZE;
}

uint8_t mp_get_planner_buffers_available(void)
{
	return (mb.buffers_available);
}

mpBuf_t * mp_get_write_buffer()     // get & clear a buffer
{
    if (mb.w->buffer_state == MP_BUFFER_EMPTY) {
        mpBuf_t *w = mb.w;
        mb.w = mb.w->nx;
        _clear_buffer(w);
        w->buffer_state = MP_BUFFER_PLANNING;
        mb.buffers_available--;
        return (w);
    }
	rpt_exception(STAT_FAILED_TO_GET_PLANNER_BUFFER, "mp_get_write_buffer");
	return (NULL);
}

/*** WARNING ***
* The function calling mp_commit_write_buffer() must NOT use the write buffer once it has
* been committed. Interrupts may use the buffer immediately, invalidating its contents.
*/
void mp_commit_write_buffer(const moveType move_type)
{
    mb.q->move_type = move_type;
    mb.q->move_state = MOVE_NEW;
//    mb.q->replannable = true;                   // ++++ TEST
    if (MOVE_TYPE_ALINE != move_type) {
        mb.q->buffer_state = MP_BUFFER_QUEUED;
        mb.q = mb.q->nx;
        if (!mb.needs_replanned) {
            if (cm.hold_state != FEEDHOLD_HOLD)
//            if ((cm.hold_state != FEEDHOLD_HOLD) && (cm.hold_state != FEEDHOLD_DECEL_FINALIZE))
                st_request_exec_move();	        // requests an exec if the runtime is not busy
                // NB: BEWARE! the exec may result in the planner buffer being
                // processed IMMEDIATELY and then freed - invalidating the contents
        }
    } else {
        mb.needs_replanned = true;
        if(cm.hold_state == FEEDHOLD_OFF)
            cm_set_motion_state(MOTION_PLANNING);
        mb.q = mb.q->nx;                        // advance the queued buffer pointer
        if (mb.planner_timer == 0) {
            mb.planner_timer = SysTickTimer.getValue() + PLANNER_TIMEOUT_MS;
        }
    }
    qr_request_queue_report(+1);                // request a QR and add to the "added buffers" count
}

bool mp_has_runnable_buffer()
{
    return (mb.r->buffer_state);                // anything other than MP_BUFFER_EMPTY returns true
}

mpBuf_t * mp_get_run_buffer()
{
    // CASE: fresh buffer; becomes running if queued or pending
    if (mb.r->buffer_state == MP_BUFFER_QUEUED) {
        mb.r->buffer_state = MP_BUFFER_RUNNING;
        mb.needs_time_accounting = true;
    }

    // This is the one point where an accurate accounting of the total time in the
    // run and the planner is established. _planner_time_accounting() also performs
    // the locking of planner buffers to ensure that sufficient "safe" time is reserved.
    _planner_time_accounting();

    // CASE: asking for the same run buffer for the Nth time
    if (mb.r->buffer_state == MP_BUFFER_RUNNING) {
        return (mb.r);                          // return same buffer
    }
    return (NULL);								// CASE: no queued buffers. fail it.
}

bool mp_free_run_buffer()    // EMPTY current run buffer & advance to the next
{
    _audit_buffers();           // diagnostic audit for buffer chain integrity

    mb.needs_time_accounting = true;

    mpBuf_t *r = mb.r;
    mb.r = mb.r->nx;                            // advance to next run buffer
	_clear_buffer(r);                           // clear it out (& reset replannable and set MP_BUFFER_EMPTY)
//	if (mb.r->buffer_state == MP_BUFFER_QUEUED) {// only if queued...
//		mb.r->buffer_state = MP_BUFFER_RUNNING; // run next buffer
////    } else {
////        __NOP(); // something to get ahold of in debugging - gets here when queue empties
//	}
	mb.buffers_available++;
	qr_request_queue_report(-1);				// request a QR and add to the "removed buffers" count
	return ((mb.w == mb.r) ? true : false); 	// return true if the queue emptied
}

/* These functions are defined here, but use the macros in planner.h instead.
mpBuf_t * mp_get_prev_buffer(const mpBuf_t *bf) return (bf->pv);
mpBuf_t * mp_get_next_buffer(const mpBuf_t *bf) return (bf->nx);
*/

mpBuf_t * mp_get_first_buffer(void) {
    if (mb.r->buffer_state == MP_BUFFER_QUEUED || mb.r->buffer_state == MP_BUFFER_RUNNING) {
        return mb.r;
    }
    return NULL;
}

/* UNUSED FUNCTIONS - left in for completeness and for reference
void mp_unget_write_buffer()
{
    mb.w = mb.w->pv;                            // queued --> write
    mb.w->buffer_state = MP_BUFFER_EMPTY;       // not loading anymore
    mb.buffers_available++;
}

mpBuf_t * mp_get_last_buffer(void)
{
	mpBuf_t *bf = mp_get_run_buffer();
	mpBuf_t *bp = bf;

	if (bf == NULL) return(NULL);

	do {
		if ((bp->nx->move_state == MOVE_OFF) || (bp->nx == bf)) {
			return (bp);
		}
	} while ((bp = mp_get_next_buffer(bp)) != bf);
	return (bp);
}

void mp_copy_buffer(mpBuf_t *bf, const mpBuf_t *bp)
{
    // copy contents of bp to by while preserving pointers in bp
    memcpy((void *)(&bf->bf_func), (&bp->bf_func), sizeof(mpBuf_t) - (sizeof(void *) * 2));
}
*/

/*
 * Planner functions and helpers
 *
 *	mp_plan_buffer()
 *	mp_is_it_phat_city_time()
 *	_planner_time_accounting()
 *  _audit_buffers()
 */

stat_t mp_plan_buffer()
{
    // Criteria to replan:
    // 0) There are items in the buffer that need replanning.
    // 1) Planner timer has "timed out"
    // 2) Less than MIN_PLANNED_TIME in the planner

    if (!mb.needs_replanned) {
        return (STAT_OK);
    }
    bool do_continue = false;

    if (mb.force_replan) {
        do_continue = true;
        mb.force_replan = false;
    }

    if (!do_continue && (mb.planner_timer < SysTickTimer.getValue()) ) {
        do_continue = true;
    }

    float total_buffer_time = mb.time_in_run + mb.time_in_planner;
    if (!do_continue && (total_buffer_time > 0) && (MIN_PLANNED_TIME >= total_buffer_time) ) {
        do_continue = true;
    }

    if (!do_continue) {
        return (STAT_OK);
    }

    // Now, finally, plan the buffer.
    mp_plan_block_list(mb.q->pv);

    if (cm.hold_state != FEEDHOLD_HOLD) {
        st_request_exec_move();					// requests an exec if the runtime is not busy
        // NB: BEWARE! the exec may result in the planner buffer being
        // processed immediately and then freed - invalidating the contents
    }

    mb.planner_timer = 0; // clear the planner timer
    mb.needs_replanned = false;
    return (STAT_OK);
}

bool mp_is_it_phat_city_time() {

	if(cm.hold_state == FEEDHOLD_HOLD) {
    	return true;
	}
    float time_in_planner = mb.time_in_run + mb.time_in_planner;
    return ((time_in_planner <= 0) || (PHAT_CITY_TIME < time_in_planner));
}

static void _planner_time_accounting()
{
//    if (((mb.time_in_run + mb.time_locked) > MIN_PLANNED_TIME) && !mb.needs_time_accounting)
//        return;

    mpBuf_t *bf = mp_get_first_buffer();  // potential to return a NULL buffer
    mpBuf_t *bp = bf;

    if (bf == NULL) {
        mb.time_in_planner = 0;
        return;
    }

    float time_in_planner = mb.time_in_run; // start with how much time is left in the runtime

    // Now step through the moves and add up the planner time, locking up until MIN_PLANNED_TIME
    while ((bp = mp_get_next_buffer(bp)) != bf && bp != mb.q) {
        if (bp->buffer_state == MP_BUFFER_QUEUED) {
            if (!bp->locked) {
                if (time_in_planner < MIN_PLANNED_TIME) {
                    bp->locked = true;
                }
            } // !locked

            // move on, it's already locked
            time_in_planner += bp->real_move_time;

        } else {
            break;
        }
    };
    mb.time_in_planner = time_in_planner;
}

#if 0
#ifdef DEBUG

#warning DEBUG TRAPS ENABLED


#pragma GCC optimize ("O0")


static void _planner_report(const char *msg)
{
    rpt_exception(STAT_PLANNER_ASSERTION_FAILURE, msg);

    for (uint8_t i=0; i<PLANNER_BUFFER_POOL_SIZE; i++) {
        printf("{\"er\":{\"stat\":%d, \"type\":%d, \"lock\":%d, \"replan\":%d",
                mb.bf[i].buffer_state,
                mb.bf[i].move_type,
                mb.bf[i].locked,
                mb.bf[i].replannable);
        if (&mb.bf[i] == mb.r) {
            printf(", \"RUN\":t");}
        if (&mb.bf[i] == mb.q) {
            printf(", \"QUE\":t");}
        if (&mb.bf[i] == mb.w) {
            printf(", \"WRT\":t");}
        printf("}}\n");
    }
}

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
        //  - MP_BUFFER_QUEUED (zero or more)
        //  - MP_BUFFER_PLANNING (zero or more)
        //  - MP_BUFFER_EMPTY (zero or more up until mb.r)
        //  - no more

        // After RUNNING, we can see anything but PENDING, but prefer not to find PLANNING
        if (bf->pv->buffer_state == MP_BUFFER_RUNNING && bf->buffer_state != MP_BUFFER_QUEUED && bf->buffer_state != MP_BUFFER_EMPTY) {
            // Exception: PLANNING is allowed, but we may want to watch for it:
            if (bf->buffer_state == MP_BUFFER_PLANNING) {
                __NOP();
            } else {
                _planner_report("buffer audit4");
                _debug_trap();
            }
        }

        // After QUEUED, we can see QUEUED, PLANNING, or EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_QUEUED && bf->buffer_state != MP_BUFFER_QUEUED && bf->buffer_state != MP_BUFFER_PLANNING && bf->buffer_state != MP_BUFFER_EMPTY) {
            _planner_report("buffer audit5");
            _debug_trap();
        }

        // After PLANNING, we can see PLANNING, or EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_PLANNING && bf->buffer_state != MP_BUFFER_PLANNING && bf->buffer_state != MP_BUFFER_QUEUED && bf->buffer_state != MP_BUFFER_EMPTY) {
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
