/*
 * plan_exec.cpp - execution function for acceleration managed lines
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

#include "tinyg2.h"
#include "config.h"
#include "controller.h"
#include "planner.h"
#include "kinematics.h"
#include "stepper.h"
#include "encoder.h"
#include "report.h"
#include "util.h"
#include "spindle.h"
#include "xio.h"    //+++++DIAGNOSTIC

// execute routines (NB: These are all called from the LO interrupt)
static stat_t _exec_aline_head(mpBuf_t *bf); // passing bf because body might need it, and it might call body
static stat_t _exec_aline_body(mpBuf_t *bf); // passing bf so that body can extend itself if the exit velocity rises.
static stat_t _exec_aline_tail(void);
static stat_t _exec_aline_segment(void);

static void _init_forward_diffs(float Vi, float Vt, const float a_0/* = 0*/, const float a_1/* = 0*/, const float j_0/* = 0*/, const float j_1/* = 0*/, const float T/* = 0*/);


/*************************************************************************
 * mp_plan_move() - call ramping function to plan moves ahead of the exec
 *
 */

stat_t mp_plan_move()
{
    mpBuf_t *bf;

    // NULL means nothing's running - this is OK
    if ((bf = mp_get_run_buffer()) == NULL) {
        st_prep_null();
        return (STAT_NOOP);
    }

    bool skipped = false;
    // We will plan one ahead if this one is already running.
    if (bf->buffer_state == MP_BUFFER_RUNNING) {
        // If we're running, we've already called ramps

        // Check to see if we need to extend the body
        // We could be in a few states right now:
        // (1) mr.group_section == head, so we can wait for it to start getting handed out
        // (2) mr.group_section == body, in which case we partially reset
        // (3) mr.group_section == tail, in which case we shouldn't even attempt to make changes.

        if ((mr.group_section != SECTION_TAIL) && (mr.group_exit_velocity < br->nx_group->pv->exit_velocity)) {
            // We'll extend the body.

            if (br->exit_velocity < mr.group_cruise_velocity) {
                // we will have a tail
                mr.group_exit_velocity = br->exit_velocity;

                // bf passed to get_target_length needs to have valid jerk (and derived values) for the group
                mr.group_tail_length = mp_get_target_length(mr.group_exit_velocity, mr.group_cruise_velocity, br->nx_group->pv);
                mr.group_body_length = mr.group_length - mr.group_tail_length;

            } else {
                // we will cruise until the end of the group
                mr.group_exit_velocity = mr.group_cruise_velocity;

                mr.group_body_length += mr.group_tail_length;
                mr.group_tail_length = 0;

                mr.group_body_time += mr.group_tail_time;
                mr.group_tail_time = 0;
            }

            if (mr.group_section == SECTION_BODY) {
                // rewidn to MOVE_NEW, and set the length into the move to be what we've executed already:
                mr.group_move_state = MOVE_NEW;
                mr.length_into_section = mr.executed_group_body_length;

            }
            // if the next more is planned already, we'll allow it to be replanned
            if (mb->nx->buffer_state == MP_BUFFER_PLANNED) {
                mb->nx->buffer_state = MP_BUFFER_PREPPED;
            }
        }
        else {
            skipped = true;
            bf = bf->nx;
        }
    }

    // Note that that can only be one PLANNED move at a time.
    // This is to help sync mr.p to point to the next planned mr.bf
    // mr.p is only advanced in mp_exec_aline, after mp.r = mr.p.

    if (bf->buffer_state == MP_BUFFER_PREPPED) {
        if (skipped) {
            bf->pv->plannable = false; // we're about to plan past bf->pv, so we must lock it
        }

        // group_move_state of MOVE_OFF means we need to run ramps for the next group
        if (mr.group_move_state == MOVE_OFF) {
            mp_calculate_ramps(bf, mr.p);

            mr.group_length = bf->group_length;
            mr.length_into_section = 0;
            mr.t_into_section = 0; // inital guess for a head.

            // copy lengths
            mr.group_head_length = mr.p->head_length;
            mr.group_body_length = mr.p->body_length;
            mr.group_tail_length = mr.p->tail_length;

            // copy times
            mr.group_head_time = mr.p->head_time;
            mr.group_body_time = mr.p->body_time;
            mr.group_tail_time = mr.p->tail_time;

            mr.executed_group_head_length = 0.0
            mr.executed_group_body_length = 0.0

            mr.group_move_state = MOVE_NEW;

            mr.group_section = SECTION_HEAD;

            // copy the jerk values to the last block of the group, so we have them all thr way through
            mpBuf_t *bf_last = bf->nx_group->pv;
            if (!fp_EQ(bf_last->jerk, bf->jerk)) { // check to see if they're the same. bf_last could = bf!
                // Copy the move jerk, and all of it's derived values over
                bf_last->jerk = bf->jerk;
                bf_last->jerk_sq = bf->jerk_sq;
                bf_last->recip_jerk = bf->recip_jerk;
                bf_last->sqrt_j = bf->sqrt_j;
                bf_last->q_recip_2_sqrt_j = bf->q_recip_2_sqrt_j;
            }
        }
    }

    // MOVE_NEW is likely from directly abot, BUT, it can also be set in EXEC to
    // indicate we need to replan the body and tail.
    if (mr.group_move_state == MOVE_NEW) {
        mr.group_move_state = MOVE_RUN;

        // Assuming bf is the head of a group

        // Now we want to "lock" every block that completely contains the head and body.
        // Important: We don't lock a block that contains the end of the head and body.
        float lock_length_left = (mr.group_head_length - mr.executed_group_head_length) + (mr.group_body_length - mr.executed_group_body_length);
        mpBuf_t *bf_lookahead = bf;
        for (; bf_lookahead->length < lock_length_left; bf_lookahead = bf_lookahead->nx) {
            bf_lookahead->plannable = false;
            lock_length_left -= bf_lookahead->length;

            // Set the nx_group so that we can find it from any of these blocks
            bf_lookahead->nx_group = bf->nx_group;
        }
        // bf_lookahead is now pointing at the last block of the tail, if any
        // We want to lock in the entry and cruise velocities if the block is still plannable
        if (bf_lookahead->plannable) {
            // Set the nx_group so that we can find it from any of these blocks
            bf_lookahead->nx_group = bf->nx_group;

            // We want need to push our exit_velocity back to the last block, so we can see if it changed.
            bf->nx_group->pv->exit_velocity = mr.group_exit_velocity;

            // We want the planner to see this as the first block of the group
            bf->nx_group->pv_group = bf_lookahead;

            // WARNING: We're setting the exits to zero. We're assuming that back-planning won't care, as
            // long as the possible entry it finds is higher than what we set, and forward planning is already done.
            // mp_calculate_block() MUST not pay attention to vmax values.
            bf_lookahead->pv->exit_vmax     = 0.0;
            bf_lookahead->pv->exit_velocity = 0.0;

            // We also ensure that the cruise can't be adjusted.
            bf_lookahead->cruise_vmax     = mr.r->cruise_velocity;
            // The actual cruise that ends up being used will be set by mp_calculate_block(), from mr.group_cruise_velocity.
            bf_lookahead->cruise_velocity = mr.r->cruise_velocity;
        }
    }

    // group_move_state of MOVE_RUN means we need to compute the head/body/tail for this block
    // when the group is one block long, this is basically a copy, plus time computation
    if (mr.group_move_state == MOVE_RUN) {
        if (bf->buffer_state == MP_BUFFER_RUNNING) {
            // We hare requested an update of the running block
            stat_t status =  mp_calculate_block(bf, mr.r);
            SANITY_TRAPS(bf, mr.r);
        } else {
            stat_t status =  mp_calculate_block(bf, mr.p);
            SANITY_TRAPS(bf, mr.p);
        }

        // status will be STAT_EAGAIN if there are more blocks in this group,
        // or STAT_OK if the group is done.

        if (status == STAT_OK) {
            mr.group_move_state = MOVE_OFF;
        }

        bf->buffer_state = MP_BUFFER_PLANNED;

        // report that we planned something...
        return (STAT_OK);
    }

    // We did nothing
    return (STAT_NOOP);
}

/*************************************************************************
 * mp_exec_move() - execute runtime functions to prep move for steppers
 *
 *	Dequeues the buffer queue and executes the move continuations.
 *	Manages run buffers and other details
 */

stat_t mp_exec_move()
{
	mpBuf_t *bf;

    // NULL means nothing's running - this is OK
    if ((bf = mp_get_run_buffer()) == NULL) {
		st_prep_null();
		return (STAT_NOOP);
	}

    // first-time operations
    if (bf->buffer_state != MP_BUFFER_RUNNING) {
        if (bf->buffer_state < MP_BUFFER_PREPPED) {
            rpt_exception(42, "mp_exec_move() buffer is not prepped");
    		st_prep_null();

            return (STAT_NOOP);
        }
        if (bf->nx->buffer_state < MP_BUFFER_PREPPED) {
            rpt_exception(42, "mp_exec_move() next buffer is empty");
        }

        if (bf->buffer_state == MP_BUFFER_PREPPED) {
            // We need to have it planned. We don't want to do this here, as it
            // might already be happening in a lower interrupt.
            st_request_plan_move();
            return (STAT_NOOP);
        }

        bf->buffer_state = MP_BUFFER_RUNNING;               // must precede mp_planner_time_acccounting()
        mp_planner_time_accounting();
    }

    if (bf->nx->buffer_state == MP_BUFFER_PREPPED) {
        // We go ahead and *ask* for a forward planning of the next move.
        // This won't call mp_plan_move until we leave this function
        // (and have called mp_exec_aline via bf->bf_func).
        // This also allows mp_exec_aline to advance mr.p first.
        st_request_plan_move();
    }

	// Manage motion state transitions
    if (bf->move_type == MOVE_TYPE_ALINE) { 			// cycle auto-start for lines only
        if ((cm.motion_state != MOTION_RUN) && (cm.motion_state != MOTION_HOLD)) {
            cm_set_motion_state(MOTION_RUN);
        }
    }
    if (bf->bf_func == NULL) {
        return(cm_panic(STAT_INTERNAL_ERROR, "mp_exec_move()")); // never supposed to get here
    }
	return (bf->bf_func(bf)); 							// run the move callback in the planner buffer
}

/*************************************************************************/
/**** ALINE EXECUTION ROUTINES *******************************************/
/*************************************************************************
 * ---> Everything here fires from interrupts and must be interrupt safe
 *
 *  _exec_aline()		  - acceleration line main routine
 *	_exec_aline_head()	  - helper for acceleration section
 *	_exec_aline_body()	  - helper for cruise section
 *	_exec_aline_tail()	  - helper for deceleration section
 *	_exec_aline_segment() - helper for running a segment
 *
 *	Returns:
 *	 STAT_OK		move is done
 *	 STAT_EAGAIN	move is not finished - has more segments to run
 *	 STAT_NOOP		cause no operation from the steppers - do not load the move
 *	 STAT_xxxxx		fatal error. Ends the move and frees the bf buffer
 *
 *	This routine is called from the (LO) interrupt level. The interrupt
 *	sequencing relies on the behaviors of the routines being exactly correct.
 *	Each call to _exec_aline() must execute and prep *one and only one*
 *	segment. If the segment is the not the last segment in the bf buffer the
 *	_aline() must return STAT_EAGAIN. If it's the last segment it must return
 *	STAT_OK. If it encounters a fatal error that would terminate the move it
 *	should return a valid error code. Failure to obey this will introduce
 *	subtle and very difficult to diagnose bugs (trust me on this).
 *
 *	Note 1 Returning STAT_OK ends the move and frees the bf buffer.
 *		   Returning STAT_OK at this point does NOT advance position meaning any
 *		   position error will be compensated by the next move.
 *
 *	Note 2 Solves a potential race condition where the current move ends but the
 * 		   new move has not started because the previous move is still being run
 *		   by the steppers. Planning can overwrite the new move.
 */
/* OPERATION:
 *	Aline generates jerk-controlled S-curves as per Ed Red's course notes:
 *	  http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf
 *	  http://www.scribd.com/doc/63521608/Ed-Red-Ch5-537-Jerk-Equations
 *
 *	A full trapezoid is divided into 5 periods Periods 1 and 2 are the
 *	first and second halves of the acceleration ramp (the concave and convex
 *	parts of the S curve in the "head"). Periods 3 and 4 are the first
 *	and second parts of the deceleration ramp (the tail). There is also
 *	a period for the constant-velocity plateau of the trapezoid (the body).
 *	There are various degraded trapezoids possible, including 2 section
 *	combinations (head and tail; head and body; body and tail), and single
 *	sections - any one of the three.
 *
 *	The equations that govern the acceleration and deceleration ramps are:
 *
 *	  Period 1	  V = Vi + Jm*(T^2)/2
 *	  Period 2	  V = Vh + As*T - Jm*(T^2)/2
 *	  Period 3	  V = Vi - Jm*(T^2)/2
 *	  Period 4	  V = Vh + As*T + Jm*(T^2)/2
 *
 * 	These routines play some games with the acceleration and move timing
 *	to make sure this actually all works out. move_time is the actual time of the
 *	move, accel_time is the time valaue needed to compute the velocity - which
 *	takes the initial velocity into account (move_time does not need to).
 */
/* --- State transitions - hierarchical state machine ---
 *
 *	bf->move_state transitions:
 *	 from _NEW to _RUN on first call (sub_state set to _OFF)
 *	 from _RUN to _OFF on final call
 * 	 or just remains _OFF
 *
 *	mr.move_state transitions on first call from _OFF to one of _HEAD, _BODY, _TAIL
 *	Within each section state may be
 *	 _NEW - trigger initialization
 *	 _RUN1 - run the first part
 *	 _RUN2 - run the second part
 */
/*	Note:
 *	For a version of these routines that execute using the original equation-of-motion
 *	math (as opposed to the forward difference math) please refer to build 357.xx or earlier.
 *	Builds 358 onward have only forward difference code. ALso, the Kahan corrections for the
 *  forward differencing were also been removed shortly after as they were not needed.
 */

/**** NOTICE ** NOTICE ** NOTICE ****
 **
 **    mp_exec_aline() is called in
 **     --INTERRUPT CONTEXT!!--
 **
 **    Things we MUST NOT do (even indirectly):
 **       mp_plan_buffer()
 **       mp_plan_block_list()
 **       printf()
 **
 **** NOTICE ** NOTICE ** NOTICE ****/

stat_t mp_exec_aline(mpBuf_t *bf)
{
    if (bf->move_state == MOVE_OFF) {
        return (STAT_NOOP);
     }

    // Initialize all new blocks, regardless of normal or feedhold operation
    if (mr.move_state == MOVE_OFF) {

        // too short lines have already been removed...
        // so is the following code is no longer needed ++++ ash
        // But let's still alert the condition should it ever occur
        if (fp_ZERO(bf->length)) {						// ...looks for an actual zero here
            rpt_exception(STAT_PLANNER_ASSERTION_FAILURE, "mp_exec_aline() zero length move");
        }

        // Start a new move by setting up the runtime singleton (mr)
        memcpy(&mr.gm, &(bf->gm), sizeof(GCodeState_t)); // copy in the gcode model state
        bf->move_state = MOVE_RUN;                      // note that this buffer is running -- note the planner doesn't look at move_state
        mr.move_state = MOVE_NEW;
        mr.section = SECTION_HEAD;
        mr.section_state = SECTION_NEW;
        mr.jerk = bf->jerk;

        mr.entry_velocity     = mr.r->exit_velocity;     // feed the old exit into the entry.
        mr.entry_acceleration = mr.r->exit_acceleration;
        mr.entry_jerk         = mr.r->exit_jerk;

        mr.r = mr.p;
        mr.p = mr.p->nx;

        // reset the executed values
        mr.executed_body_length = 0.0;
        mr.executed_body_time   = 0.0;


        // Assumptions that are required for this to work:
        // entry velocity <= cruise velocity && cruise velocity >= exit velocity
        // Even if the move is head or tail only, cruise velocity needs to be valid.
        // This is because a "head" is *always* entry->cruise, and a "tail" is *always* cruise->exit,
        // even if there are not other sections int he move. (This is a significant time savings.)


        // Here we will check to make sure that the sections are longer than MIN_SEGMENT_TIME

        if (mr.r->head_time < MIN_SEGMENT_TIME) {
            // head_time !== body_time
            // We have to compute the new body time addition.
            mr.r->body_time += mr.r->head_length/mr.r->cruise_velocity;
            mr.r->head_time = 0;

            mr.r->body_length += mr.r->head_length;
            mr.r->head_length = 0;
        }
        if (mr.r->tail_time < MIN_SEGMENT_TIME) {
            // tail_time !== body_time
            // We have to compute the new body time addition.
            mr.r->body_time += mr.r->tail_length/mr.r->cruise_velocity;
            mr.r->tail_time = 0;

            mr.r->body_length += mr.r->tail_length;
            mr.r->tail_length = 0;
        }

        // At this point, we've already possibly merged heat and/or tail into the body.
        // If the body is too "short" (brief) still, we *might* be able to add it to a head or tail.
        // If there's still a head or a tail, we will add the body to whichever there is, maybe both.
        // We saved it for last since it's the most expensive.
        if (mr.r->body_time < MIN_SEGMENT_TIME) {
            if (mr.r->tail_length > 0) {
                if (mr.r->head_length > 0) {
                    // We'll split the body of the head and tail
                    float body_split = mr.r->body_length/2.0;
                    mr.r->body_length = 0;

                    mr.r->head_length += body_split;
                    mr.r->tail_length += body_split;

                    // TODO: ++++ RG This is more complicated than this.
                    mr.r->head_time += (2.0 * body_split)/(mr.entry_velocity + mr.r->cruise_velocity);
                    mr.r->tail_time += (2.0 * body_split)/(mr.r->cruise_velocity + mr.r->exit_velocity);
                } else {
                    // We'll put it all in the tail
                    mr.r->tail_length += mr.r->body_length;
                    // TODO: ++++ RG This is more complicated than this.
                    mr.r->tail_time += (2.0 * mr.r->body_length)/(mr.r->cruise_velocity + mr.r->exit_velocity);

                    mr.r->body_length = 0;
                }
            }
            else if (mr.r->head_length > 0) {
                // We'll put it all in the head
                mr.r->head_length += mr.r->body_length;
                // TODO: ++++ RG This is more complicated than this.
                mr.r->head_time += (2.0 * mr.r->body_length)/(mr.entry_velocity + mr.r->cruise_velocity);

                mr.r->body_length = 0;
            }
            else {
                // Uh oh! We have a move that's all body, and is still too short!!
                // ++++ RG For now, we'll consider this impossible.
                while (1);
            }
        }

        copy_vector(mr.unit, bf->unit);
        copy_vector(mr.target, bf->gm.target);          // save the final target of the move
        copy_vector(mr.axis_flags, bf->axis_flags);

        // generate the way points for position correction at section ends
        for (uint8_t axis=0; axis<AXES; axis++) {
            mr.waypoint[SECTION_HEAD][axis] = mr.position[axis] + mr.unit[axis] * mr.r->head_length;
            mr.waypoint[SECTION_BODY][axis] = mr.position[axis] + mr.unit[axis] * (mr.r->head_length + mr.r->body_length);
            mr.waypoint[SECTION_TAIL][axis] = mr.position[axis] + mr.unit[axis] * (mr.r->head_length + mr.r->body_length + mr.r->tail_length);
        }

        // Update the planner buffer times
        mb.run_time_remaining = bf->move_time;   // initialize the run_time_remaining
    }

    // Feed Override Processing - We need to handle the following cases (listed in rough sequence order):
    //  (1) - We've received a feed override request in the middle of a cycle

    // Feedhold Processing - We need to handle the following cases (listed in rough sequence order):
    //  (1) - We have a block midway through normal execution and a new feedhold request
    //   (1a) - The deceleration will fit in the length remaining in the running block (mr)
    //   (1b) - The deceleration will not fit in the running block
    //   (1c) - 1a, except the remaining length would be zero or EPSILON close to zero (unlikely)
    //  (2) - We have a new block and a new feedhold request that arrived at EXACTLY the same time (unlikely, but handled)
    //  (3) - We are in the middle of a block that is currently decelerating
    //  (4) - We have decelerated a block to some velocity > zero (needs continuation in next block)
    //  (5) - We have decelerated a block to zero velocity
    //  (6) - We have finished all the runtime work now we have to wait for the steppers to stop
    //  (7) - The steppers have stopped. No motion should occur
    //  (8) - We are removing the hold state and there is queued motion (handled outside this routine)
    //  (9) - We are removing the hold state and there is no queued motion (also handled outside this routine)

    if (cm.motion_state == MOTION_HOLD) {

        // Case (3) is a no-op and is not trapped. It just continues the deceleration.

        // Case (7) - all motion has ceased
        if (cm.hold_state == FEEDHOLD_HOLD) {
            return (STAT_NOOP);                 // VERY IMPORTANT to exit as a NOOP. No more movement
        }

        // Case (6) - wait for the steppers to stop
        if (cm.hold_state == FEEDHOLD_PENDING) {
            if (mp_runtime_is_idle()) {                                 // wait for the steppers to actually clear out
                cm.hold_state = FEEDHOLD_HOLD;
                mp_zero_segment_velocity();                             // for reporting purposes
                sr_request_status_report(SR_REQUEST_IMMEDIATE);         // was SR_REQUEST_TIMED
                cs.controller_state = CONTROLLER_READY;                 // remove controller readline() PAUSE
            }
            return (STAT_OK);                                           // hold here. No more movement
        }

        // Case (5) - decelerated to zero
        // Update the run buffer then force a replan of the whole planner queue
        if (cm.hold_state == FEEDHOLD_DECEL_END) {
            mr.move_state = MOVE_OFF;	                                // invalidate mr buffer to reset the new move
            bf->move_state = MOVE_NEW;                                  // tell _exec to re-use the bf buffer
            bf->length = get_axis_vector_length(mr.target, mr.position);// reset length
            //bf->entry_vmax = 0;                                         // set bp+0 as hold point
            mp_replan_queue(mb.r);                                      // make it replan all the blocks
            cm.hold_state = FEEDHOLD_PENDING;
            return (STAT_OK);
        }

        // Cases (1a, 1b), Case (2), Case (4)
        // Build a tail-only move from here. Decelerate as fast as possible in the space we have.
        if ((cm.hold_state == FEEDHOLD_SYNC) ||
            ((cm.hold_state == FEEDHOLD_DECEL_CONTINUE) && (mr.move_state == MOVE_NEW))) {
            if (mr.section == SECTION_TAIL) {   // if already in a tail don't decelerate. You already are
                if (fp_ZERO(mr.r->exit_velocity)) {
                    cm.hold_state = FEEDHOLD_DECEL_TO_ZERO;
                } else {
                    cm.hold_state = FEEDHOLD_DECEL_CONTINUE;
                }
            } else {
                mr.entry_velocity = mr.segment_velocity;
                if (mr.section == SECTION_HEAD) {
                    mr.entry_velocity += mr.forward_diff_5; // compute velocity for next segment (this new one)
                }
                mr.r->cruise_velocity = mr.entry_velocity;

                mr.section = SECTION_TAIL;
                mr.section_state = SECTION_NEW;
                mr.jerk = bf->jerk;
                mr.r->head_length = 0;
                mr.r->body_length = 0;

                float available_length = get_axis_vector_length(mr.target, mr.position);
                mr.r->tail_length = mp_get_target_length(0, mr.r->cruise_velocity, bf);   // braking length

                if (fp_ZERO(available_length - mr.r->tail_length)) {    // (1c) the deceleration time is almost exactly the remaining of the current move
                    cm.hold_state = FEEDHOLD_DECEL_TO_ZERO;
                    mr.r->exit_velocity = 0;
                    mr.r->tail_length = available_length;
                } else if (available_length < mr.r->tail_length) { // (1b) the deceleration has to span multiple moves
                    cm.hold_state = FEEDHOLD_DECEL_CONTINUE;
                    mr.r->tail_length = available_length;
                    mr.r->exit_velocity = mr.r->cruise_velocity - mp_get_target_velocity(0, mr.r->tail_length, bf);
                } else {                                        // (1a)the deceleration will fit into the current move
                    cm.hold_state = FEEDHOLD_DECEL_TO_ZERO;
                    mr.r->exit_velocity = 0;
                }
                mr.r->tail_time = mr.r->tail_length*2 / (mr.r->exit_velocity + mr.r->cruise_velocity);
            }
        }
    }
    mr.move_state = MOVE_RUN;

    // NB: from this point on the contents of the bf buffer do not affect execution

	//**** main dispatcher to process segments ***
	stat_t status = STAT_OK;
	if (mr.section == SECTION_HEAD) { status = _exec_aline_head(bf);} else
	if (mr.section == SECTION_BODY) { status = _exec_aline_body(bf);} else
	if (mr.section == SECTION_TAIL) { status = _exec_aline_tail();} else
	{ return(cm_panic(STAT_INTERNAL_ERROR, "exec_aline()"));}	// never supposed to get here

    // We can't use the if/else block above, since the head may call body, and boty call tail, so we wait till after
    if (mr.section == SECTION_TAIL) {
        bf->plannable = false; // Once we're in the tail, we can't plan the block anymore
    } else if (mr.section == SECTION_BODY) {
        // TODO: adjust body if the exit changed
    }

	// Feedhold Case (5): Look for the end of the deceleration to go into HOLD state
    if ((cm.hold_state == FEEDHOLD_DECEL_TO_ZERO) && (status == STAT_OK)) {
        cm.hold_state = FEEDHOLD_DECEL_END;
        bf->move_state = MOVE_NEW;                      // reset bf so it can restart the rest of the move
    }

	// There are 4 things that can happen here depending on return conditions:
	//  status       bf->move_state   Description
	//  -----------	 --------------   ----------------------------------------
	//  STAT_EAGAIN  <don't care>     mr buffer has more segments to run
	//  STAT_OK       MOVE_RUN        mr and bf buffers are done
	//  STAT_OK       MOVE_NEW        mr done; bf must be run again (it's been reused)
    //  There is no fourth thing. Nobody expects the Spanish Inquisition

	if (status == STAT_EAGAIN) {
		sr_request_status_report(SR_REQUEST_TIMED);		// continue reporting mr buffer
                                                        // Note that tha'll happen in a lower interrupt level.
	} else {
		mr.move_state = MOVE_OFF;						// invalidate mr buffer (reset)
		mr.section_state = SECTION_OFF;
        mb.run_time_remaining = 0.0;                    // it's done, so time goes to zero

        if (bf->move_state == MOVE_RUN) {
			if (mp_free_run_buffer() && cm.hold_state == FEEDHOLD_OFF) {
				cm_cycle_end();	// free buffer & end cycle if planner is empty
            }
		}
	}
    return (status);
}

/*
 * mp_exit_hold_state() - end a feedhold
 *
 *	Feedhold is executed as cm.hold_state transitions executed inside _exec_aline()
 *  Invoke a feedhold by calling cm_request_hold() or cm_start_hold() directly
 *  Return from feedhold by calling cm_request_end_hold() or cm_end_hold directly.
 *  See canonical_macine.c for a more detailed explanation of feedhold operation.
 */

void mp_exit_hold_state()
{
	cm.hold_state = FEEDHOLD_OFF;
	if (mp_has_runnable_buffer()) {
	    cm_set_motion_state(MOTION_RUN);
        st_request_exec_move();
	    sr_request_status_report(SR_REQUEST_IMMEDIATE);
    } else {
		cm_set_motion_state(MOTION_STOP);
	}
}

/*
 * Forward difference math explained:
 *
 *	We are using a quintic (fifth-degree) Bezier polynomial for the velocity curve.
 *	This gives us a "linear pop" velocity curve; with pop being the sixth derivative of position:
 *	velocity - 1st, acceleration - 2nd, jerk - 3rd, snap - 4th, crackle - 5th, pop - 6th
 *
 * The Bezier curve takes the form:
 *
 *  V(t) = P_0 * B_0(t) + P_1 * B_1(t) + P_2 * B_2(t) + P_3 * B_3(t) + P_4 * B_4(t) + P_5 * B_5(t)
 *
 * Where 0 <= t <= 1, and V(t) is the velocity. P_0 through P_5 are the control points, and B_0(t)
 * through B_5(t) are the Bernstein basis as follows:
 *
 *		B_0(t) =   (1-t)^5        =   -t^5 +  5t^4 - 10t^3 + 10t^2 -  5t   +   1
 *		B_1(t) =  5(1-t)^4 * t    =   5t^5 - 20t^4 + 30t^3 - 20t^2 +  5t
 *		B_2(t) = 10(1-t)^3 * t^2  = -10t^5 + 30t^4 - 30t^3 + 10t^2
 *		B_3(t) = 10(1-t)^2 * t^3  =  10t^5 - 20t^4 + 10t^3
 *		B_4(t) =  5(1-t)   * t^4  =  -5t^5 +  5t^4
 *		B_5(t) =             t^5  =    t^5
 *		                              ^       ^       ^       ^       ^       ^
 *		                              |       |       |       |       |       |
 *		                              A       B       C       D       E       F
 *
 *  We use forward-differencing to calculate each position through the curve.
 *	This requires a formula of the form:
 *
 *		V_f(t) = A*t^5 + B*t^4 + C*t^3 + D*t^2 + E*t + F
 *
 *  Looking at the above B_0(t) through B_5(t) expanded forms, if we take the coefficients of t^5
 *  through t of the Bezier form of V(t), we can determine that:
 *
 *		A =    -P_0 +  5*P_1 - 10*P_2 + 10*P_3 -  5*P_4 +  P_5
 *		B =   5*P_0 - 20*P_1 + 30*P_2 - 20*P_3 +  5*P_4
 *		C = -10*P_0 + 30*P_1 - 30*P_2 + 10*P_3
 *		D =  10*P_0 - 20*P_1 + 10*P_2
 *		E = - 5*P_0 +  5*P_1
 *		F =     P_0
 *
 *	Now, since we will (currently) *always* want the initial acceleration and jerk values to be 0,
 *	We set P_i = P_0 = P_1 = P_2 (initial velocity), and P_t = P_3 = P_4 = P_5 (target velocity),
 *	which, after simplification, resolves to:
 *
 *		A = - 6*P_i +  6*P_t
 *		B =  15*P_i - 15*P_t
 *		C = -10*P_i + 10*P_t
 *		D = 0
 *		E = 0
 *		F = P_i
 *
 *  UPDATE: We will now be accepting initial/final Accel/Jerk, which means we will have all six
 *  control points. j_0, j_1 are the jerk, a_0, a_1 are the acceleration, and T is total move time.
 *
 *    P_0 = v_0
 *    P_1 = v_0 + (1/5) T a_0
 *    P_2 = v_0 + (2/5) T a_0 + (1/20) T^2 j_0
 *    P_3 = v_1 - (2/5) T a_1 - (1/20) T^2 j_1
 *    P_4 = v_1 - (1/5) T a_1
 *    P_5 = v_1
 *
 * Simplified:
 *    A =  5( P_1 - P_4 + 2(P_3 - P_2) ) + P_5 - P_0
 *    B =  5( P_0 + P_4 - 4(P_3 + P_1) + 6 P_2 )
 *    C = 10( P_3 - P_0 + 3(P_1 - P_2) )
 *    D = 10( P_0 + P_2 - 2 P_1 )
 *    E = 5 ( P_1 - P_0 )
 *    F =     P_0
 *
 *	Given an interval count of I to get from P_i to P_t, we get the parametric "step" size of h = 1/I.
 *	We need to calculate the initial value of forward differences (F_0 - F_5) such that the inital
 *	velocity V = P_i, then we iterate over the following I times:
 *
 *		V   += F_5
 *		F_5 += F_4
 *		F_4 += F_3
 *		F_3 += F_2
 *		F_2 += F_1
 *
 *	See http://www.drdobbs.com/forward-difference-calculation-of-bezier/184403417 for an example of
 *	how to calculate F_0 - F_5 for a cubic bezier curve. Since this is a quintic bezier curve, we
 *	need to extend the formulas somewhat. I'll not go into the long-winded step-by-step here,
 *	but it gives the resulting formulas:
 *
 *		a = A, b = B, c = C, d = D, e = E, f = F
 *		F_5(t+h)-F_5(t) = (5ah)t^4 + (10ah^2 + 4bh)t^3 + (10ah^3 + 6bh^2 + 3ch)t^2 +
 *			(5ah^4 + 4bh^3 + 3ch^2 + 2dh)t + ah^5 + bh^4 + ch^3 + dh^2 + eh
 *
 *		a = 5ah
 *		b = 10ah^2 + 4bh
 *		c = 10ah^3 + 6bh^2 + 3ch
 *		d = 5ah^4 + 4bh^3 + 3ch^2 + 2dh
 *
 *  (After substitution, simplification, and rearranging):
 *		F_4(t+h)-F_4(t) = (20ah^2)t^3 + (60ah^3 + 12bh^2)t^2 + (70ah^4 + 24bh^3 + 6ch^2)t +
 *			30ah^5 + 14bh^4 + 6ch^3 + 2dh^2
 *
 *		a = (20ah^2)
 *		b = (60ah^3 + 12bh^2)
 *		c = (70ah^4 + 24bh^3 + 6ch^2)
 *
 *  (After substitution, simplification, and rearranging):
 *		F_3(t+h)-F_3(t) = (60ah^3)t^2 + (180ah^4 + 24bh^3)t + 150ah^5 + 36bh^4 + 6ch^3
 *
 *  (You get the picture...)
 *		F_2(t+h)-F_2(t) = (120ah^4)t + 240ah^5 + 24bh^4
 *		F_1(t+h)-F_1(t) = 120ah^5
 *
 *  Normally, we could then assign t = 0, use the A-F values from above, and get out initial F_* values.
 *  However, for the sake of "averaging" the velocity of each segment, we actually want to have the initial
 *  V be be at t = h/2 and iterate I-1 times. So, the resulting F_* values are (steps not shown):
 *
 *		F_5 = (121Ah^5)/16 + 5Bh^4 + (13Ch^3)/4 + 2Dh^2 + Eh
 *		F_4 = (165Ah^5)/2 + 29Bh^4 + 9Ch^3 + 2Dh^2
 *		F_3 = 255Ah^5 + 48Bh^4 + 6Ch^3
 *		F_2 = 300Ah^5 + 24Bh^4
 *		F_1 = 120Ah^5
 *
 */

#define USE_OLD_FORWARD_DIFFS 0

#if USE_OLD_FORWARD_DIFFS==1

// Total time: 147us
static void _init_forward_diffs(float Vi, float Vt, const float a_0 = 0, const float a_1 = 0, const float j_0 = 0, const float j_1 = 0, const float T = 0)
{
    // Times from *here*
    float A =  -6.0*Vi +  6.0*Vt;
    float B =  15.0*Vi - 15.0*Vt;
    float C = -10.0*Vi + 10.0*Vt;
    // D = 0
    // E = 0
    // F = Vi

    float h   = 1/(mr.segments);
    //	float h_3 = h * h * h;
    //	float h_4 = h_3 * h;
    //	float h_5 = h_4 * h;

    float Ah_5 = A * h * h * h * h * h;
    float Bh_4 = B * h * h * h * h;
    float Ch_3 = C * h * h * h;

    mr.forward_diff_5 = (121.0/16.0)*Ah_5 + 5.0*Bh_4 + (13.0/4.0)*Ch_3;
    mr.forward_diff_4 = (165.0/2.0)*Ah_5 + 29.0*Bh_4 + 9.0*Ch_3;
    mr.forward_diff_3 = 255.0*Ah_5 + 48.0*Bh_4 + 6.0*Ch_3;
    mr.forward_diff_2 = 300.0*Ah_5 + 24.0*Bh_4;
    mr.forward_diff_1 = 120.0*Ah_5;

    // Calculate the initial velocity by calculating V(h/2)
    float half_h = h/2.0;
    float half_Ch_3 = C * half_h * half_h * half_h;
    float half_Bh_4 = B * half_h * half_h * half_h * half_h;
    float half_Ah_5 = A * half_h * half_h * half_h * half_h * half_h;
    mr.segment_velocity = half_Ah_5 + half_Bh_4 + half_Ch_3 + Vi;
}

#else

// Total time: 147us
static void _init_forward_diffs(const float v_0, const float v_1, const float a_0, const float a_1, const float j_0, const float j_1, const float T)
{
    // Times from *here*
    const float fifth_T        = T * 0.2; //(1/5) T
    const float two_fifths_T   = T * 0.4; //(1/5) T
    const float twentienth_T_2 = T * T * 0.05; // (1/20) T^2

    const float P_0 = v_0;
    const float P_1 = v_0 +      fifth_T*a_0;
    const float P_2 = v_0 + two_fifths_T*a_0 + twentienth_T_2*j_0;
    const float P_3 = v_1 - two_fifths_T*a_1 - twentienth_T_2*j_1;
    const float P_4 = v_1 -      fifth_T*a_1;
    const float P_5 = v_1;

    const float A =  5*( P_1 - P_4 + 2*(P_3 - P_2) ) +   P_5 - P_0;
    const float B =  5*( P_0 + P_4 - 4*(P_3 + P_1)   + 6*P_2 );
    const float C = 10*( P_3 - P_0 + 3*(P_1 - P_2) );
    const float D = 10*( P_0 + P_2 - 2*P_1 );
    const float E =  5*( P_1 - P_0 );
    //const float F =      P_0;

	const float h   = 1/(mr.segments);
    const float h_2 = h   * h;
	const float h_3 = h_2 * h;
	const float h_4 = h_3 * h;
	const float h_5 = h_4 * h;

	const float Ah_5 = A * h_5;
	const float Bh_4 = B * h_4;
	const float Ch_3 = C * h_3;
    const float Dh_2 = D * h_2;
    const float Eh   = E * h;

    const float const1 = 7.5625; // (121.0/16.0)
    const float const2 = 3.25;   // ( 13.0/ 4.0)
    const float const3 = 82.5;   // (165.0/ 2.0)

	mr.forward_diff_5 = const1*Ah_5 +  5.0*Bh_4 + const2*Ch_3 + 2.0*Dh_2 + Eh;
	mr.forward_diff_4 = const3*Ah_5 + 29.0*Bh_4 +    9.0*Ch_3 + 2.0*Dh_2;
	mr.forward_diff_3 =  255.0*Ah_5 + 48.0*Bh_4 +    6.0*Ch_3;
	mr.forward_diff_2 =  300.0*Ah_5 + 24.0*Bh_4;
	mr.forward_diff_1 =  120.0*Ah_5;

	// Calculate the initial velocity by calculating V(h/2)
	const float half_h   = h * 0.5; // h/2
    const float half_h_2 = half_h   * half_h;
    const float half_h_3 = half_h_2 * half_h;
    const float half_h_4 = half_h_3 * half_h;
    const float half_h_5 = half_h_4 * half_h;

    const float half_Eh =   E * half_h;
    const float half_Dh_2 = D * half_h_2;
	const float half_Ch_3 = C * half_h_3;
	const float half_Bh_4 = B * half_h_4;
	const float half_Ah_5 = A * half_h_5;

	mr.segment_velocity = half_Ah_5 + half_Bh_4 + half_Ch_3 + half_Dh_2 + half_Eh + v_0;
}

#endif

/*********************************************************************************************
 * _exec_aline_head()
 */

static stat_t _exec_aline_head(mpBuf_t *bf)
{
    if (mr.section_state == SECTION_NEW) {							// initialize the move singleton (mr)
        if (fp_ZERO(mr.r->head_length)) {
            mr.section = SECTION_BODY;
            return(_exec_aline_body(bf));								// skip ahead to the body generator
        }
        mr.segments = ceil(uSec(mr.r->head_time) / NOM_SEGMENT_USEC);  // # of segments for the section
        mr.segment_time = mr.r->head_time / mr.segments;
        mr.segment_count = (uint32_t)mr.segments;

        if (mr.segment_count == 1) {
            // We will only have one section, simply average the velocities, and skip to the second half
            mr.segment_velocity = (mr.entry_velocity + mr.r->cruise_velocity) / 2;
            mr.forward_diff_5 = 0; // prevent the velocity from being adjusted
            mr.section_state = SECTION_2nd_HALF;
        } else {
            _init_forward_diffs(mr.entry_velocity, mr.r->cruise_velocity, mr.entry_acceleration, mr.r->cruise_acceleration, mr.entry_jerk, mr.r->cruise_jerk, mr.r->head_time);
            mr.section_state = SECTION_1st_HALF;
        }
        if (mr.segment_time < MIN_SEGMENT_TIME) {
            while(1);
            return(STAT_MINIMUM_TIME_MOVE);                         // exit without advancing position
        }
        mr.section = SECTION_HEAD;
    }
    // For forward differencing we should have the first segment in SECTION_1st_HALF
    // However, if there was only one segment in this section it will skip the first half.
    if (mr.section_state == SECTION_1st_HALF) {						// FIRST HALF (concave part of accel curve)
        mr.section_state = SECTION_2nd_HALF;
        return(STAT_EAGAIN);
    }
    if (mr.section_state == SECTION_2nd_HALF) {						// SECOND HALF (convex part of accel curve)
        mr.segment_velocity += mr.forward_diff_5;
        if (_exec_aline_segment() == STAT_OK) { 					// set up for body
            if ((fp_ZERO(mr.r->body_length)) && (fp_ZERO(mr.r->tail_length))) {
                return(STAT_OK);                                    // ends the move
            }

            // Update executed_group_head_length in case we wish to extend the body of a group.
            mr.executed_group_head_length += mr.r->head_length;

            mr.section = SECTION_BODY;
            mr.section_state = SECTION_NEW;
        } else {

            // TODO - check for body extensions

            mr.forward_diff_5 += mr.forward_diff_4;
            mr.forward_diff_4 += mr.forward_diff_3;
            mr.forward_diff_3 += mr.forward_diff_2;
            mr.forward_diff_2 += mr.forward_diff_1;
        }
    }
    return(STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_body()
 *
 *	The body is broken into little segments even though it is a straight line so that
 *	feed holds can happen in the middle of a line with a minimum of latency
 */
static stat_t _exec_aline_body(mpBuf_t *bf)
{
    if (mr.section_state == SECTION_NEW) {
        if (fp_ZERO(mr.r->body_length - mr.executed_body_length)) {
            // We will always go from *here* to the tail.

            // Update the group data, in case we're in a block that's all body, and part of a larger body.
            // This allows us to extend a multi-block body.
            mr.executed_group_body_length += mr.r->body_length;

            mr.section = SECTION_TAIL;
            return(_exec_aline_tail());						// skip ahead to tail periods
        }

        float body_time = mr.r->body_time - mr.executed_body_time;
        mr.segments = ceil(uSec(body_time) / NOM_SEGMENT_USEC);
        mr.segment_time = body_time / mr.segments;
        mr.segment_velocity = mr.r->cruise_velocity;
        mr.segment_count = (uint32_t)mr.segments;
        if (mr.segment_time < MIN_SEGMENT_TIME) {
            while(1);
            return(STAT_MINIMUM_TIME_MOVE);                 // exit without advancing position
        }

        mr.executed_body_length = mr.r->body_length;
        mr.executed_body_time   = mr.r->body_time;

        mr.section = SECTION_BODY;
        mr.section_state = SECTION_2nd_HALF;				// uses PERIOD_2 so last segment detection works
    }
    if (mr.section_state == SECTION_2nd_HALF) {				// straight part (period 3)

        // TODO - check for body extensions

        if (_exec_aline_segment() == STAT_OK) {				// OK means this section is done
            // Try the body again, in case it's extended -- it'll jump to the tail if needed.
            mr.section = SECTION_BODY;
            mr.section_state = SECTION_NEW;
        }
    }
    return(STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_tail()
 */

static stat_t _exec_aline_tail()
{
	if (mr.section_state == SECTION_NEW) {							// INITIALIZATION
		if (fp_ZERO(mr.r->tail_length)) { return(STAT_OK);}			// end the move
		mr.segments = ceil(uSec(mr.r->tail_time) / NOM_SEGMENT_USEC);  // # of segments for the section
		mr.segment_time = mr.r->tail_time / mr.segments;			    // time to advance for each segment
        mr.segment_count = (uint32_t)mr.segments;

        if (mr.segment_count == 1) {
            // We will only have one section, simply average the velocities, and skip to the second half
            mr.segment_velocity = (mr.r->cruise_velocity + mr.r->exit_velocity) / 2;
            mr.forward_diff_5 = 0; // prevent the velocity from being adjusted
            mr.section_state = SECTION_2nd_HALF;
        } else {
            _init_forward_diffs(mr.r->cruise_velocity, mr.r->exit_velocity, mr.r->cruise_acceleration, mr.r->exit_acceleration, mr.r->cruise_jerk, mr.r->exit_jerk, mr.r->tail_time);
            mr.section_state = SECTION_1st_HALF;
        }
		if (mr.segment_time < MIN_SEGMENT_TIME) {
            while(1);
            return(STAT_MINIMUM_TIME_MOVE);                         // exit without advancing position
        }
		mr.section = SECTION_TAIL;
	}
    // For forward differencing we should have the first segment in SECTION_1st_HALF
    // However, if there was only one segment in this section it will skip the first half.
	if (mr.section_state == SECTION_1st_HALF) {						// FIRST HALF - convex part (period 4)
        mr.section_state = SECTION_2nd_HALF;
		return(STAT_EAGAIN);
	}
	if (mr.section_state == SECTION_2nd_HALF) {						// SECOND HALF - concave part (period 5)
		mr.segment_velocity += mr.forward_diff_5;
		if (_exec_aline_segment() == STAT_OK) {
			return(STAT_OK);                                        // STAT_OK completes the move
		} else {
			mr.forward_diff_5 += mr.forward_diff_4;
			mr.forward_diff_4 += mr.forward_diff_3;
			mr.forward_diff_3 += mr.forward_diff_2;
			mr.forward_diff_2 += mr.forward_diff_1;
		}
	}
	return(STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_segment() - segment runner helper
 *
 * NOTES ON STEP ERROR CORRECTION:
 *
 *	The commanded_steps are the target_steps delayed by one more segment.
 *	This lines them up in time with the encoder readings so a following error can be generated
 *
 *	The following_error term is positive if the encoder reading is greater than (ahead of)
 *	the commanded steps, and negative (behind) if the encoder reading is less than the
 *	commanded steps. The following error is not affected by the direction of movement -
 *	it's purely a statement of relative position. Examples:
 *
 *    Encoder Commanded   Following Err
 *	  	  100	     90	       +10		encoder is 10 steps ahead of commanded steps
 *	      -90	   -100	       +10		encoder is 10 steps ahead of commanded steps
 *		   90	    100	       -10		encoder is 10 steps behind commanded steps
 *	     -100	    -90	       -10		encoder is 10 steps behind commanded steps
 */

static stat_t _exec_aline_segment()
{
	float travel_steps[MOTORS];

	// Set target position for the segment
	// If the segment ends on a section waypoint synchronize to the head, body or tail end
	// Otherwise if not at a section waypoint compute target from segment time and velocity
	// Don't do waypoint correction if you are going into a hold.

	if ((--mr.segment_count == 0) && (mr.section_state == SECTION_2nd_HALF) &&
		(cm.motion_state != MOTION_HOLD)) {
		copy_vector(mr.gm.target, mr.waypoint[mr.section]);
	} else {
		float segment_length = mr.segment_velocity * mr.segment_time;
		for (uint8_t a=0; a<AXES; a++) {
			mr.gm.target[a] = mr.position[a] + (mr.unit[a] * segment_length);
		}
	}

	// Convert target position to steps
	// Bucket-brigade the old target down the chain before getting the new target from kinematics
	//
	// NB: The direct manipulation of steps to compute travel_steps only works for Cartesian kinematics.
	//	   Other kinematics may require transforming travel distance as opposed to simply subtracting steps.


	for (uint8_t m=0; m<MOTORS; m++) {
		mr.commanded_steps[m] = mr.position_steps[m];		// previous segment's position, delayed by 1 segment
		mr.position_steps[m] = mr.target_steps[m];			// previous segment's target becomes position
		mr.encoder_steps[m] = en_read_encoder(m);			// get current encoder position (time aligns to commanded_steps)
		mr.following_error[m] = mr.encoder_steps[m] - mr.commanded_steps[m];
	}
    kn_inverse_kinematics(mr.gm.target, mr.target_steps);   // now determine the target steps...
    for (uint8_t m=0; m<MOTORS; m++) {                      // and compute the distances to be traveled
        travel_steps[m] = mr.target_steps[m] - mr.position_steps[m];
    }

    // Update the mb->run_time_remaining -- we know it's missing the current segment's time before it's loaded, that's ok.
    mb.run_time_remaining -= mr.segment_time;
    if (mb.run_time_remaining < 0) {
        mb.run_time_remaining = 0.0;
    }

	// Call the stepper prep function
	ritorno(st_prep_line(travel_steps, mr.following_error, mr.segment_time));
	copy_vector(mr.position, mr.gm.target); 				// update position from target
	if (mr.segment_count == 0) {
        return (STAT_OK);			                        // this section has run all its segments
    }
	return (STAT_EAGAIN);									// this section still has more segments to run
}
