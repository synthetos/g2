/*
 * plan_zoid.cpp - acceleration managed line planning and motion execution - trapezoid planner
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
#include "planner.h"
#include "report.h"
#include "util.h"

//+++++ DIAGNOSTICS

#define LOG_RETURN(msg)                              // LOG_RETURN with no action (production)
/*
#include "xio.h"
static char logbuf[128];
static void _logger(const char *msg, const mpBuf_t *bf)         // LOG_RETURN with full state dump
{
    sprintf(logbuf, "[%2d] %s (%d) mt:%5.2f, L:%1.3f [%1.3f, %1.3f, %1.3f] V:[%1.2f, %1.2f, %1.2f]\n",
                    bf->buffer_number, msg, bf->hint, (bf->move_time * 60000),
                    bf->length, bf->head_length, bf->body_length, bf->tail_length,
                    bf->pv->exit_velocity, bf->cruise_velocity, bf->exit_velocity);
    xio_writeline(logbuf);
}
#define LOG_RETURN(msg) { _logger(msg, bf); }
*/

//#define TRAP_ZERO(t,m)
#define TRAP_ZERO(t,m) if (fp_ZERO(t)) { rpt_exception(STAT_MINIMUM_LENGTH_MOVE, m); _debug_trap(); }

//+++++ END DIAGNOSTICS

/* local functions */

//static float _get_target_length_min(const float v_0, const float v_1, const mpBuf_t *bf, const float min);
static float _get_meet_velocity(const float v_0, const float v_2, const float L, mpBuf_t *bf, mpGroupRuntimeBuf_t *rbf);

/****************************************************************************************
 * mp_calculate_ramps() - calculate trapezoid-like ramp parameters for an entire group
 * ++++ UPDATE DESCRIPTION
 *
 *  This long-ish function sets section lengths and velocities based on the move length
 *  and velocities requested. It modifies the incoming bf buffer and returns accurate
 *  head, body and tail lengths, and accurate or reasonably approximate velocities.
 *  We care about length accuracy, less so for velocity (as long as jerk is not exceeded).
 *
 *  We need velocities to be set even for zero-length sections (nb: sections, not moves)
 *  so plan_exec can compute entry and exits for adjacent sections.
 *
 *  bf values treated as constants:
 *    bf->length	            - actual block length (L)
 *	  mr.exit_velocity	    - RUNTIME Ve
 *	  bf->exit_velocity     - requested Vx
 *
 *	bf values that may be changed by plan_zoid:
 *	  bf->cruise_velocity   - requested target velocity (Vc)
 *	  bf->head_length       - bf->length allocated to head (Lh)
 *	  bf->body_length       - bf->length allocated to body (Lb)
 *	  bf->tail_length       - bf->length allocated to tail (Lt)
 *
 *	The following conditions MUST be met on entry (therefore must be validated upstream):
 *    bf->length > 0
 *    mr.exit_velocity >= 0
 *    bf->cruise_velocity >= 0
 *    bf->exit_velocity >= 0
 *    mr.exit_velocity <= bf->cruise_velocity >= bf->exit_velocity
 *    bf->move_time >= MIN_SEGMENT_TIME (expressed as cruise_vmax must not yield a patholigical segment)
 *    bf->head_length = 0
 *    bf->body_length = 0
 *    bf->tail_length = 0
 *    bf->head_time = 0
 *    bf->body_time = 0
 *    bf->tail_time = 0
 */
/*	Classes of moves:
 *
 *    Perfect-Fit - The move exactly matches the jerk profile. These may be set up
 *      by line planning and merely filled in here.
 *
 *    Requested-Fit - The move has sufficient length to achieve Vc. I.e: it will
 *      accommodate the acceleration / deceleration profile in the given length.
 *
 *    Rate-Limited-Fit - The move does not have sufficient length to achieve Vc.
 *      In this case Vc will be set lower than the requested velocity
 *      (incoming bf->cruise_velocity). Ve and Vx will be satisfied.
 */

//*************************************************************************************************
//*************************************************************************************************
//** RULE #1 of mp_calculate_trapezoid(): Don't change bf->length                                **
//** RULE #2 of mp_calculate_trapezoid(): All moves must > MIN_SEGMENT_TIME before reaching here **
//*************************************************************************************************
//*************************************************************************************************

void _zoid_exit (mpBuf_t *bf, zoidExitPoint exit_point)
{
	//+++++ DIAGNOSTIC
//    bf->zoid_exit = exit_point;
	if (mp_runtime_is_idle()) {		// normally the runtime keeps this value fresh
//		bf->time_in_plan_ms += bf->move_time_ms;
		bf->plannable_time_ms += bf->move_time_ms;
	}
}


// Hint will be one of these from back-planning: COMMAND_BLOCK, PERFECT_DECELERATION, PERFECT_CRUISE, MIXED_DECELERATION, ASYMMETRIC_BUMP
// We are incorporating both the forward planning and ramp-planning into one function, since we use the same data.
// IMPORTANT: Expects group->primary_bf to be correctly assigned.
void mp_calculate_ramps(mpGroupRuntimeBuf_t *group, const float entry_velocity)
{

    // WARNING! WARNING! WARNING!
    // This function runs against GROUPS! If you see bf->length in here, IT'S A BUG!

    // Quick cheat-sheet on which is in bf and whcih is in rbf:
    // bf:
    //  move_type
    //  hint
    //  {cruise,exit}_vmax
    //  move_time
    //  group_length
    //  (start values of {cruise,exit}_velocity)
    //
    // rg:
    //  {entry,cruise,exit}_velocity
    //  {head,body,tail}_length
    //  {head,body,tail}_time

    mpBuf_t *bf = group->primary_bf;

    // *** Skip non-move commands ***
    if (bf->move_type == MOVE_TYPE_COMMAND) {
        bf->hint = COMMAND_BLOCK;
        return;
    }
    TRAP_ZERO (bf->length, "zoid() got L=0");           //+++++ Move this outside of zoid
    TRAP_ZERO (bf->cruise_velocity, "zoid() got Vc=0"); // move this outside

    // Timing from *here*

    group->length = bf->group_length;

    // initialize parameters to know values
    group->head_time = 0;
    group->body_time = 0;
    group->tail_time = 0;

    group->head_length = 0;
    group->body_length = 0;
    group->tail_length = 0;

    group->cruise_velocity = min(bf->cruise_velocity, bf->cruise_vmax);
    group->exit_velocity   = min(bf->exit_velocity, bf->exit_vmax);

    // +++++ RG trap
    if ((group->cruise_velocity < entry_velocity)) {
        __asm__("BKPT"); // entry > cruise
    }
    if ((group->cruise_velocity < group->exit_velocity)) {
        __asm__("BKPT"); // exit > cruise
    }
    
    // Note that we are lookig at the first group/block after the running block.
    // So we can only merge *forward*.
    // We want to see if we will merge *at all*, then we look forward at each block
    // to see if it can merge into this one.
    // We are looking to merge acceleration groups. We've already made deceleration groups.
    //
    // We WON'T merge forward if the current group is:
    //   !entry_changed (still a deceleration group or limited by deceleration limits)
    //   or entry_velocity > rbf->exit_velocity (is a deceleration),
    //   or jerk >= nx_group->jerk (might degrade velocity),
    //   or computed_exit >= exit.
    //
    // Merge, then re-assess.

    bool did_merge = false;

    // We *might* do this exact computation later, so cache the value.
    float test_velocity = 0;
    bool test_velocity_valid = false; // record if we have a validly cached value

    do { // while did_merge

        did_merge = false;

        // *** Perfect-Fit Cases (1) *** Cases where curve fitting has already been done

        // PERFECT_CRUISE (1c) Velocities all match (or close enough), treat as body-only
        // NOTE: PERFECT_CRUISE is set in back-planning without knowledge of pv->exit, since it can't see it yet.
        // Here we verify it moving forward, checking to make sure it still is true.
        // If so, we plan the "ramp" as flat, body-only.
        if (bf->hint == PERFECT_CRUISE) {
            if ((!mb.entry_changed) && fp_EQ(entry_velocity, bf->cruise_vmax)) {

                // We need to ensure that neither the entry or the exit velocities are
                // <= the cruise velocity even though there is tolerance in fp_EQ comparison.
                group->exit_velocity = entry_velocity;
                group->cruise_velocity = entry_velocity;

                group->body_length = group->length;
                group->body_time = group->body_length / group->cruise_velocity;
                bf->move_time = group->body_time;

                LOG_RETURN("1c");
                return (_zoid_exit(bf, ZOID_EXIT_1c));
            } else {
                // we need to degrade the hint to MIXED_ACCELERATION
                bf->hint = MIXED_ACCELERATION;
            }
        }


        // Quick test to ensure we haven't violated the hint
        if (entry_velocity > group->exit_velocity) {
            // We're in a deceleration.
            if (mb.entry_changed) {
                // If entry_changed, then entry_velocity is lower than the hints expect.
                // A deceleration will never become an acceleration (post-hinting).
                // If it is marked as MIXED_DECELERATION, it means the entry was CRUISE_VMAX.
                // If it is marked as PERFECT_DECELERATION, it means the entry was as <= CRUISE_VMAX,
                //   but as high as possible.
                // So, this move is and was a DECELERATION, meaning it *could* achieve the previous (higher)
                //   entry safely, it will likely get a head section, so we will now degrade the hint to an
                //   ASYMMETRIC_BUMP.

                bf->hint = ASYMMETRIC_BUMP;
            }

            // MIXED_DECELERATION (2d) 2 segment BT deceleration move
            // Only possible if the entry has not changed since hinting.
            else if (bf->hint == MIXED_DECELERATION) {
                group->tail_length = mp_get_target_length(group->exit_velocity, group->cruise_velocity, bf);
                group->body_length = group->length - group->tail_length;
                group->head_length = 0;

                group->body_time = group->body_length / group->cruise_velocity;
                group->tail_time = group->tail_length*2 / (group->exit_velocity + group->cruise_velocity);
                bf->move_time = group->body_time + group->tail_time;
                LOG_RETURN("2d");
                return (_zoid_exit(bf, ZOID_EXIT_2d));
            }

            // PERFECT_DECELERATION (1d) single tail segment (deltaV == delta_vmax)
            // Only possible if the entry has not changed since hinting.
            else if (bf->hint == PERFECT_DECELERATION) {
                group->tail_length = group->length;
                group->cruise_velocity = entry_velocity;
                group->tail_time = group->tail_length*2 / (group->exit_velocity + group->cruise_velocity);
                bf->move_time = group->tail_time;
                LOG_RETURN("1d");
                return (_zoid_exit(bf, ZOID_EXIT_1d));
            }


            // Reset entry_changed. We won't likely be changing the next block's entry velocity.
            mb.entry_changed = false;


        // Since we are not generally decelerating, this is effectively all of forward planning that we need.
        } else {
            // Note that the hints from back-planning are ignored in this section, since back-planing can only predict decel and cruise.

            float accel_velocity;
            if (test_velocity_valid) {
                accel_velocity = test_velocity;
                test_velocity_valid = false;
            }
            else {
                accel_velocity = mp_get_target_velocity(entry_velocity, group->length, bf);
            }

            if (accel_velocity < group->exit_velocity) {   // still accelerating

                mb.entry_changed = true; // we are changing the *next* block's entry velocity

#if GROUPING_ENABLED == 1
                // We'll check to see if we can merge this block into the next one.
                if (bf->mergable && (bf->nx_group->buffer_state == MP_BUFFER_PREPPED) && (bf->nx_group->exit_vmax <= bf->exit_vmax)) {
                    bool will_merge = false;
                    if (bf->nx_group->jerk < bf->jerk) {
                        test_velocity = mp_get_target_velocity(entry_velocity, bf->nx_group->group_length+group->length, bf->nx_group);

                        if (test_velocity > group->exit_velocity) {
                            test_velocity_valid = true; // record that this has been computed, so it doesn't have to happen again.
                            will_merge = true;
                        }
                    } else {
                        will_merge = true;
                    }

                    if (will_merge) {
                        // We're going to merge with nx_group from us
                        // And store the results *there*, not here.
                        bf->nx_group->group_length += group->length;
                        group->length = bf->nx_group->group_length;

                        // figure out the group jerk values
                        if (bf->jerk < bf->nx_group->jerk) {
                            // Copy the move jerk, and all of it's derived values over
                            bf->nx_group->jerk = bf->jerk;
                            bf->nx_group->jerk_sq = bf->jerk_sq;
                            bf->nx_group->recip_jerk = bf->recip_jerk;
                            bf->nx_group->sqrt_j = bf->sqrt_j;
                            bf->nx_group->q_recip_2_sqrt_j = bf->q_recip_2_sqrt_j;
                        }

                        group->exit_velocity = bf->nx_group->exit_velocity;
                        group->cruise_velocity = bf->nx_group->cruise_velocity;

                        bf->hint = PART_OF_A_GROUP;

                        // We make bf now what was the nx_group, then correct the group pointers
                        bf = bf->nx_group;
                        bf->pv_group = bf->pv_group->pv_group;
                        bf->pv_group->nx_group = bf;

                        did_merge = true;
                    }
                }
#endif //GROUPING_ENABLED

                if (!did_merge) {
                    group->exit_velocity = accel_velocity;
                    group->cruise_velocity = accel_velocity;

                    bf->hint = PERFECT_ACCELERATION;

                    // PERFECT_ACCELERATION (1a) single head segment (deltaV == delta_vmax)
                    group->head_length = group->length;
                    group->cruise_velocity = group->exit_velocity;
                    group->head_time = (group->head_length*2.0) / (entry_velocity + group->cruise_velocity);
                    bf->move_time = group->head_time;
                    LOG_RETURN("1a");
                    return (_zoid_exit(bf, ZOID_EXIT_1a));
                }
            } else {                        // it's hit the cusp

                mb.entry_changed = false; // we are NOT changing the next block's entry velocity

                group->cruise_velocity = bf->cruise_vmax;

                if (group->cruise_velocity > group->exit_velocity) {

                    // We will likely have a head section, so hint the move as an ASYMMETRIC_BUMP
                    bf->hint = ASYMMETRIC_BUMP;

                } else {
                    // We know that exit_velocity is higher than cruise_vmax, so adjust it
                    group->exit_velocity = bf->cruise_vmax;

                    bf->hint = MIXED_ACCELERATION;

                    // MIXED_ACCELERATION (2a) 2 segment HB acceleration move
                    group->head_length = mp_get_target_length(entry_velocity, group->cruise_velocity, bf);
                    group->body_length = group->length - group->head_length;
                    group->tail_length = 0; // we just set it, now we unset it
                    group->head_time = (group->head_length*2.0) / (entry_velocity + group->cruise_velocity);
                    group->body_time = group->body_length / group->cruise_velocity;
                    bf->move_time = group->head_time + group->body_time;
                    LOG_RETURN("2a");
                    return (_zoid_exit(bf, ZOID_EXIT_2a));

                }
            }
        }

    } while (did_merge);

    // We've eliminated the following at this point:

    // PERFECT_ACCELERATION
    // MIXED_ACCELERATION
    // PERFECT_DECELERATION
    // MIXED_DECELERATION

    // All that remians is ASYMMETRIC_BUMP and SYMMETRIC_BUMP.
    // We don't really care if it's symmetric, since the first test that _get_meet_velocity
    //  does is for a symmetic move. It's cheaper to just let it do that then to try and prevent it.

    // *** Requested-Fit cases (2) ***

    // Prepare the head and tail lengths for evaluating cases (nb: zeros head / tail < min length)
    group->head_length = mp_get_target_length(entry_velocity, group->cruise_velocity, bf);
    group->tail_length = mp_get_target_length(group->exit_velocity, group->cruise_velocity, bf);

    if ((group->length - 0.0001) > (group->head_length + group->tail_length)) {

        // 3 segment HBT move (2c) - either with a body or just a symmetric bump
        group->body_length = group->length - (group->head_length + group->tail_length); // body guaranteed to be positive

        group->head_time = (group->head_length*2.0) / (entry_velocity + group->cruise_velocity);
        group->body_time = group->body_length / group->cruise_velocity;
        group->tail_time = (group->tail_length*2.0) / (group->exit_velocity + group->cruise_velocity);
        bf->move_time = group->head_time + group->body_time + group->tail_time;

        bf->hint = ASYMMETRIC_BUMP;

        LOG_RETURN("2c");
        return (_zoid_exit(bf, ZOID_EXIT_2c));

    }

    // *** Rate-Limited-Fit cases (3) ***
    // This means that bf->length < (bf->head_length + bf->tail_length)

    // Rate-limited asymmetric cases (3)
    // compute meet velocity to see if the cruise velocity rises above the entry and/or exit velocities
    group->cruise_velocity = _get_meet_velocity(entry_velocity, group->exit_velocity, group->length, bf, group);
    TRAP_ZERO (group->cruise_velocity, "zoid() Vc=0 asymmetric HT case");

    // We now store the head/tail lengths we computed in _get_meet_velocity.
    // treat as a full up and down (head and tail)
    bf->hint = ASYMMETRIC_BUMP;

    // Compute move times

    // save a few divides where we can
    if (fp_NOT_ZERO(group->head_length)) {
        group->head_time = (group->head_length*2.0) / (entry_velocity + group->cruise_velocity);
    }
    if (fp_NOT_ZERO(group->body_length)) {
        group->body_time = group->body_length / group->cruise_velocity;
    }
    if (fp_NOT_ZERO(group->tail_length)) {
        group->tail_time = (group->tail_length*2.0) / (group->exit_velocity + group->cruise_velocity);
    }
    bf->move_time = group->head_time + group->body_time + group->tail_time;

    LOG_RETURN("3c");
    return (_zoid_exit(bf, ZOID_EXIT_3c)); // 550us worst case
}

/****************************************************************************************
 * mp_calculate_block() - calculate the ramp parameters for a block
 * ++++ FINISH DESCRIPTION
 *
 *  This long-ish function sets section lengths and velocities based on the values for the group
 *  stored in mr, and applies them to this block. When a group is a single block, the is mostly a
 *  copy function.
 *
 *  For blocks that are part of a larger group, this function is all that's needed for all blocks
 *  but the first.
 *
 */

stat_t mp_calculate_block(mpBuf_t *bf, mpGroupRuntimeBuf_t *group, mpBlockRuntimeBuf_t *block, const float group_entry_velocity, const float entry_velocity, const float entry_acceleration, const float entry_jerk)
{
    // We look at head, then body, then tail, in order. It's possible to visit one, two, or all three in one call.
    // Note that the vmax values of the block may have been set to zero to prevent back-planning from making adjustments.
    // We MUST NOT pay attention to vmax values here.

    // Reset the values of this block
    block->head_length = 0;
    block->head_time = 0;
    block->head_t = 0;

    block->body_length = 0;
    block->body_time = 0;

    block->tail_length = 0;
    block->tail_time = 0;
    block->tail_t = 0;


    bool no_head = true;
    if (group->group_state == GROUP_HEAD) {
        bool done_with_head = false;

        float head_left = group->head_length - group->length_into_section;
        if (head_left > 0) {
            no_head = false;

            if ((bf->length + 0.0001) < head_left) {
                block->head_length = bf->length;
                group->length_into_section += bf->length;

                float t = mp_find_t(group_entry_velocity,
                                    group->cruise_velocity,
                                    group->length_into_section,
                                    group->head_length,
                                    group->t_into_section,
                                    group->head_time);
                block->cruise_velocity     = mp_calc_v(t, group_entry_velocity, group->cruise_velocity);
                block->cruise_acceleration = mp_calc_a(t, group_entry_velocity, group->cruise_velocity, group->head_time);
                block->cruise_jerk         = mp_calc_j(t, group_entry_velocity, group->cruise_velocity, group->head_time);

                // We won't enter the body or tail assignment sections, assign the exit velocities
                block->exit_velocity     = block->cruise_velocity;
                block->exit_acceleration = block->cruise_acceleration;
                block->exit_jerk         = block->cruise_jerk;

                block->head_time = group->head_time * (t - group->t_into_section);
                block->head_t = t;
                group->t_into_section = t;
            }
            // We will use of the rest of the head in this block
            else {
                block->head_length = head_left;
                block->head_time = group->head_time * (1.0 - group->t_into_section);

                // In the unlikely case where there is no body or tail, and we ended a head section exactly on a block boundary:
                block->cruise_velocity     = group->cruise_velocity;
                block->cruise_acceleration = 0;
                block->cruise_jerk         = 0;

                done_with_head = true;
            }
        }
        else {
            done_with_head = true;
        }

        if (done_with_head) {
            group->group_state = GROUP_BODY;

            // These are redundant, since they aren't used for the body, but it's noce to be thorough
            group->length_into_section = group->completed_group_body_length;
            group->t_into_section = 0.0;

            block->head_t = 1.0;
        }
    }

    if (no_head) {
        // Since there is no head section in this move, we will want to ensure the entry (runtime exit) is copied to the cruise
        // If there's a body, then it should already be at cruise and with zero accel and jerk.
        block->cruise_velocity     = entry_velocity;
        block->cruise_acceleration = entry_acceleration;
        block->cruise_jerk         = entry_jerk;
    }



    if (group->group_state == GROUP_BODY) {
        bool done_with_body = false;

        float body_left = group->body_length - group->length_into_section;
        if (body_left > 0) {
            float length_left = (bf->length - block->head_length);

            if ((length_left + 0.0001) < body_left) {
                block->body_length = length_left;
                group->length_into_section += length_left;

                // We have no tail in this block, enforce it
                block->tail_length = 0;
                block->tail_time = 0;

                // We won't enter the tail assignment section, assign the exit velocities
                block->exit_velocity     = group->cruise_velocity;
                block->exit_acceleration = 0;
                block->exit_jerk         = 0;
            }
            // We will use of the rest of the body in this block
            else {
                block->body_length = body_left;

                done_with_body = true;
            }

            block->cruise_velocity   = group->cruise_velocity;
            block->cruise_acceleration = 0;
            block->cruise_jerk         = 0;

            block->body_time = block->body_length / block->cruise_velocity;
        }
        else {
            done_with_body = true;
        }

        if (done_with_body) {
            group->group_state = GROUP_TAIL;
            
            group->length_into_section = 0.0;
            group->t_into_section = 0.0;
        }
    }



    bool no_tail = true;
    if (group->group_state == GROUP_TAIL) {
        bool done_with_tail = false;

        float tail_left = group->tail_length - group->length_into_section;
        if (tail_left > 0) {
            no_tail = false;

            float length_left = (bf->length - (block->head_length+block->body_length));
            if ((length_left + 0.0001) < tail_left) {
                block->tail_length = length_left;
                group->length_into_section += length_left;

                // mp_find_t works for decelrations as well, so we don't need to invert everything here.
                float t = mp_find_t(group->cruise_velocity,
                                    group->exit_velocity,
                                    group->length_into_section,
                                    group->tail_length,
                                    group->t_into_section,
                                    group->tail_time);
                block->exit_velocity     = mp_calc_v(t, group->cruise_velocity, group->exit_velocity);
                block->exit_acceleration = mp_calc_a(t, group->cruise_velocity, group->exit_velocity, group->tail_time);
                block->exit_jerk         = mp_calc_j(t, group->cruise_velocity, group->exit_velocity, group->tail_time);

                block->tail_time = group->tail_time * (t - group->t_into_section);
                block->tail_t = t;
                group->t_into_section = t;
            }
            // We will use of the rest of the tail in this block
            else {
                block->tail_length = tail_left;
                block->tail_time = group->tail_time * (1.0 - group->t_into_section);

                block->exit_velocity     = group->exit_velocity;
                block->exit_acceleration = 0;
                block->exit_jerk         = 0;

                done_with_tail = true;
            }
        }
        else {
            done_with_tail = true;
        }

        if (done_with_tail) {
            group->group_state = GROUP_DONE;

            group->length_into_section = 0.0;
            group->t_into_section = 0.0; // initial position for head is 0, going up to 0

            block->tail_t = 1.0;
            block->completes_group = true;
        }
    }

    if (no_tail) {
        block->exit_velocity     = block->cruise_velocity;
        block->exit_acceleration = block->cruise_acceleration;
        block->exit_jerk         = block->cruise_jerk;

        // Also, BONUS: We can't improve this group anymore!
        bf->plannable = false;
    }

    if (group->group_state == GROUP_DONE) {
        // We're officially done with this group
        return STAT_OK;
    }

    return STAT_EAGAIN;
}


/**** Planner helpers ****
 *
 * mp_get_target_length()   - find accel/decel length from delta V and jerk
 * mp_get_target_velocity() - find velocity achievable from Vi, length and jerk
 * _get_target_length_min() - find target length with correction for minimum length moves
 * _get_meet_velocity()     - find velocity at which two lines intersect
 *
 *	The get_target functions know 3 things and return the 4th:
 * 	  Jm = maximum jerk of the move
 *	  T  = time of the entire move
 *    Vi = initial velocity
 *    Vf = final velocity
 *
 *  TODO: fill in this section with Linear-Pop maths.
 */

/*
 * _get_target_length_min() - helper to get target length or return zero if length < minimum for this move
 */
//static float _get_target_length_min(const float v_0, const float v_1, const mpBuf_t *bf, const float min)
//{
//    float len = mp_get_target_length(v_0, v_1, bf);
//    if (len < min) {
//        len = 0;
//    }
//    return (len);
//}

/*
 * mp_get_target_length()   - find accel/decel length from delta V and jerk
 *
 *  mp_get_target_length cost: approx 20us unless interrupted (84MHz SAM3x8c)
 *
 *  First define some static constants. Not trusting compiler to precompile them, we precompute
 *  The naming is symbolic and verbose, but still easier to understand that "i", "j", and "k".
 *  'f' is added to the beginning when the first char would be a number.
 *
 *  Try 1 constants:
 *  L_c(v_0, v_1, j) = sqrt(5)/( sqrt(2)pow(3,4) ) * sqrt(j * abs(v_1-v_0)) * (v_0+v_1) * (1/j)
 *
 *  static const float sqrt_five = 2.23606797749979;                      // sqrt(5)
 *  static const float sqrt_two_x_fourthroot_three = 1.861209718204198;   // pow(3, 1/4) * sqrt(2)
 *
 *  Try 2 constants:
 *  L_c(v_0, v_1, j) = (sqrt(5) (v_0 + v_1) sqrt(j abs(v_1 - v_0))) / (sqrt(2) 3^(1 / 4) j)
 *
 *  Cost 6 uSec - 85 uSec - avg about 60
 *
 *  Try 3 constants:
 *  Fundamental Jerk curve formula (t=[0...1])
 *    J(t) = 60 (v_1 - v_0) (1 - t) (1 - 2t) t / T²
 *  Peak jerk is at:
 *   t = (3-sqrt(3))/6
 *   n = (1 - t) (1 - 2t) t where t = (3-sqrt(3))/6
 *   n = sqrt(3)/18
 *
 *  J(t) = 60 sqrt(3)/18 (v_1 - v_0) / T²
 *  Solve for T:
 *   T = ±(sqrt(10) sqrt(v_1-v_0))/(3^(1/4) sqrt(j)) and sqrt(j)!=0 and sqrt(v_1-v_0)!=0
 *  
 *  Define R = T², solve for R instead:
 *    R = (10/sqrt(3))((v_1-v_0)/j) and j!=0 and v_0 !=v_1
 *
 *  Fundamental Length formula:
 *    L = (sqrt(10) sqrt(v_1-v_0))/(3^(1/4) sqrt(j))( (v_1 - v_0) ((t-3) t+5/2) t⁴ + v_0 t)
 *
 *  Define m:
 *    m = ((t-3) t+5/2) t⁴
 *    L = (sqrt(10) sqrt(v_1-v_0))/(3^(1/4) sqrt(j))( (v_1 - v_0) m + v_0 t)
 *
 *  Common multiple q:
 *    q = (sqrt(10)/(3^(1/4))) ~= 2.40281141413
 *    L = q (sqrt(v_1-v_0)/sqrt(j))( (v_1 - v_0) m + v_0 t)
 *
 *  m where t = 1:
 *    m = ½
 *    t = 1
 *    L = q (sqrt(v_1-v_0)/sqrt(j))( (v_1 + v_0)/2 )
 *
 *  When j is guaranteed to be positive we can simplify some:
 *    L = q (sqrt((v_1-v_0)/j))( (v_1 + v_0)/2 )
 *    L = (q/2) (sqrt((v_1-v_0)/j)) (v_1 + v_0)
 *
 */

//Just calling this tl_constant. It's full name is:
//static const float tl_constant = 1.201405707067378;                     // sqrt(5)/( sqrt(2)pow(3,4) )

float mp_get_target_length(const float v_0, const float v_1, const mpBuf_t *bf)
{
    //const float j = bf->jerk;
    //const float recip_j = bf->recip_jerk;

    //Try 1 math:
    //return (sqrt_five * (v_0 + v_1) * sqrt(abs(v_1 - v_0) * bf->jerk))/(sqrt_two_x_fourthroot_three * bf->jerk); // newer formula -- linear POP!

    //Try 2 math (same, but rearranged):
    //float len = tl_constant * sqrt(j * abs(v_1-v_0)) * (v_0+v_1) * recip_j;
    //return (len);

    //Try 3 math:
    // time: 36us (no interrupts)
    //const float q_half = 1.2014057071; //2.40281141413/2;
    //float sqrt_delta_v_0_recip_j = sqrt(fabs(v_1-v_0)*recip_j);
    //return (q_half * sqrt_delta_v_0_recip_j * (v_1+v_0));

    //Try 4 math:
    // (q/(2 sqrt(j))) sqrt(v_1-v_0)(v_1 + v_0)
    // time: 36us - 66us (interrupted)
    const float q_recip_2_sqrt_j = bf->q_recip_2_sqrt_j;
    return q_recip_2_sqrt_j * sqrt(fabs(v_1-v_0)) * (v_1+v_0);
}

/*
 *
 * mp_get_target_velocity()
 *
 * Fundamental Jerk curve formula:
 *   J(t) = 60 (v_1 - v_0) (1 - t) (1 - 2t) t / T^2
 *
 * Fundamental Length curve formula:
 *   L(t) = T( (v_1 - v_0) ((t-3) t+5/2) t⁴ + v_0 t)
 *
 * Where t = 1;
 *   L(1) = T( (v_1 - v_0) 1/2 + v_0)
 *
 * Solve for T:
 *   T = (2 L)/(v_0 + v_1)
 *
 * Substitute T^2 = ((4L^2) / (v_0+v_1)^2) in J(t) formula
 *   J(t) = (v_1 - v_0) 60 (1 - 2t) (1 - t) t / ((4L^2) / (v_0+v_1)^2)
 *
 * Rearranged to isolate t:
 *   J(t) = (15 (1-2 t) (1-t) t (v_1-v_0) (v_0+v_1)^2)/L^2
 * Define a = (1 - 2t) (1 - t) t:
 *   J(t) = 60 (v_1 - v_0) a / T²
 *
 * At peak jerk ( where t=(3 sqrt(3))/6 ) a = 1-sqrt(3)/4

 * J(t) = ((v_0+v_1)^2) (1/(4L^2)) (v_1 - v_0) 60 a
 * Rearranged:
 * J(t) = (15 a (v_1-v_0) (v_0+v_1)^2)/L²

 * Solve for v_1:
 * v_1 = 1/3 ((4 10^(1/3) a v_0^2)/
            (80 a^3 v_0^3+9 a^2 j L^2+3 sqrt(160 a^5 j L^2 v_0^3+9 a^4 j^2 L^4))^(1/3)
        +
            (80 a^3 v_0^3+9 a^2 j L^2+3 sqrt(160 a^5 j L^2 v_0^3+9 a^4 j^2 L^4))^(1/3)
                /(10^(1/3) a))-v_0/3
        where a!=0 and L!=0

 * Define b:
 *   b = (80 a^3 v_0^3+9 a^2 j L^2+3 sqrt(160 a^5 j L^2 v_0^3+9 a^4 j^2 L^4))^(1/3)
 * Rearranged:
 *   b = (3 sqrt(a^4 j L^2 (160 a v_0^3+9 j L^2))+80 a^3 v_0^3+9 a^2 j L^2)^(1/3)
 *   b^3 = 3 sqrt(a^4 j L^2 (160 a v_0^3+9 j L^2))+80 a^3 v_0^3+9 a^2 j L^2

 * Pull out sqrt(a^4)=a^2 and sqrt(L^2)=L, and factor out a^2:
 *   b^3 = a^2 (3 L sqrt(j (160 a v_0^3+9 j L^2)) + 80 a v_0^3 + 9 j L^2)

 * Define b_part1 = 9 j L^2:
 *   b^3 = a^2 (3 L sqrt(j (160 a v_0^3 + b_part1)) + 80 a v_0^3 + b_part1)
 * Define b_part2 = 80 a v_0^3:
 *   b^3 = a^2 (3 L sqrt(j (2 b_part2 + b_part1)) + b_part2 + b_part1)

 * Using b:
 *   v_1 = 1/3 ((4 10^(1/3) a v_0^2)/b+b/(10^(1/3) a))-v_0/3 and a!=0 and P!=0
 * Rearranged:
 *   v_1 = ((4 10^(1/3) a v_0^2)/b+b/(10^(1/3) a) - v_0)/3 and a!=0 and P!=0

 * Using const1a = 4 10^(1/3) * a  and  const2a = 1/(10^(1/3) * a):
 *   v_1 = 1/3 ((const1a v_0^2)/b + b const2a - v_0)
 */

// 14 *, 1 /, 1 sqrt, 1 cbrt
// time: 68 us
float mp_get_target_velocity(const float v_0, const float L, const mpBuf_t *bf)
{
    if (fp_ZERO(L)) {                                       // handle exception case
        return (0);
    }

    const float j = bf->jerk;

    const float a80 = 7.698003589195;   // 80 * a
    const float a_2 = 0.00925925925926; // a^2

    const float v_0_2 = v_0 * v_0;      // v_0^2
    const float v_0_3 = v_0_2 * v_0;    // v_0^3

    const float L_2 = L * L;            // L^2

    const float b_part1 = 9*j*L_2;      // 9 j L^2
    const float b_part2 = a80*v_0_3;    // 80 a v_0^3

    //              b^3 = a^2 (3 L sqrt(j (2 b_part2  +  b_part1))  +  b_part2  +  b_part1)
    const float b_cubed = a_2*(3*L*sqrt(j*(2*b_part2  +  b_part1))  +  b_part2  +  b_part1);
    const float b = cbrtf(b_cubed);

    const float const1a = 0.8292422988276;    // 4 * 10^(1/3) * a
    const float const2a = 4.823680612597;     // 1/(10^(1/3) * a)
    const float const3  = 0.333333333333333;  // 1/3

    //          v_1 =    1/3 ((const1a v_0^2)/b  +  b const2a  -  v_0)
    const float v_1 = const3*((const1a*v_0_2)/b  +  b*const2a  -  v_0);
    
    return fabs(v_1);
}

/*
 * _get_meet_velocity() - find intersection velocity
 *
 * t = (3-sqrt(3))/6
 * q = (sqrt(10)/(3^(1/4)))
 * m = 4/27-1/(4 sqrt(3))
 * r = (t-m)
 * r = 85/72-2/(3 sqrt(3))
 * L = q (sqrt(v_1-v_0)/sqrt(j))( (v_1 + v_0)/2) + q (sqrt(v_1-v_2)/sqrt(j))( (v_1 + v_2)/2)
 * L = (q/(2 sqrt(j))) sqrt(v_1-v_0)(v_1 + v_0) + (q/(2 sqrt(j))) sqrt(v_1-v_2)(v_1 + v_2)
 * L = (q/(2 sqrt(j))) (sqrt(v_1-v_0)(v_1 + v_0) + sqrt(v_1-v_2)(v_1 + v_2))
 */

static float _get_meet_velocity(const float v_0, const float v_2, const float L, mpBuf_t *bf, mpGroupRuntimeBuf_t *group)
{

    // NOTE!! Do NOT use bf->lenght in here, we may be running against a group, and we'll want to use L to be safe.

    //const float j = bf->jerk;
    //const float recip_j = bf->recip_jerk;

    const float q =      2.40281141413; // (sqrt(10)/(3^(1/4)))
//    //const float q_half = 2.40281141413/2; // q/2
//
//    const float sqrt_j = sqrt(j); // 110us
//    //const float recip_sqrt_j = 1/sqrt_j;
//    const float q_recip_2_sqrt_j = q/(2*sqrt_j); // 183us

    const float sqrt_j = bf->sqrt_j;
    const float q_recip_2_sqrt_j = bf->q_recip_2_sqrt_j;

    // v_1 can never be smaller than v_0 or v_2, so we keep track of this value
    const float min_v_1 = max(v_0, v_2);

    // v_1 is our estimated return value.
    // We estimate with the speed obtained by L/2 traveled from the highest speed of v_0 or v_2.
    float v_1 = mp_get_target_velocity(min_v_1, L/2.0, bf);
    //var v_1 = min_v_1 + 100;

    if (fp_EQ(v_0, v_2)) {
        // We can catch a symmetric case early and return now

        // We'll have a head roughly equal to the tail, and no body
        group->head_length = mp_get_target_length(v_0, v_1, bf);
        group->body_length = 0;
        group->tail_length = L - group->head_length;

        bf->meet_iterations = -1;

        return v_1;
    }

    // Per iteration: 2 sqrt, 2 abs, 6 -, 4 +, 12 *, 3 /
    int i = 0; // limit the iterations // 466us - 644us
    while (i++ < 30) { // If it fails after 30, something's wrong
        if (v_1 < min_v_1) {
            // We have caught a rather nasty problem. There is no meet velocity.
            // This is does to an inversion in the velocities of very short moves.
            // We need to compute the head OR tail length, and the body will be the rest.
            // Yes, that means we're computing a cruise in here.

            v_1 = min_v_1;

            if (v_0 < v_2) {

                // acceleration - it'll be a head/body
                group->head_length = mp_get_target_length(v_0, v_2, bf);
                if (group->head_length > L) {
                    group->head_length = L;
                    group->body_length = 0;
                    v_1 = mp_get_target_velocity(v_0, L, bf);
                } else {
                    group->body_length = L - group->head_length;
                }
                group->tail_length = 0;

            } else {

                // deceleration - it'll be tail/body
                group->tail_length = mp_get_target_length(v_2, v_0, bf);
                if (group->tail_length > L) {
                    group->tail_length = L;
                    group->body_length = 0;
                    v_1 = mp_get_target_velocity(v_2, L, bf);
                } else {
                    group->body_length = L - group->tail_length;
                }
                group->head_length = 0;

            }

            break;
        }

        // Precompute some common chunks -- note that some attempts may have v_1 < v_0 or v_1 < v_2
        const float sqrt_delta_v_0 = sqrt(fabs(v_1-v_0));
        const float sqrt_delta_v_2 = sqrt(fabs(v_1-v_2)); // 849us

        // l_c is our total-length calculation with the current v_1 estimate, minus the expected length.
        // This makes l_c == 0 when v_1 is the correct value.
        //          l_c = ( q/(2 sqrt(j))) (sqrt(v_1-v_0)  (v_1 + v_0) + sqrt(v_1-v_2)  (v_1 + v_2)) - l
        //const float l_c = q_recip_2_sqrt_j*(sqrt_delta_v_0*(v_1 + v_0) + sqrt_delta_v_2*(v_1 + v_2)) - L;

        // GAMBLE: At the cost of one more multiply per iteration, we will keep the two length calculations seperate.
        // This allows us to store the resulting head/tail lengths.
        const float l_h = q_recip_2_sqrt_j*(sqrt_delta_v_0*(v_1 + v_0));
        const float l_t = q_recip_2_sqrt_j*(sqrt_delta_v_2*(v_1 + v_2));
        const float l_c = (l_h + l_t) - L;

        group->head_length = l_h;
        group->tail_length = l_t;
        group->body_length = 0;

        // Early escape -- if we're within 2 of "root" then we can call it good.
        // We need this level of precision, or out length computations fail to match the block length. // 989us
        if (fabs(l_c) < 0.00001) {
            break;
        }

        // l_d is the derivative of l_c, and is used for the Newton-Raphson iteration.
        // d = (q (sqrt(v_1-v_0) (3 v_1-v_2)-(v_0-3 v_1) sqrt(v_1-v_2)))/(4 sqrt(j) sqrt(v_1-v_0) sqrt(v_1-v_2))
        // 1/d = (4 sqrt(j) sqrt(v_1-v_0) sqrt(v_1-v_2))/(q (sqrt(v_1-v_0) (3 v_1-v_2)-(v_0-3 v_1) sqrt(v_1-v_2)))
        const float v_1x3 = 3*v_1;
        const float recip_l_d = (4*sqrt_j*sqrt_delta_v_0*sqrt_delta_v_2)/(q*(sqrt_delta_v_0*(v_1x3-v_2)-(v_0-v_1x3)*sqrt_delta_v_2));

        v_1 = v_1 - (l_c * recip_l_d);
    }

    bf->meet_iterations = i; // 509/3, 585/4, 650/5, 846/6

    //v_1 = max(min_v_1, v_1);
    // We're going to allow it to return a v_1 < (min(v_0, v_2)).

    return v_1;
}

const float FIND_T_PRECISION = 0.0001;

float mp_find_t(float v_0, float v_1, float L, float totalL, float initial_t, float T) {

    if (fabs(L-totalL) < FIND_T_PRECISION) {
        return 1.0;
    }

    if (L < FIND_T_PRECISION) {
        return 0.0;
    }

    if (fp_ZERO(initial_t) || fp_EQ(1.0, initial_t)) {
        initial_t = 0.5;
    }

    // Comput LT = Length/Time
    //const float LT = L/((2*totalL)/(v_0+v_1)); // (brute force)
    const float LT = L/T; //  (given time as T)

    const float deltaV = (v_1 - v_0);

    // The initial guess dramatically effects how many iterations are needed.
    // We'll let the caller pass in a guess, usually the previous found value.
    // Note that it's usually better to guess high.

    //var n = Math.max(0.65, L/totalL);
    // x = (sqrt(v_0^2-2 T (v_0-v_1))+v_0)/(v_0-v_1)
    //var n = (Math.sqrt(v_0*v_0-2*LT*(v_0-v_1))+v_0)/(v_1-v_0);
    float t = initial_t;// || 0.5;

    int count = 0;

    while (++count < 10) { // we'll bail after 10 iterations .. hopefully that's enough!
        float t2 = t*t;
        float t4 = t2*t2;

        // L(t) = T( (v_1 - v_0) (t² - 3 t + 5 / 2) t⁴ + v_0 t)
        // L(t) = T( (v_1 - v_0) ((t-3) t+5/2) t⁴ + v_0 t)
        float Lt = (deltaV*((t-3.0)*t+2.5)*t4 + v_0*t) - LT;
        if (fabs(Lt) < FIND_T_PRECISION) {
            break;
        }

        float t3 = t2*t;

        // V(t) = (v_1 - v_0) (6t² - 15t + 10) t³ + v_0
        // V(t) = (v_1 - v_0) (t (6 t-15)+10) t³ + v_0
        float Vt = ((v_1 - v_0)*(t*(6.0*t - 15.0) + 10.0)*t3 + v_0);

        t = t - Lt/Vt;
    }
    
    return t;
}

float mp_calc_v(const float t, const float v_0, const float v_1) {
    //V(t) = (v_1 - v_0) (6t² - 15t + 10) t³ + v_0
    //V(t) = (v_1 - v_0) (t (6 t-15)+10) t³ + v_0

    const float t3 = t * t * t;

    return (v_1-v_0) * (t*(6.0*t-15.0)+10.0)*t3 + v_0;
}

float mp_calc_a(const float t, const float v_0, const float v_1, const float T) {
    // A(t) = 30 (v_1 - v_0) (1 - t)² t² / T

    if (fp_EQ(t, 0.0) || fp_EQ(t, 1.0)) {
        return 0.0;
    }

    const float i_t    = (1.0 - t);
    const float i_t_2  = i_t*i_t;
    const float t_2    = t * t;
//    const float T = (2*L)/(v_0+v_1); // if given L, get T
    const float T_inv = 1/T;
    
    return (30.0 * (v_1 - v_0) * i_t_2 * t_2 * T_inv);
}

float mp_calc_j(const float t, const float v_0, const float v_1, const float T) {
    //J(t) = 60 (v_1 - v_0) (1 - t) (1 - 2t) t / T²

    const float i_t = (1.0 - t);
    const float i2t = (1.0 - (2.0 * t));
    const float T2_inv = 1/(T * T);

    return (60.0 * (v_1 - v_0) * i_t * i2t * t * T2_inv);
}

//float mp_calc_l(const float t, const float v_0, const float v_1, const float T) {
//    //L(t) = T( (v_1 - v_0) (t² - 3 t + 5 / 2) t⁴ + v_0 t)
//    //L(t) = T( (v_1 - v_0) ((t-3) t+5/2) t⁴ + v_0 t)
//    const float t2 = t * t;
//    const float t4 = t2 * t2;
//
//    return T*( (v_1 - v_0)*((t-3)*t+5/2)*t4 + v_0*t);
//}
