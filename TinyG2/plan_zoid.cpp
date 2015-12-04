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

static float _get_target_length_min(const float v_0, const float v_1, const mpBuf_t *bf, const float min);
static float _get_meet_velocity(const float v_0, const float v_2, const float L, mpBuf_t *bf);

/****************************************************************************************
 * mp_calculate_trapezoid() - calculate trapezoid parameters
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

#if 0 
## THIS CODE IS DISABLED
## LEFT FOR REFERENCE TEMPORARILY


// The minimum lengths are dynamic and depend on the velocity
// These expressions evaluate to the minimum lengths for the current velocity settings
// Note: The head and tail lengths are 2 minimum segments, the body is 1 min segment
#define MIN_HEAD_LENGTH (MIN_SEGMENT_TIME * (bf->cruise_velocity + mr.exit_velocity))
#define MIN_TAIL_LENGTH (MIN_SEGMENT_TIME * (bf->cruise_velocity + bf->exit_velocity))
#define MIN_BODY_LENGTH (MIN_SEGMENT_TIME *  bf->cruise_velocity)


void mp_calculate_trapezoid(mpBuf_t *bf)
{
    // *** Skip non-move commands ***
    if (bf->move_type == MOVE_TYPE_COMMAND) {
        bf->hint = COMMAND_BLOCK;
        return;
    }
    TRAP_ZERO (bf->length, "zoid() got L=0");           //+++++ Move this outside of zoid
    TRAP_ZERO (bf->cruise_velocity, "zoid() got Vc=0"); // move this outside

    // If the move has already been planned one or more times re-initialize the lengths
    // Otherwise you can end up with spurious lengths that kill accuracy
    if (bf->buffer_state == MP_BUFFER_PLANNED) {  // re-initialize parameters after 1st pass
        bf->head_time = 0;
        bf->body_time = 0;
        bf->tail_time = 0;
        bf->head_length = 0;
        bf->body_length = 0;
        bf->tail_length = 0;
    }

    // *** Perfect-Fit Cases (1) *** Cases where curve fitting has already been done

    // PERFECT_CRUISE (1c) Velocities all match (or close enough), treat as body-only
    if (bf->hint == PERFECT_CRUISE) {
        bf->body_length = bf->length;
        bf->body_time = bf->body_length / bf->cruise_velocity;
        bf->move_time = bf->body_time;
        LOG_RETURN("1c");
        return (_zoid_exit(bf, ZOID_EXIT_1c));
	}

    // PERFECT_ACCEL (1a) single head segment (deltaV == delta_vmax)
    if (bf->hint == PERFECT_ACCELERATION) {
    	bf->head_length = bf->length;
        bf->cruise_velocity = bf->exit_velocity;
        bf->head_time = bf->head_length*2 / (mr.exit_velocity + bf->cruise_velocity);
        bf->move_time = bf->head_time;
    	LOG_RETURN("1a");
        return (_zoid_exit(bf, ZOID_EXIT_1a));
    }

    // PERFECT_DECEL (1d) single tail segment (deltaV == delta_vmax)
    if (bf->hint == PERFECT_DECELERATION) {
    	bf->tail_length = bf->length;
        bf->cruise_velocity = mr.exit_velocity;
    	bf->tail_time = bf->tail_length*2 / (bf->exit_velocity + bf->cruise_velocity);
        bf->move_time = bf->tail_time;
    	LOG_RETURN("1d");
        return (_zoid_exit(bf, ZOID_EXIT_1d));
    }

    // *** Requested-Fit cases (2) ***

	// Prepare the head and tail lengths for evaluating cases (nb: zeros head / tail < min length)
    bf->head_length = _get_target_length_min(mr.exit_velocity, bf->cruise_velocity, bf, MIN_HEAD_LENGTH);
    bf->tail_length = _get_target_length_min(bf->exit_velocity, bf->cruise_velocity, bf, MIN_TAIL_LENGTH);

    if (bf->length + EPSILON > (bf->head_length + bf->tail_length)) {

        // 2 segment HB acceleration move (2a)
        if (bf->hint == MIXED_ACCELERATION &&
            VELOCITY_EQ(bf->exit_velocity, bf->cruise_velocity) &&
            VELOCITY_LT(mr.exit_velocity, bf->cruise_velocity)) {
            bf->body_length = bf->length - bf->head_length;
            bf->tail_length = 0; // we just set it, now we unset it
            bf->head_time = bf->head_length*2 / (mr.exit_velocity + bf->cruise_velocity);
            bf->body_time = bf->body_length / bf->cruise_velocity;
    	    bf->move_time = bf->head_time + bf->body_time;
            LOG_RETURN("2a");
            return (_zoid_exit(bf, ZOID_EXIT_2a));
        }

        // 2 segment BT deceleration move (2d)
        if (bf->hint == MIXED_DECELERATION &&
            VELOCITY_EQ(mr.exit_velocity, bf->cruise_velocity) &&
            VELOCITY_LT(bf->exit_velocity, bf->cruise_velocity)) {
            bf->body_length = bf->length - bf->tail_length;
            bf->head_length = 0; // we just set it, now we unset it
            bf->body_time = bf->body_length / bf->cruise_velocity;
            bf->tail_time = bf->tail_length*2 / (bf->exit_velocity + bf->cruise_velocity);
            bf->move_time = bf->body_time + bf->tail_time;
            LOG_RETURN("2d");
            return (_zoid_exit(bf, ZOID_EXIT_2d));
        }

        // 3 segment HBT move (2c) - either with a body or just a symmetric bump
        bf->body_length = bf->length - (bf->head_length + bf->tail_length); // body guaranteed to be positive
        if (bf->body_length < MIN_BODY_LENGTH) {                            // distribute to head and tail if so
            bf->head_length += bf->body_length/2;
            bf->tail_length += bf->body_length/2;
            bf->body_length = 0;
            bf->hint = SYMMETRIC_BUMP;
        }
        bf->head_time = bf->head_length*2 / (mr.exit_velocity + bf->cruise_velocity);
        bf->body_time = bf->body_length / bf->cruise_velocity;
        bf->tail_time = bf->tail_length*2 / (bf->exit_velocity + bf->cruise_velocity);
        bf->move_time = bf->head_time + bf->body_time + bf->tail_time;
//        bf->hint = SYMMETRIC_BUMP;
        LOG_RETURN("2c");
        return (_zoid_exit(bf, ZOID_EXIT_2c));
    }

    // *** Rate-Limited-Fit cases (3) ***
    // This means that bf->length < (bf->head_length + bf->tail_length)

    // Rate-limited symmetric case (3s) - rare except for single isolated moves
    // or moves between similar entry / exit velocities (e.g. Z lifts)
    if (VELOCITY_EQ(mr.exit_velocity, bf->exit_velocity)) {
        // Adjust exit velocity to be *exactly* the same to keep downstrean calculations from blowing up (exec)
        bf->exit_velocity = mr.exit_velocity;

        bf->head_length = bf->length/2;
        bf->tail_length = bf->head_length;
        bf->cruise_velocity = mp_get_target_velocity(mr.exit_velocity, bf->head_length, bf);
        TRAP_ZERO (bf->cruise_velocity, "zoid: Vc=0 symmetric case");

        if (bf->head_length < MIN_HEAD_LENGTH) {        // revert it to a single segment move
            bf->body_length = bf->length;
            bf->head_length = 0;
            bf->tail_length = 0;
            bf->body_time = bf->move_time;

            bf->cruise_velocity = mr.exit_velocity;   // set to reflect a body-only move
            bf->exit_velocity = mr.exit_velocity;     // keep velocities the same for sanity

            bf->hint = SYMMETRIC_BUMP;
            LOG_RETURN("3s2");
            return (_zoid_exit(bf, ZOID_EXIT_3s2));
        }
        // T = (2L_0) / (v_1 + v_0) + L_1 / v_1 + (2L_2) / (v_1 + v_2)
        bf->head_time = bf->head_length*2 / (mr.exit_velocity + bf->cruise_velocity);
        bf->tail_time = bf->tail_length*2 / (bf->exit_velocity + bf->cruise_velocity);
        bf->move_time = bf->head_time + bf->tail_time;
        bf->hint = SYMMETRIC_BUMP;  //+++++ Could also detect ZERO_BUMP here with more tests
        LOG_RETURN("3s");
        return (_zoid_exit(bf, ZOID_EXIT_3s));
    }

	// Rate-limited asymmetric cases (3)

    bool head_only = false;
    bool tail_only = false;

    if (bf->head_length < MIN_HEAD_LENGTH) {            // asymmetric deceleration (tail-only move) (3d)
        tail_only = true;
    } else if (bf->tail_length < MIN_TAIL_LENGTH) {     // asymmetric acceleration (head-only move) (3a)
        head_only = true;
    } else {
        // compute meet velocity to see if the cruise velocity rises above the entry and/or exit velocities
        bf->cruise_velocity = _get_meet_velocity(mr.exit_velocity, bf->exit_velocity, bf->length, bf);
        TRAP_ZERO (bf->cruise_velocity, "zoid() Vc=0 asymmetric HT case");

        if (VELOCITY_EQ(mr.exit_velocity, bf->cruise_velocity)) {
            tail_only = true;
        } else if (VELOCITY_EQ(bf->exit_velocity, bf->cruise_velocity)) {
            head_only = true;
        }
    }

    if (tail_only) {
		bf->tail_length = bf->length;
		bf->head_length = 0;
		bf->tail_time = bf->tail_length*2 / (bf->exit_velocity + bf->cruise_velocity);
		bf->move_time = bf->tail_time;
		LOG_RETURN("3d2");
        return (_zoid_exit(bf, ZOID_EXIT_3d2));
    }
    if (head_only) {
        bf->head_length = bf->length;
        bf->tail_length = 0;
        bf->head_time = bf->head_length*2 / (mr.exit_velocity + bf->cruise_velocity);
        bf->move_time = bf->head_time;
        LOG_RETURN("3a2");
        return (_zoid_exit(bf, ZOID_EXIT_3a2));
    }

    // treat as a full up and down (head and tail)
	bf->head_length = min(bf->length, _get_target_length_min(mr.exit_velocity, bf->cruise_velocity, bf, MIN_HEAD_LENGTH));
	bf->tail_length = max((float)0.0, (bf->length - bf->head_length));

    // save a few divides where we can
    if (fp_NOT_ZERO(bf->head_length)) {
        bf->head_time = bf->head_length*2 / (mr.exit_velocity + bf->cruise_velocity);
    }
    if (fp_NOT_ZERO(bf->body_length)) {
        bf->body_time = bf->body_length / bf->cruise_velocity;
    }
    if (fp_NOT_ZERO(bf->tail_length)) {
        bf->tail_time = bf->tail_length*2 / (bf->exit_velocity + bf->cruise_velocity);
    }
    bf->move_time = bf->head_time + bf->body_time + bf->tail_time;
    bf->hint = ASYMMETRIC_BUMP;
    LOG_RETURN("3c");
    return (_zoid_exit(bf, ZOID_EXIT_3c));
}

#endif # 0

// Hint will be one of these from back-planning: COMMAND_BLOCK, PERFECT_DECELERATION, PERFECT_CRUISE, MIXED_DECELERATION
// We are incorporating both the forward planning and ramp-planning into one function, since we use the same data.

void mp_calculate_ramps(mpBuf_t *bf)
{
    // *** Skip non-move commands ***
    if (bf->move_type == MOVE_TYPE_COMMAND) {
        bf->hint = COMMAND_BLOCK;
        return;
    }
    TRAP_ZERO (bf->length, "zoid() got L=0");           //+++++ Move this outside of zoid
    TRAP_ZERO (bf->cruise_velocity, "zoid() got Vc=0"); // move this outside

    // Timing from *here*

    const float entry_velocity = mr.exit_velocity;

    // If the move has already been planned one or more times re-initialize the lengths
    // Otherwise you can end up with spurious lengths that kill accuracy
    if (bf->buffer_state == MP_BUFFER_PLANNED) {  // re-initialize parameters after 1st pass
        bf->head_time = 0;
        bf->body_time = 0;
        bf->tail_time = 0;
        bf->head_length = 0;
        bf->body_length = 0;
        bf->tail_length = 0;
    }

    // *** Perfect-Fit Cases (1) *** Cases where curve fitting has already been done

    // PERFECT_CRUISE (1c) Velocities all match (or close enough), treat as body-only
    // NOTE: PERFECT_CRUISE is set in back-planning without knowledge of pv->exit, since it can't see it yet.
    // Here we verify it moving forward, checking to make sure it still is true.
    // If so, we plan the "ramp" as flat, body-only.
    if (bf->hint == PERFECT_CRUISE) {
        if ((mb.entry_changed) && fp_EQ(entry_velocity, bf->cruise_vmax)) {

            // We need to ensure that neither the entry or the exit velocities are
            // <= the cruise velocity even though there is tolerance in fp_EQ comparison.
            bf->exit_velocity = entry_velocity;
            bf->cruise_velocity = entry_velocity;

            bf->body_length = bf->length;
            bf->body_time = bf->body_length / bf->cruise_velocity;
            bf->move_time = bf->body_time;

            LOG_RETURN("1c");
            return (_zoid_exit(bf, ZOID_EXIT_1c));
        } else {
            // we need to degrade the hint to MIXED_ACCELERATION
            bf->hint = MIXED_ACCELERATION;
        }
    }


    // Quick test to ensure we haven't violated the hint
    if (entry_velocity > bf->exit_velocity) {
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
            bf->body_length = bf->length - bf->tail_length;
            bf->head_length = 0; // we just set it, now we unset it
            bf->body_time = bf->body_length / bf->cruise_velocity;
            bf->tail_time = bf->tail_length*2 / (bf->exit_velocity + bf->cruise_velocity);
            bf->move_time = bf->body_time + bf->tail_time;
            LOG_RETURN("2d");
            return (_zoid_exit(bf, ZOID_EXIT_2d));
        }

        // PERFECT_DECELERATION (1d) single tail segment (deltaV == delta_vmax)
        // Only possible if the entry has not changed since hinting.
        else if (bf->hint == PERFECT_DECELERATION) {
            bf->tail_length = bf->length;
            bf->cruise_velocity = entry_velocity;
            bf->tail_time = bf->tail_length*2 / (bf->exit_velocity + bf->cruise_velocity);
            bf->move_time = bf->tail_time;
            LOG_RETURN("1d");
            return (_zoid_exit(bf, ZOID_EXIT_1d));
        }


        // Reset entry_changed. We won't likely be changing the next block's entry velocity.
        mb.entry_changed = false;


    // Since we are not generally decelerating, this is effectively all of forward planning that we need.
    } else {


        float accel_velocity = mp_get_target_velocity(entry_velocity, bf->length, bf);

        if (accel_velocity < bf->exit_velocity) {   // still accelerating

            mb.entry_changed = true; // we are changing the *next* block's entry velocity

            bf->exit_velocity = accel_velocity;
            bf->cruise_velocity = accel_velocity;

            bf->hint = PERFECT_ACCELERATION;

            // PERFECT_ACCELERATION (1a) single head segment (deltaV == delta_vmax)
            bf->head_length = bf->length;
            bf->cruise_velocity = bf->exit_velocity;
            bf->head_time = bf->head_length*2 / (entry_velocity + bf->cruise_velocity);
            bf->move_time = bf->head_time;
            LOG_RETURN("1a");
            return (_zoid_exit(bf, ZOID_EXIT_1a));

        } else {                        // it's hit the cusp

            mb.entry_changed = false; // we are NOT changing the next block's entry velocity

            bf->cruise_velocity = bf->cruise_vmax;

            if (bf->cruise_velocity > bf->exit_velocity) {

                // We will likely had a head section, so hint the move as an ASYMMETRIC_BUMP
                bf->hint = ASYMMETRIC_BUMP;

            } else {

                /// ++++ RG WHY!?! If we don't set exit_velocity here it sometimes is HIGHER than cruise?!?
                bf->exit_velocity = min(bf->exit_vmax, bf->cruise_vmax);

                bf->hint = MIXED_ACCELERATION;

                // MIXED_ACCELERATION (2a) 2 segment HB acceleration move
                bf->head_length = mp_get_target_length(entry_velocity, bf->cruise_velocity, bf);
                bf->body_length = bf->length - bf->head_length;
                bf->tail_length = 0; // we just set it, now we unset it
                bf->head_time = bf->head_length*2 / (entry_velocity + bf->cruise_velocity);
                bf->body_time = bf->body_length / bf->cruise_velocity;
                bf->move_time = bf->head_time + bf->body_time;
                LOG_RETURN("2a");
                return (_zoid_exit(bf, ZOID_EXIT_2a));

            }
        }
    }

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
    bf->head_length = mp_get_target_length(entry_velocity, bf->cruise_velocity, bf);
    bf->tail_length = mp_get_target_length(bf->exit_velocity, bf->cruise_velocity, bf);

    if (bf->length > (bf->head_length + bf->tail_length)) {

        // 3 segment HBT move (2c) - either with a body or just a symmetric bump
        bf->body_length = bf->length - (bf->head_length + bf->tail_length); // body guaranteed to be positive
//        if (bf->body_length < MIN_BODY_LENGTH) {                            // distribute to head and tail if so
//            bf->head_length += bf->body_length/2;
//            bf->tail_length += bf->body_length/2;
//            bf->body_length = 0;
//            bf->hint = SYMMETRIC_BUMP;
//        }
        bf->head_time = bf->head_length*2 / (entry_velocity + bf->cruise_velocity);
        bf->body_time = bf->body_length / bf->cruise_velocity;
        bf->tail_time = bf->tail_length*2 / (bf->exit_velocity + bf->cruise_velocity);
        bf->move_time = bf->head_time + bf->body_time + bf->tail_time;
        //        bf->hint = SYMMETRIC_BUMP;

        LOG_RETURN("2c");
        return (_zoid_exit(bf, ZOID_EXIT_2c));

    }

    // *** Rate-Limited-Fit cases (3) ***
    // This means that bf->length < (bf->head_length + bf->tail_length)

    // Rate-limited asymmetric cases (3)
    // compute meet velocity to see if the cruise velocity rises above the entry and/or exit velocities
    bf->cruise_velocity = _get_meet_velocity(entry_velocity, bf->exit_velocity, bf->length, bf);
    TRAP_ZERO (bf->cruise_velocity, "zoid() Vc=0 asymmetric HT case");

    // We now store the head/tail lengths we computed in _get_meet_velocity.
    // treat as a full up and down (head and tail)
    //bf->head_length = mp_get_target_length(entry_velocity, bf->cruise_velocity, bf);
    //bf->tail_length = max((float)0.0, (bf->length - bf->head_length)); // max(0.0, ...) to ensure sanity

    // save a few divides where we can
    if (fp_NOT_ZERO(bf->head_length)) {
        bf->head_time = bf->head_length*2 / (entry_velocity + bf->cruise_velocity);
    }
    if (fp_NOT_ZERO(bf->body_length)) {
        bf->body_time = bf->body_length / bf->cruise_velocity;
    }
    if (fp_NOT_ZERO(bf->tail_length)) {
        bf->tail_time = bf->tail_length*2 / (bf->exit_velocity + bf->cruise_velocity);
    }
    bf->move_time = bf->head_time + bf->body_time + bf->tail_time;
    bf->hint = ASYMMETRIC_BUMP;
    
    LOG_RETURN("3c");
    return (_zoid_exit(bf, ZOID_EXIT_3c)); // 550us worst case
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
static float _get_target_length_min(const float v_0, const float v_1, const mpBuf_t *bf, const float min)
{
    float len = mp_get_target_length(v_0, v_1, bf);
    if (len < min) {
        len = 0;
    }
    return (len);
}

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

#define NEW_TARGET_VELOCITY 1
// NOTE: Get meet velocity is switched out too! They share contants.
#if NEW_TARGET_VELOCITY == 1

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

static float _get_meet_velocity(const float v_0, const float v_2, const float L, mpBuf_t *bf)
{
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
                bf->head_length = mp_get_target_length(v_0, v_2, bf);
                bf->body_length = bf->length = bf->head_length;
                bf->tail_length = 0;
            } else {
                // deceleration - it'll be tail/body
                bf->tail_length = mp_get_target_length(v_2, v_0, bf);
                bf->body_length = bf->length = bf->head_length;
                bf->head_length = 0;
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

        bf->head_length = l_h;
        bf->tail_length = l_t;
        bf->body_length = 0;

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

    v_1 = max(min_v_1, v_1);

    return v_1;
}


#else // if !NEW_TARGET_VELOCITY


// WARNING !!! OLD MATH !! //

this won't compile on purpose'

/*
 * mp_get_target_velocity()
 *
 *  Cost: ~60 - 175us, average on the high end of that
 */

static const float third = 0.333333333333333;               // 1/3 = 0.333333333333333
static const float sqrt_3 = 1.732050807568877;              // sqrt(3) = 1.732050807568877
static const float f3_sqrt_3 = 5.196152422706631;           // 3*sqrt(3) = 5.196152422706631
static const float f4_thirds_x_cbrt_5 = 2.279967928902263;  // 4/3*5^(1/3) = 2.279967928902263
static const float f1_15th_x_2_3_rt_5 = 0.194934515880858;  // 1/15*5^(2/3) = 0.194934515880858

float mp_get_target_velocity(const float v_0, const float L, const mpBuf_t *bf)
//float mp_get_target_velocity(const float v_0, const float L, const float jerk)
{
    if (fp_ZERO(L)) {                                       // handle exception case
        return (0);
    }

    // Why are these consts? So that the compiler knows they'll never change once it's computed.
    // Also, to ensure that it isn't accidentally changed once computed.

    const float j = bf->jerk;
    const float j_sq = bf->jerk_sq;                         //j^2

    const float v_0_sq = v_0 * v_0;                         //v_0^2
    const float v_0_cu = v_0 * v_0 * v_0;                   //v_0^3
    const float v_0_cu_x_40 = v_0_cu * 40;                  //v_0^3*40
    const float L_sq = L * L;                               //L^2
    const float L_fourth = L_sq * L_sq;                     //L^4
    const float L_sq_x_j_x_sqrt_3 = L_sq * j * sqrt_3;      //L^2*j*sqrt(3)

    // v_1 = 4/3*5^(1/3) *  v_0^2 /(27*sqrt(3)*L^2*j + 40*v_0^3 + 3*sqrt(3)*sqrt(80*sqrt(3)*L^2*j*v_0^3 + 81*L^4*j^2))^(1/3)
    //        + 1/15*5^(2/3)*(27*sqrt(3)*L^2*j + 40*v_0^3 + 3*sqrt(3)*sqrt(80*sqrt(3)*L^2*j*v_0^3 + 81*L^4*j^2))^(1/3)
    //        - 1/3*v_0

    // chunk_1 = pow( (27 * sqrt(3)*L^2*j     + 40 * v_0^3  + 3*sqrt(3) * sqrt(80 * v_0^3  * sqrt(3)*L^2*j     + 81 * L^4      * j^2 ) ), 1/3)
    //         = pow( (27 * L_sq_x_j_x_sqrt_3 + 40 * v_0_cu + f3_sqrt_3 * sqrt(80 * v_0_cu * L_sq_x_j_x_sqrt_3 + 81 * L_fourth * j_sq) ), third)
    //         = pow( (27 * L_sq_x_j_x_sqrt_3 + 40 * v_0_cu + f3_sqrt_3 * sqrt(2 * 40 * v_0_cu * L_sq_x_j_x_sqrt_3 + 81 * L_fourth * j_sq) ), third)
    //         = pow( (27 * L_sq_x_j_x_sqrt_3 + v_0_cu_x_40 + f3_sqrt_3 * sqrt(2 * v_0_cu_x_40 * L_sq_x_j_x_sqrt_3 + 81 * L_fourth * j_sq) ), third)

    const float chunk_1_cubed = ( 27 * L_sq_x_j_x_sqrt_3 + v_0_cu_x_40 + f3_sqrt_3 *
                                 sqrt(2 * v_0_cu_x_40 * L_sq_x_j_x_sqrt_3 + 81 * L_fourth * j_sq) );

    const float chunk_1 = cbrtf(chunk_1_cubed);

    // v_1 = 4/3*5^(1/3)        * v_0^2  / chunk_1  +   1/15*5^(2/3)       * chunk_1  -  1/3  *v_0
    // v_1 = f4_thirds_x_cbrt_5 * v_0_sq / chunk_1  +   f1_15th_x_2_3_rt_5 * chunk_1  -  third*v_0

    float v_1 = (f4_thirds_x_cbrt_5 * v_0_sq) / chunk_1  +  f1_15th_x_2_3_rt_5 * chunk_1  -  third * v_0;
    return (v_1);

}


// WARNING !!! OLD MATH !! //


/*
 * _get_meet_velocity() - find intersection velocity
 *
 *    L_d(v_0, v_1, j) = (sqrt(5) abs(v_0 - v_1) (v_0 - 3v_1)) / (2sqrt(2) 3^(1 / 4) sqrt(j abs(v_0 - v_1)) (v_0 - v_1))
 *                        sqrt(5) / (2 sqrt(2) nroot(3,4)) ( v_0 - 3 v_1) / ( sqrt(j abs(v_0 - v_1)))
 *                        sqrt(5) / (2 sqrt(2) nroot(3,4)) = 0.60070285354
 *
 * Cost: ~67us (84MHz SAM3x8c)
 */
const float mv_constant = 0.60070285354;    // sqrt(5) / (2 sqrt(2) nroot(3,4)) = 0.60070285354

static float _get_meet_velocity(const float v_0, const float v_2, const float L, const mpBuf_t *bf)
{
    const float j = bf->jerk;
    const float recip_j = bf->recip_jerk;

    float l_c_head, l_c_tail, l_c; // calculated L
    float l_d_head, l_d_tail, l_d; // calculated derivative of L with respect to v_1

    // chunks of precomputed values
    float sqrt_j_delta_v_0, sqrt_j_delta_v_1;

    // v_1 is our estimated return value.
    // We estimate with the speed obtained by L/2 traveled from the highest speed of v_0 or v_2.
    float v_1 = mp_get_target_velocity(max(v_0, v_2), L/2, bf);
    float last_v_1 = 0;

    // Per iteration: 2 sqrt, 2 abs, 6 -, 4 +, 12 *, 3 /
    int i = 0; // limit the iterations
    while (i++ < 5 && abs(last_v_1 - v_1) < 2) {    // was 10
        last_v_1 = v_1;

        // Precompute some common chunks
        sqrt_j_delta_v_0 = sqrt(j * abs(v_1-v_0));
        sqrt_j_delta_v_1 = sqrt(j * abs(v_1-v_2));

        l_c_head = tl_constant * sqrt_j_delta_v_0 * (v_0+v_1) * recip_j;
        l_c_tail = tl_constant * sqrt_j_delta_v_1 * (v_2+v_1) * recip_j;

        // l_c is our total-length calculation with the current v_1 estimate, minus the expected length.
        // This makes l_c == 0 when v_1 is the correct value.
        l_c = (l_c_head + l_c_tail) - L;

        // Early escape -- if we're within 2 of "root" then we can call it good.
        if (abs(l_c) < 2) { break; }

        // l_d is the derivative of l_c, and is used for the Newton-Raphson iteration.
        l_d_head = (mv_constant * (v_0 - 3*v_1)) / sqrt_j_delta_v_0;
        l_d_tail = (mv_constant * (v_2 - 3*v_1)) / sqrt_j_delta_v_1;
        l_d = l_d_head + l_d_tail;

        v_1 = v_1 - (l_c/l_d);
    }
    return (v_1);
}

#endif //NEW_TARGET_VELOCITY
