/*
 * plan_line.c - acceleration managed line planning and motion execution
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2012 - 2019 Rob Giseburt
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

#include "g2core.h"
#include "config.h"
#include "controller.h"
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"
#include "report.h"
#include "util.h"
#include "spindle.h"
#include "settings.h"
#include "xio.h"

// using Motate::Timeout;

// Using motate pins for profiling (see main.cpp)
// see https://github.com/synthetos/g2/wiki/Using-Pin-Changes-for-Timing-(and-light-debugging)
extern OutputPin<Motate::kDebug1_PinNumber> debug_pin1;
extern OutputPin<Motate::kDebug2_PinNumber> debug_pin2;
extern OutputPin<Motate::kDebug3_PinNumber> debug_pin3;

// planner helper functions
static mpBuf_t* _plan_block(mpBuf_t* bf);
static void _calculate_override(mpBuf_t* bf);
static void _calculate_jerk(mpBuf_t* bf);
static void _calculate_vmaxes(mpBuf_t* bf, const float axis_length[], const float axis_square[]);
static void _calculate_junction_vmax(mpBuf_t* bf);


#ifdef __PLANNER_DIAGNOSTICS
#pragma GCC push_options
#pragma GCC optimize("O0")  // this pragma is required to force the planner to actually set these unused values
static void _set_bf_diagnostics(mpBuf_t* bf) {
    UPDATE_BF_DIAGNOSTICS(bf);
}
#pragma GCC reset_options
#else
static void _set_bf_diagnostics(mpBuf_t* bf) {}
#endif

/* Runtime-specific setters and getters
 *
 * mp_zero_segment_velocity()         - correct velocity in last segment for reporting purposes
 * mp_get_runtime_velocity()          - returns current velocity (aggregate)
 * mp_get_runtime_machine_position()  - returns current axis position in machine coordinates
 * mp_set_runtime_display_offset()    - set combined display offsets in the MR struct
 * mp_get_runtime_display_position()  - returns current axis position in work display coordinates
 *                                      that were in effect at move planning time
 */

void  mp_zero_segment_velocity() { mr->segment_velocity = 0; }
float mp_get_runtime_velocity(void) { return (mr->segment_velocity); }
float mp_get_runtime_absolute_position(mpPlannerRuntime_t *_mr, uint8_t axis) { return (_mr->position[axis]); }
void mp_set_runtime_display_offset(float offset[]) { copy_vector(mr->gm.display_offset, offset); }

// We have to handle rotation - "rotate" by the transverse of the matrix to got "normal" coordinates
float mp_get_runtime_display_position(uint8_t axis) {
    // Shorthand:
    // target_rotated[0] = a x_1 + b x_2 + c x_3
    // target_rotated[1] = a y_1 + b y_2 + c y_3
    // target_rotated[2] = a z_1 + b z_2 + c z_3 + z_offset

    if (axis == AXIS_X) {
        return mr->position[0] * cm->rotation_matrix[0][0] + mr->position[1] * cm->rotation_matrix[1][0] +
               mr->position[2] * cm->rotation_matrix[2][0] - mr->gm.display_offset[0];
    } else if (axis == AXIS_Y) {
        return mr->position[0] * cm->rotation_matrix[0][1] + mr->position[1] * cm->rotation_matrix[1][1] +
               mr->position[2] * cm->rotation_matrix[2][1] - mr->gm.display_offset[1];
    } else if (axis == AXIS_Z) {
        return mr->position[0] * cm->rotation_matrix[0][2] + mr->position[1] * cm->rotation_matrix[1][2] +
               mr->position[2] * cm->rotation_matrix[2][2] - cm->rotation_z_offset - mr->gm.display_offset[2];
    } else {
        // ABC, UVW, we don't rotate them
        return (mr->position[axis] - mr->gm.display_offset[axis]);
    }
}

/****************************************************************************************
 * mp_get_runtime_busy() - returns TRUE if motion control busy (i.e. robot is moving)
 * mp_runtime_is_idle()  - returns TRUE if steppers are not actively moving
 *
 *  Use mp_get_runtime_busy() to sync to the queue. If you wait until it returns
 *  FALSE you know the queue is empty and the motors have stopped.
 */
bool mp_get_runtime_busy()
{
    if (cm->cycle_type == CYCLE_NONE) {
        return (false);
    }
    if ((st_runtime_isbusy() == true) ||
        (mr->block_state == BLOCK_ACTIVE) ||
        (mp_get_r()->buffer_state > MP_BUFFER_EMPTY)) {
        return (true);
    }
    return (false);
}

bool mp_runtime_is_idle() { return (!st_runtime_isbusy()); }

/****************************************************************************************
 * mp_aline() - plan a line with acceleration / deceleration
 *
 *  This function uses constant jerk motion equations to plan acceleration and deceleration
 *  The jerk is the rate of change of acceleration; it's the 1st derivative of acceleration,
 *  and the 3rd derivative of position. Jerk is a measure of impact to the machine.
 *  Controlling jerk smooths transitions between moves and allows for faster feeds while
 *  controlling machine oscillations and other undesirable side-effects.
 *
 *  Note: All math is done in absolute coordinates using single precision floating point (float).
 *
 *  Note: Returning a status that is not STAT_OK means the endpoint is NOT advanced. So lines
 *        that are too short to move will accumulate and get executed once the accumulated error
 *        exceeds the minimums.
 */

stat_t mp_aline(GCodeState_t* _gm)
{
    float target_rotated[]  = INIT_AXES_ZEROES;
    float axis_square[]     = INIT_AXES_ZEROES;
    float axis_length[]     = INIT_AXES_ZEROES;
    bool  flags[]           = INIT_AXES_FALSE;

    float length_square = 0;
    float length;

    // A few notes about the rotated coordinate space:
    // These are positions PRE-rotation:
    //  _gm.* (anything in _gm)
    //
    // These are positions POST-rotation:
    //  target_rotated (after the rotation here, of course)
    //  mp.* (anything in mp, including mp.gm.*)
    //
    // Shorthand:
    //  target_rotated[0] = a x_0 + b y_0 + c z_0
    //  target_rotated[1] = a x_1 + b y_1 + c z_1
    //  target_rotated[2] = a x_2 + b y_2 + c z_2 + z_offset
    //
    // With:
    //  a being target[0],
    //  b being target[1],
    //  c being target[2],
    //  x_1 being cm->rotation_matrix[1][0]

    target_rotated[AXIS_X] = _gm->target[AXIS_X] * cm->rotation_matrix[0][0] +
                             _gm->target[AXIS_Y] * cm->rotation_matrix[0][1] +
                             _gm->target[AXIS_Z] * cm->rotation_matrix[0][2];

    target_rotated[AXIS_Y] = _gm->target[AXIS_X] * cm->rotation_matrix[1][0] +
                             _gm->target[AXIS_Y] * cm->rotation_matrix[1][1] +
                             _gm->target[AXIS_Z] * cm->rotation_matrix[1][2];

    target_rotated[AXIS_Z] = _gm->target[AXIS_X] * cm->rotation_matrix[2][0] +
                             _gm->target[AXIS_Y] * cm->rotation_matrix[2][1] +
                             _gm->target[AXIS_Z] * cm->rotation_matrix[2][2] +
                             cm->rotation_z_offset;

#if (AXES == 9)
    // copy rotation axes for UVW (no changes)
    target_rotated[AXIS_U] = _gm->target[AXIS_U];
    target_rotated[AXIS_V] = _gm->target[AXIS_V];
    target_rotated[AXIS_W] = _gm->target[AXIS_W];
#endif

    // copy rotation axes for ABC (no changes)
    target_rotated[AXIS_A] = _gm->target[AXIS_A];
    target_rotated[AXIS_B] = _gm->target[AXIS_B];
    target_rotated[AXIS_C] = _gm->target[AXIS_C];

    for (uint8_t axis = 0; axis < AXES; axis++) {
        axis_length[axis] = target_rotated[axis] - mp->position[axis];
        if ((flags[axis] = fp_NOT_ZERO(axis_length[axis]))) {  // yes, this supposed to be = not ==
            axis_square[axis] = square(axis_length[axis]);
            length_square += axis_square[axis];
        } else {
            axis_length[axis] = 0;  // make it truly zero if it was tiny
            axis_square[axis] = 0;  // Fix bug that can kill feedholds by corrupting block_time in _calculate_times
        }
    }
    length = sqrt(length_square);

    // exit if the move has zero movement. At all.
//    if (length < 0.00002) {  // this value is 2x EPSILON and prevents trap failures in _plan_aline()
    if (length < 0.0001) {      // this value is 0.1 microns. Prevents planner trap failures
        sr_request_status_report(SR_REQUEST_TIMED_FULL);  // Was SR_REQUEST_IMMEDIATE_FULL
        return (STAT_MINIMUM_LENGTH_MOVE);                // STAT_MINIMUM_LENGTH_MOVE needed to end cycle
    }

    // get a cleared buffer and copy in the Gcode model state
    mpBuf_t* bf = mp_get_write_buffer();

    if (bf == NULL) {                                   // never supposed to fail
        return (cm_panic(STAT_FAILED_GET_PLANNER_BUFFER, "aline()"));
    }
    memcpy(&bf->gm, _gm, sizeof(GCodeState_t));
    copy_vector(bf->gm.target, target_rotated);         // copy the rotated target in place

    // setup the buffer
    bf->bf_func = mp_exec_aline;                        // register the callback to the exec function
    bf->length = length;                                // record the length
    for (uint8_t axis = 0; axis < AXES; axis++) {       // compute the unit vector and set flags
        if ((bf->axis_flags[axis] = flags[axis])) {     // yes, this is supposed to be = and not ==
            bf->unit[axis] = axis_length[axis] / length;// nb: bf-> unit was cleared by mp_get_write_buffer()
        }
    }
    _calculate_jerk(bf);                                // compute bf->jerk values
    _calculate_vmaxes(bf, axis_length, axis_square);    // compute cruise_vmax and absolute_vmax
    _set_bf_diagnostics(bf);                            // DIAGNOSTIC

    // Note: these next lines must remain in exact order. Position must update before committing the buffer.
    copy_vector(mp->position, bf->gm.target);           // update the planner position for the next move
    mp_commit_write_buffer(BLOCK_TYPE_ALINE);           // commit current block (must follow the position update)
    return (STAT_OK);
}

/****************************************************************************************
 * mp_plan_block_list() - plan all the blocks in the list
 *
 *  This parent function is just a dispatcher that reads forward in the list
 *  (towards the newest block) and calls the block planner as needed.
 *
 *  mp_plan_block_list() plans blocks starting at the planning block (p) and continuing
 *  until there are no more blocks to plan (see discussion of optimistic and pessimistic
 *  planning in planner.cpp/mp_plan_buffer(). The planning pass may be planning moves for
 *  the first time, or replanning moves, or any combination. Starting "early" will cause
 *  a replan, which is useful for feedholds and feed overrides.
 */

void mp_plan_block_list()
{
    mpBuf_t* bf = mp->p;

    while (bf->buffer_state != MP_BUFFER_EMPTY) {
        // OK to replan running buffer during feedhold, but no other times (not supposed to happen)
        if ((cm->hold_state == FEEDHOLD_OFF) && (bf->buffer_state == MP_BUFFER_RUNNING)) {
            mp->p = mp->p->nx;
            return;
        }
        bf = _plan_block(bf);       // returns next block to plan
        mp->p = bf;                 // DIAGNOSTIC - this is not needed but is set here for debugging purposes
    }

    if (mp->planner_state > PLANNER_STARTUP) {
        if (cm->hold_state != FEEDHOLD_HOLD) {
            st_request_forward_plan();  // start motion if runtime is not already busy
        }
    }
    mp->p = bf;  // update planner pointer - this one IS needed!
}

/****************************************************************************************
 * _plan_block() - stitch and backplan a new block to the planner queue
 */

static mpBuf_t* _plan_block(mpBuf_t* bf)
{
    // First time blocks - set vmaxes for as many blocks as possible (forward loading of priming blocks)
    // Note: cruise_vmax was computed in _calculate_vmaxes() in aline()
    if (mp->planner_state == PLANNER_PRIMING) {
        // Sometimes this part is called "stitching" - we join the moves in the order that they'll execute to find
        //   the junction velocities

        if (bf->pv->plannable) {
            // calculate junction with previous move
            if (bf->buffer_state == MP_BUFFER_INITIALIZING) {
                _calculate_junction_vmax(bf->pv);  // compute maximum junction velocity constraint - but only once
            }

            if (bf->pv->gm.path_control == PATH_EXACT_STOP) {
                bf->pv->exit_vmax = 0;
            } else {
                // bf->pv->exit_vmax = std::min(std::min(bf->pv->junction_vmax, bf->pv->cruise_vmax), bf->cruise_vmax);
                bf->pv->exit_vmax = std::min(std::min(bf->pv->junction_vmax, bf->pv->absolute_vmax), bf->absolute_vmax);
            // }
            }
        }


        if (bf->buffer_state == MP_BUFFER_INITIALIZING) {
            bf->buffer_state = MP_BUFFER_NOT_PLANNED;
            bf->hint = NO_HINT;                             // ensure we've cleared the hints
        }

        if (bf->nx->plannable) {                        // read in new buffers until EMPTY
            return (bf->nx);
        }
        mp->planning_return = bf->nx;                   // where to return after planning is complete
        mp->planner_state   = PLANNER_BACK_PLANNING;    // start backplanning
    }

    // Backward Planning Pass
    // Build a perfect deceleration ramp by setting entry and exit velocities based on the braking velocity
    // If it reaches cruise_vmax generate perfect cruises instead
    // Note: Vmax's are already set by the time you get here
    // Hint options from back-planning: COMMAND_BLOCK, PERFECT_DECELERATION, PERFECT_CRUISE, MIXED_DECELERATION

    if (mp->planner_state == PLANNER_BACK_PLANNING) {
        // NOTE: We stop when the previous block is no longer plannable.
        // We will alter the previous block's exit_velocity.
        float braking_velocity = 0;  // we use this to store the previous entry velocity, start at 0
        bool optimal = false;  // we use the optimal flag (as the opposite of plannable) to carry plan-ability backward.

        // We test for (braking_velocity < bf->exit_velocity) in case of an inversion, and plannable is then violated.
        for (; bf->plannable || (braking_velocity < bf->exit_velocity); bf = bf->pv) {
            INC_PLANNER_ITERATIONS    // DIAGNOSTIC
            bf->plannable = bf->plannable && !optimal;  // Don't accidentally enable plannable!

            // Let's be mindful that forward planning may change exit_vmax, and our exit velocity may be lowered
            braking_velocity = std::min(braking_velocity, bf->exit_vmax);

            // We *must* set cruise before exit, and keep it at least as high as exit.
            bf->cruise_velocity = std::max(braking_velocity, bf->cruise_velocity);
            bf->exit_velocity   = braking_velocity;

            // We have two places where it could be a mixed decel or an asymmetric bump,
            // depending on if the pv->exit_vmax is the same as bf.cruise_vmax
            bool test_decel_or_bump = false;

            // command blocks
            if (bf->block_type == BLOCK_TYPE_COMMAND) {
                // Nothing in the buffer before this will get any more optimal, so we'll call it
                optimal = true;

                // Update braking_velocity for use in the top of the loop
                // ++++ TEMPORARY force PTZ
                bf->exit_velocity = 0;
                // We just invalidated the next block's hint, but this should be fine

                // braking_velocity = bf->pv->exit_velocity;
                braking_velocity = 0;

                // bf->plannable = !optimal && bf->pv->plannable;
                bf->plannable = false;

                bf->hint = COMMAND_BLOCK;
            }

            // cruises - a *possible* perfect cruise is detected if exit_velocity == cruise_vmax
            // forward planning may degrade this to a mixed accel
            else if (VELOCITY_EQ(bf->exit_velocity, bf->cruise_vmax) &&
                     VELOCITY_EQ(bf->pv->exit_vmax, bf->cruise_vmax)) {
                // Remember: Set cruise FIRST
                bf->cruise_velocity = std::min(bf->cruise_vmax, bf->exit_vmax);  // set exactly to wash out EQ tolerances
                bf->exit_velocity   = bf->cruise_velocity;

                // Update braking_velocity for use in the top of the loop
                braking_velocity = bf->exit_velocity;

                bf->hint = PERFECT_CRUISE;

                // We can't improve this entry more
                optimal = true;
            }

            // not a command or a cruise
            // test to see if we'll *have* to enter slower than we can exit
            else if (bf->pv->exit_vmax < bf->exit_velocity) {
                test_decel_or_bump = true;
            }

            // Ok, now we can test deceleration cases
            else {
                braking_velocity = mp_get_target_velocity(bf->exit_velocity, bf->length, bf);

                if (bf->pv->exit_vmax > braking_velocity) {  // remember, exit vmax already is min of
                                                             // pv->cruise_vmax, cruise_vmax, and
                                                             // pv->junction_vmax
                    bf->cruise_velocity = braking_velocity;  // put this here to avoid a race condition with _exec()
                    bf->hint = PERFECT_DECELERATION;         // This is advisory, and may be altered by forward planning
                }

                else {
                    test_decel_or_bump = true;
                }
            }  // end else not a cruise

            if (test_decel_or_bump) {
                // Update braking_velocity for use in the top of the loop
                braking_velocity = bf->pv->exit_vmax;

                if (bf->cruise_vmax > bf->pv->exit_vmax) {
                    bf->cruise_velocity = bf->cruise_vmax;
                    bf->hint            = ASYMMETRIC_BUMP;
                } else {
                    bf->cruise_velocity = bf->pv->exit_vmax;
                    bf->hint            = MIXED_DECELERATION;
                    // We might still be able to merge this.
                }
                optimal = true;   // We can't improve this entry more
            }

            // Exit the loop if we've hit and passed the running buffer. It can happen.
            if (bf->buffer_state == MP_BUFFER_EMPTY) {
                break;
            }

            // if (fp_ZERO(bf->exit_velocity) && !fp_ZERO(bf->exit_vmax)) { // DIAGNOSTIC
            //     debug_trap("_plan_block(): Why is exit velocity zero?");
            // }

            // We might back plan into the running or planned buffer, so we have to check.
            if (bf->buffer_state < MP_BUFFER_BACK_PLANNED) {
                bf->buffer_state = MP_BUFFER_BACK_PLANNED;
            }
        }  // for loop
    }      // exits with bf pointing to a locked or EMPTY block

    mp->planner_state = PLANNER_PRIMING;  // revert to initial state
    return (mp->planning_return);
}

/***** ALINE HELPERS *****
 * _calculate_jerk()
 * _calculate_vmaxes()
 * _calculate_junction_vmax()
 * _calculate_decel_time()
 */

/****************************************************************************************
 * _calculate_jerk() - calculate jerk given the dynamic state
 *
 *  Set the jerk scaling to the lowest axis with a non-zero unit vector.
 *  Go through the axes one by one and compute the scaled jerk, then pick
 *  the highest jerk that does not violate any of the axes in the move.
 *
 * Cost about ~65 uSec
 */

static float _get_axis_jerk(mpBuf_t* bf, uint8_t axis)
{
    #ifdef TRAVERSE_AT_HIGH_JERK
    switch (bf->gm.motion_mode) {
        case MOTION_MODE_STRAIGHT_TRAVERSE:
            //case MOTION_MODE_STRAIGHT_PROBE: // <-- not sure on this one
            return cm->a[axis].jerk_high;
            break;
        default:
            return cm->a[axis].jerk_max;
    }
    #else
   return cm->a[axis].jerk_max;
    #endif
}

static void _calculate_jerk(mpBuf_t* bf)
{
    // compute the jerk as the largest jerk that still meets axis constraints
    bf->jerk   = 8675309;  // a ridiculously large number
    float jerk = 0;

    for (uint8_t axis = 0; axis < AXES; axis++) {
        if (std::abs(bf->unit[axis]) > 0) {  // if this axis is participating in the move
            float axis_jerk = _get_axis_jerk(bf, axis);

            jerk = axis_jerk / std::abs(bf->unit[axis]);
            if (jerk < bf->jerk) {
                bf->jerk = jerk;
                //              bf->jerk_axis = axis;           // +++ diagnostic
            }
        }
    }
    bf->jerk *= JERK_MULTIPLIER;           // goose it!
    bf->jerk_sq    = bf->jerk * bf->jerk;  // pre-compute terms used multiple times during planning
    bf->recip_jerk = 1 / bf->jerk;

    const float q        = 2.40281141413;  // (sqrt(10)/(3^(1/4)))
    const float sqrt_j   = sqrt(bf->jerk);
    bf->sqrt_j           = sqrt_j;
    bf->q_recip_2_sqrt_j = q / (2.0 * sqrt_j);
}

/****************************************************************************************
 * _calculate_vmaxes() - compute cruise_vmax and absolute_vmax based on velocity constraints
 *
 *  The following feeds and times are compared and the longest (slowest velocity) is returned:
 *      - G93 inverse time (if G93 is active)
 *      - time for coordinated move at requested feed rate
 *      - time that the slowest axis would require for the move
 *
 *  bf->block_time corresponds to bf->cruise_vmax and is either the velocity resulting from
 *  the requested feed rate or the fastest possible (minimum time) if the requested feed
 *  rate is not achievable. Move times for traverses are always the minimum time.
 *
 *  bf->absolute_vmax is the fastest the move can be executed given the velocity constraints
 *  on each participating axis - regardless of the feed rate requested. The minimum time /
 *  absolute_vmax is the time limited by the rate-limiting axis. It is saved for possible
 *  use later in feed override computation.
 *
 *  Velocities may be also be degraded (slowed down) if:
 *    - The block calls for a time that is less than the minimum update time (min segment time).
 *      This is very important to ensure proper block planning and trapezoid generation.
 *
 *  Prerequisites for calling this function:
 *    - Targets must be set via cm_set_target(). Axis modes are taken into account by this.
 *    - The unit vector and associated flags were computed.
 */
/* --- NIST RS274NGC_v3 Guidance ---
 *
 *  The following is verbatim text from NIST RS274NGC_v3. As I interpret A for moves that
 *  combine both linear and rotational movement, the feed rate should apply to the XYZ
 *  movement, with the rotational axis (or axes) timed to start and end at the same time
 *  the linear move is performed. It is possible under this case for the rotational move
 *  to rate-limit the linear move.
 *
 *   2.1.2.5 Feed Rate
 *
 *  The rate at which the controlled point or the axes move is nominally a steady rate
 *  which may be set by the user. In the Interpreter, the interpretation of the feed
 *  rate is as follows unless inverse time feed rate mode is being used in the
 *  RS274/NGC view (see Section 3.5.19). The canonical machining functions view of feed
 *  rate, as described in Section 4.3.5.1, has conditions under which the set feed rate
 *  is applied differently, but none of these is used in the Interpreter.
 *
 *    A. For motion involving one or more of the X, Y, and Z axes (with or without
 *       simultaneous rotational axis motion), the feed rate means length units per
 *       minute along the programmed XYZ path, as if the rotational axes were not moving.
 *
 *    B. For motion of one rotational axis with X, Y, and Z axes not moving, the
 *       feed rate means degrees per minute rotation of the rotational axis.
 *
 *    C. For motion of two or three rotational axes with X, Y, and Z axes not moving,
 *       the rate is applied as follows. Let dA, dB, and dC be the angles in degrees
 *       through which the A, B, and C axes, respectively, must move.
 *       Let D = sqrt(dA^2 + dB^2 + dC^2). Conceptually, D is a measure of total
 *       angular motion, using the usual Euclidean metric. Let T be the amount of
 *       time required to move through D degrees at the current feed rate in degrees
 *       per minute. The rotational axes should be moved in coordinated linear motion
 *       so that the elapsed time from the start to the end of the motion is T plus
 *       any time required for acceleration or deceleration.
 */
static void _calculate_vmaxes(mpBuf_t* bf, const float axis_length[], const float axis_square[])
{
    float feed_time = 0;        // one of: XYZ time, ABC time or inverse time. Mutually exclusive
    float max_time  = 0;        // time required for the rate-limiting axis
    float tmp_time  = 0;        // temp value used in computation
    float block_time;           // resulting move time

    // compute feed time for feeds and probe motion
    if (bf->gm.motion_mode != MOTION_MODE_STRAIGHT_TRAVERSE) {
        if (bf->gm.feed_rate_mode == INVERSE_TIME_MODE) {
            feed_time = bf->gm.feed_rate;  // NB: feed rate was un-inverted to minutes by cm_set_feed_rate()
            bf->gm.feed_rate_mode = UNITS_PER_MINUTE_MODE;
        } else {
            // compute length of linear move in millimeters. Feed rate is provided as mm/min
#if (AXES == 9)
            feed_time = sqrt(axis_square[AXIS_X] + axis_square[AXIS_Y] + axis_square[AXIS_Z] + axis_square[AXIS_U] + axis_square[AXIS_V] + axis_square[AXIS_W]) / bf->gm.feed_rate;
#else
            feed_time = sqrt(axis_square[AXIS_X] + axis_square[AXIS_Y] + axis_square[AXIS_Z]) / bf->gm.feed_rate;
#endif
            // if no linear axes, compute length of multi-axis rotary move in degrees.
            // Feed rate is provided as degrees/min
            if (fp_ZERO(feed_time)) {
                feed_time = sqrt(axis_square[AXIS_A] + axis_square[AXIS_B] + axis_square[AXIS_C]) / bf->gm.feed_rate;
            }
        }
    }

    // compute rate limits and absolute maximum limit
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (bf->axis_flags[axis]) {
            if (bf->gm.motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE) {
                tmp_time = std::abs(axis_length[axis]) / cm->a[axis].velocity_max;
            } else {// gm.motion_mode == MOTION_MODE_STRAIGHT_FEED
                tmp_time = std::abs(axis_length[axis]) / cm->a[axis].feedrate_max;
            }
            max_time = std::max(max_time, tmp_time);
        }
    }

    block_time        = std::max(max_time, MIN_BLOCK_TIME); // the slowest of most-limited axis or MIN_BLOCK_TIME
    bf->absolute_vmax = bf->length / block_time;            // absolute velocity limit - never override beyond this limit
    bf->block_time    = block_time;                         // initial estimate - used for ramp computations

    block_time        = std::max(block_time, feed_time);    // further limited by requested feedrate
    bf->cruise_vset   = bf->length / block_time;            // target velocity requested
    // bf->cruise_vmax   = bf->cruise_vset;                    // starting value for cruise vmax
    bf->cruise_vmax   = bf->absolute_vmax;                  // starting value for cruise vmax to absolute highest
}

/****************************************************************************************
 * _calculate_junction_vmax() - Giseburt's Algorithm ;-)
 *
 *  Computes the maximum allowable junction speed by finding the velocity that will not
 *  violate the jerk value of any axis. We have one tunable parameter: the time we expect
 *  or allow the corner to take over which we apply the jerk of the relevant axes. This
 *  is stored in junction_integration_time.
 *
 *  In order to achieve this we take the difference of the unit vectors of the two moves
 *  of the corner, at the point from vector a to vector b. The unit vectors of those two
 *  moves are provided as the current block (a_unit) and previous block (b_unit).
 *
 *      Delta[i]       = (b_unit[i] - a_unit[i])                   (1)
 *
 *  We want to find the velocity V[i] where, when scaled by each Delta[i], the Peak Jerk of a move
 *  that takes T time will be at (or below) the set limit for each axis, or Max Jerk. The lowest
 *  velocity of all the relevant axes is the one used.
 *
 *      MaxJerk[i] = (10/sqrt(3))*(Delta[i]*V[i])/T^2
 *
 *  Solved for V[i]:
 *
 *      V[i] = sqrt(3)/10 * MaxJerk[i] * T^2 / D[i]                (2)
 *
 *
 *  Edge cases:
 *    A) One or more axes do not change. Completely degenerate case is a straight line.
 *        We have to detect this or we'll have a divide-by-zero.
 *        To deal with this, we start at the cruise vmax, and lower from there. If nothing lowers it,
 *        then that's our cornering velocity.
 *
 *    B) Over a series of very short (length) moves that have little angular change (a highly segmented circle with
 *        a very small radius, for example) then we will not slow down sufficiently.
 *        To deal with this, we keep track of the unit vector 0.5mm back, which may be in another move.
 *        We then use that.
 *        We will use the max delta between the current vector and that vector or that of the next move.
 *
 *    C) For the last move, where there is not a next move yet, we will compute as if the "next move" has a unit vector
 *       of zero.
 */

static void _calculate_junction_vmax(mpBuf_t* bf)
{
    // (C) special case for planning the last block
    if (bf->nx->buffer_state == MP_BUFFER_EMPTY) {
        // Compute a junction velocity to full stop
        float velocity = bf->absolute_vmax;  // start with our maximum possible velocity

        for (uint8_t axis = 0; axis < AXES; axis++) {
            if (bf->axis_flags[axis]) {       // skip axes with no movement
                float delta = bf->unit[axis];

                if (delta > EPSILON) {
                    velocity = std::min(velocity, ((cm->a[axis].max_junction_accel * _get_axis_jerk(bf, axis)) / delta)); // formula (2)
                }
            }
        }

        bf->junction_vmax = velocity;
        return;
    }

    // (A) degenerate near-zero deltas to the lowest absolute_vmax of the two moves
    float velocity = std::min(bf->absolute_vmax, bf->nx->absolute_vmax);  // start with our maximum possible velocity

    // (B) special case to deal with many very short moves that are almost linear
    bool using_junction_unit = false;
    float junction_length_since = bf->junction_length_since + bf->length;
    if (junction_length_since < 0.5) {
        // push the length_since forward, and copy the junction_unit
        bf->nx->junction_length_since = junction_length_since;
        using_junction_unit = true;
    } else {
        bf->nx->junction_length_since = bf->length;
    }

    for (uint8_t axis = 0; axis < AXES; axis++) {
        if (bf->axis_flags[axis] || bf->nx->axis_flags[axis]) {       // (A) skip axes with no movement
            float delta = std::abs(bf->unit[axis] - bf->nx->unit[axis]);  // formula (1)

            if (using_junction_unit) { // (B) special case
                // use the highest delta of the two
                delta = std::max(delta, std::abs(bf->junction_unit[axis] - bf->nx->unit[axis])); // formula (1)

                // push the junction_unit for this axis into the next block, for future (B) cases
                bf->nx->junction_unit[axis] = bf->junction_unit[axis];
            } else { // prepare for future (B) cases
                // push this unit to the next junction_unit
                bf->nx->junction_unit[axis] = bf->unit[axis];
            }

            // (A) special case handling
            if (delta > EPSILON) {
                velocity = std::min(velocity, ((cm->a[axis].max_junction_accel * _get_axis_jerk(bf, axis)) / delta)); // formula (2)
            }
        }
    }
    bf->junction_vmax = velocity;
}
