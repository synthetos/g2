/*
 * plan_line.c - acceleration managed line planning and motion execution
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2017 Alden S. Hart, Jr.
 * Copyright (c) 2012 - 2017 Rob Giseburt
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
using namespace Motate;
extern OutputPin<kDebug1_PinNumber> debug_pin1;
extern OutputPin<kDebug2_PinNumber> debug_pin2;
extern OutputPin<kDebug3_PinNumber> debug_pin3;

// planner helper functions
static mpBuf_t* _plan_block(mpBuf_t* bf);
static void _calculate_override(mpBuf_t* bf);
static void _calculate_jerk(mpBuf_t* bf);
static void _calculate_vmaxes(mpBuf_t* bf, const float axis_length[], const float axis_square[]);
static void _calculate_junction_vmax(mpBuf_t* bf);

//+++++DIAGNOSTICS
#pragma GCC optimize("O0")  // this pragma is required to force the planner to actually set these unused values
//#pragma GCC reset_options
static void _set_bf_diagnostics(mpBuf_t* bf) {
    bf->linenum = bf->gm.linenum;
//  UPDATE_BF_DIAGNOSTICS(bf);   //+++++
}
#pragma GCC reset_options

/* Runtime-specific setters and getters
 *
 * mp_zero_segment_velocity()         - correct velocity in last segment for reporting purposes
 * mp_get_runtime_velocity()          - returns current velocity (aggregate)
 * mp_get_runtime_machine_position()  - returns current axis position in machine coordinates
 * mp_set_runtime_work_offset()       - set offsets in the MR struct
 * mp_get_runtime_work_position()     - returns current axis position in work coordinates
 *                                      that were in effect at move planning time
 */

void  mp_zero_segment_velocity() { mr.segment_velocity = 0; }
float mp_get_runtime_velocity(void) { return (mr.segment_velocity); }
float mp_get_runtime_absolute_position(uint8_t axis) { return (mr.position[axis]); }
void mp_set_runtime_work_offset(float offset[]) { copy_vector(mr.gm.work_offset, offset); }

// We have to handle rotation - "rotate" by the transverse of the matrix to got "normal" coordinates
float mp_get_runtime_work_position(uint8_t axis) {
    // Shorthand:
    // target_rotated[0] = a x_1 + b x_2 + c x_3
    // target_rotated[1] = a y_1 + b y_2 + c y_3
    // target_rotated[2] = a z_1 + b z_2 + c z_3 + z_offset

    if (axis == AXIS_X) {
        return mr.position[0] * cm.rotation_matrix[0][0] + mr.position[1] * cm.rotation_matrix[1][0] +
               mr.position[2] * cm.rotation_matrix[2][0] - mr.gm.work_offset[0];
    } else if (axis == AXIS_Y) {
        return mr.position[0] * cm.rotation_matrix[0][1] + mr.position[1] * cm.rotation_matrix[1][1] +
               mr.position[2] * cm.rotation_matrix[2][1] - mr.gm.work_offset[1];
    } else if (axis == AXIS_Z) {
        return mr.position[0] * cm.rotation_matrix[0][2] + mr.position[1] * cm.rotation_matrix[1][2] +
               mr.position[2] * cm.rotation_matrix[2][2] - cm.rotation_z_offset - mr.gm.work_offset[2];
    } else {
        // ABC, UVW, we don't rotate them
        return (mr.position[axis] - mr.gm.work_offset[axis]);
    }
}

/*
 * mp_get_runtime_busy() - returns TRUE if motion control busy (i.e. robot is moving)
 * mp_runtime_is_idle()  - returns TRUE is steppers are not actively moving
 *
 *  Use mp_get_runtime_busy() to sync to the queue. If you wait until it returns
 *  FALSE you know the queue is empty and the motors have stopped.
 */
bool mp_get_runtime_busy() 
{
    if (cm.cycle_state == CYCLE_OFF) {
        return (false);
    }
    if ((st_runtime_isbusy() == true) || (mr.block_state == BLOCK_ACTIVE) || (mb.r->buffer_state > MP_BUFFER_EMPTY)) {
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
 *  Note All math is done in absolute coordinates using single precision floating point (float).
 *
 *  Note: Returning a status that is not STAT_OK means the endpoint is NOT advanced. So lines
 *        that are too short to move will accumulate and get executed once the accumulated error
 *        exceeds the minimums.
 */

stat_t mp_aline(GCodeState_t* gm_in) 
{
    mpBuf_t* bf;  // current move pointer
    float target_rotated[AXES] = {0, 0, 0, 0, 0, 0};
    float axis_square[AXES]    = {0, 0, 0, 0, 0, 0};
    float axis_length[AXES];
    bool  flags[AXES];
    float length_square = 0;
    float length;

    // A few notes about the rotated coordinate space:
    // These are positions PRE-rotation:
    //  gm_in.* (anything in gm_in)
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
    //  x_1 being cm.rotation_matrix[1][0]

    target_rotated[0] = gm_in->target[0] * cm.rotation_matrix[0][0] + 
                        gm_in->target[1] * cm.rotation_matrix[0][1] +
                        gm_in->target[2] * cm.rotation_matrix[0][2];

    target_rotated[1] = gm_in->target[0] * cm.rotation_matrix[1][0] + 
                        gm_in->target[1] * cm.rotation_matrix[1][1] +
                        gm_in->target[2] * cm.rotation_matrix[1][2];

    target_rotated[2] = gm_in->target[0] * cm.rotation_matrix[2][0] + 
                        gm_in->target[1] * cm.rotation_matrix[2][1] +
                        gm_in->target[2] * cm.rotation_matrix[2][2] + 
                        cm.rotation_z_offset;

    // copy rotation axes ABC
    target_rotated[3] = gm_in->target[3];
    target_rotated[4] = gm_in->target[4];
    target_rotated[5] = gm_in->target[5];

    for (uint8_t axis = 0; axis < AXES; axis++) {
        axis_length[axis] = target_rotated[axis] - mp.position[axis];
        if ((flags[axis] = fp_NOT_ZERO(axis_length[axis]))) {  // yes, this supposed to be = not ==
            axis_square[axis] = square(axis_length[axis]);
            length_square += axis_square[axis];
        } else {
            axis_length[axis] = 0;  // make it truly zero if it was tiny
        }
    }
    length = sqrt(length_square);

    // exit if the move has zero movement. At all.
    if (fp_ZERO(length)) {
        sr_request_status_report(SR_REQUEST_TIMED_FULL);// Was SR_REQUEST_IMMEDIATE_FULL
        return (STAT_MINIMUM_LENGTH_MOVE);              // STAT_MINIMUM_LENGTH_MOVE needed to end cycle
    }

    // get a cleared buffer and copy in the Gcode model state
    if ((bf = mp_get_write_buffer()) == NULL) {         // never supposed to fail
        return (cm_panic(STAT_FAILED_GET_PLANNER_BUFFER, "aline()"));
    }
    memcpy(&bf->gm, gm_in, sizeof(GCodeState_t));
    // Since bf->gm.target is being used all over the place, we'll make it the rotated target
    copy_vector(bf->gm.target, target_rotated);  // copy the rotated taget in place

    // setup the buffer
    bf->bf_func = mp_exec_aline;                        // register the callback to the exec function
    bf->length  = length;                               // record the length
    for (uint8_t axis = 0; axis < AXES; axis++) {       // compute the unit vector and set flags
        if ((bf->axis_flags[axis] = flags[axis])) {     // yes, this is supposed to be = and not ==
            bf->unit[axis] = axis_length[axis] / length;// nb: bf-> unit was cleared by mp_get_write_buffer()
        }
    }
    _calculate_jerk(bf);                              // compute bf->jerk values
    _calculate_vmaxes(bf, axis_length, axis_square);  // compute cruise_vmax and absolute_vmax
    _set_bf_diagnostics(bf);                          //+++++DIAGNOSTIC

    // Note: these next lines must remain in exact order. Position must update before committing the buffer.
    copy_vector(mp.position, bf->gm.target);   // set the planner position
    mp_commit_write_buffer(BLOCK_TYPE_ALINE);  // commit current block (must follow the position update)
    return (STAT_OK);
}

/*
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
    mpBuf_t* bf                = mp.p;
    bool     planned_something = false;

    while (true) {
        // unconditional exit condition
        if (bf->buffer_state == MP_BUFFER_EMPTY) {
            break;
        }

        // OK to replan running buffer during feedhold, but no other times (not supposed to happen)
        if ((cm.hold_state == FEEDHOLD_OFF) && (bf->buffer_state == MP_BUFFER_RUNNING)) {
            mp.p = mp.p->nx;
            return;
        }

        bf = _plan_block(bf);  // returns next block to plan

        planned_something = true;
        mp.p              = bf;  //+++++ DIAGNOSTIC - this is not needed but is set here for debugging purposes
    }
    if (mp.planner_state > PLANNER_STARTUP) {
        if (planned_something && (cm.hold_state != FEEDHOLD_HOLD)) {
            st_request_forward_plan();  // start motion if runtime is not already busy
        }
    }
    mp.p = bf;  // update planner pointer
}

/*
 * _plan_block() - the block chain using pessimistic assumptions
 */

static mpBuf_t* _plan_block(mpBuf_t* bf) 
{
    // First time blocks - set vmaxes for as many blocks as possible (forward loading of priming blocks)
    // Note: cruise_vmax was computed in _calculate_vmaxes() in aline()
    if (mp.planner_state == PLANNER_PRIMING) {
        // Timings from *here*

        if (bf->pv->plannable) {
            _calculate_junction_vmax(bf->pv);  // compute maximum junction velocity constraint
            if (bf->pv->gm.path_control == PATH_EXACT_STOP) {
                bf->pv->exit_vmax = 0;
            } else {
                bf->pv->exit_vmax = min3(bf->pv->junction_vmax, bf->pv->cruise_vmax, bf->cruise_vmax);
            }
        }
        _calculate_override(bf);  // adjust cruise_vmax for feed/traverse override
 //     bf->plannable_time = bf->pv->plannable_time;    // set plannable time - excluding current move
        bf->buffer_state = MP_BUFFER_IN_PROCESS;

        // +++++ Why do we have to do this here?
        // bf->pv_group = bf->pv;

        bf->hint = NO_HINT;     // ensure we've cleared the hints
        // Time: 12us-41us
        if (bf->nx->plannable) {  // read in new buffers until EMPTY
            return (bf->nx);
        }
        mp.planning_return = bf->nx;                 // where to return after planning is complete
        mp.planner_state   = PLANNER_BACK_PLANNING;  // start backplanning
    }

    // Backward Planning Pass
    // Build a perfect deceleration ramp by setting entry and exit velocities based on the braking velocity
    // If it reaches cruise_vmax generate perfect cruises instead
    // Note: Vmax's are already set by the time you get here
    // Hint options from back-planning: COMMAND_BLOCK, PERFECT_DECELERATION, PERFECT_CRUISE, MIXED_DECELERATION

    if (mp.planner_state == PLANNER_BACK_PLANNING) {
        // NOTE: We stop when the previous block is no longer plannable.
        // We will alter the previous block's exit_velocity.
        float braking_velocity = 0;  // we use this to stre the previous entry velocity, start at 0
        bool optimal = false;  // we use the optimal flag (as the opposite of plannable) to carry plan-ability backward.

        // We test for (braking_velocity < bf->exit_velocity) in case of an inversion, and plannable is then violated.
        for (; bf->plannable || (braking_velocity < bf->exit_velocity); bf = bf->pv) {
            // Timings from *here*

            bf->iterations++;
            bf->plannable = bf->plannable && !optimal;  // Don't accidentally enable plannable!

            // Let's be mindful that forward planning may change exit_vmax, and our exit velocity may be lowered
            braking_velocity = min(braking_velocity, bf->exit_vmax);

            // We *must* set cruise before exit, and keep it at least as high as exit.
            bf->cruise_velocity = max(braking_velocity, bf->cruise_velocity);
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

                // Time: XXXus (was 7us)
            }

            // cruises - a *possible* perfect cruise is detected if exit_velocity == cruise_vmax
            // forward planning may degrade this to a mixed accel
            else if (VELOCITY_EQ(bf->exit_velocity, bf->cruise_vmax) &&
                     VELOCITY_EQ(bf->pv->exit_vmax, bf->cruise_vmax)) {
                // Remember: Set cruise FIRST
                bf->cruise_velocity = min(bf->cruise_vmax, bf->exit_vmax);  // set exactly to wash out EQ tolerances
                bf->exit_velocity   = bf->cruise_velocity;

                // Update braking_velocity for use in the top of the loop
                braking_velocity = bf->exit_velocity;

                bf->hint = PERFECT_CRUISE;

                // We can't improve this entry more
                optimal = true;

                // Time: XXXus (was 21us-27us)
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
                                                             // pv_group->cruise_vmax, cruise_vmax, and
                                                             // pv_group->junction_vmax
                    bf->cruise_velocity = braking_velocity;  // put this here to avoid a race condition with _exec()
                    bf->hint = PERFECT_DECELERATION;         // This is advisory, and may be altered by forward planning

                    // Time: XXXus (was 71us-78us)
                }

                else {
                    test_decel_or_bump = true;
                    // Time: XXXus (was 72us-79us)
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

            // +++++
            if (bf->buffer_state == MP_BUFFER_EMPTY) {
            //     _debug_trap("Exec apparently cleared this block while we were planning it.");
                break;  // exit the loop, we've hit and passed the running buffer
            }
            // if (fp_ZERO(bf->exit_velocity) && !fp_ZERO(bf->exit_vmax)) {
            //     _debug_trap(); // why zero?
            // }

            // We might back plan into the running or planned buffer, so we have to check.
            if (bf->buffer_state < MP_BUFFER_PREPPED) {
                bf->buffer_state = MP_BUFFER_PREPPED;
            }
        }  // for loop
    }      // exits with bf pointing to a locked or EMPTY block

    mp.planner_state = PLANNER_PRIMING;  // revert to initial state
    return (mp.planning_return);
}

/***** ALINE HELPERS *****
 * _calculate_override() - calculate cruise_vmax given cruise_vset and feed rate factor
 * _calculate_jerk()
 * _calculate_vmaxes()
 * _calculate_junction_vmax()
 * _calculate_decel_time()
 */

static void _calculate_override(mpBuf_t* bf)  // execute ramp to adjust cruise velocity
{
    // TODO: Account for rapid overrides as well as feed overrides

    // pull in override factor from previous block or seed initial value from the system setting
    bf->override_factor = fp_ZERO(bf->pv->override_factor) ? cm.gmx.mfo_factor : bf->pv->override_factor;
    bf->cruise_vmax     = bf->override_factor * bf->cruise_vset;

    // generate ramp term is a ramp is active
    if (mp.ramp_active) {
        bf->override_factor += mp.ramp_dvdt * bf->block_time;
        if (mp.ramp_dvdt > 0) {  // positive is an acceleration ramp
            if (bf->override_factor > mp.ramp_target) {
                bf->override_factor = mp.ramp_target;
                mp.ramp_active      = false;  // detect end of ramp
            }
            bf->cruise_velocity *= bf->override_factor;
            if (bf->cruise_velocity > bf->absolute_vmax) {  // test max cruise_velocity
                bf->cruise_velocity = bf->absolute_vmax;
                mp.ramp_active      = false;  // don't allow exceeding absolute_vmax
            }
        } else {  // negative is deceleration ramp
            if (bf->override_factor < mp.ramp_target) {
                bf->override_factor = mp.ramp_target;
                mp.ramp_active      = false;
            }
            bf->cruise_velocity *= bf->override_factor;  // +++++ this is probably wrong
            //  bf->exit_velocity *= bf->mfo_factor;        //...but I'm not sure this is right,
            //  bf->cruise_velocity = bf->pv->exit_velocity;   //...either
        }
    } else {
        bf->cruise_velocity *= bf->override_factor;  // apply original or changed factor
    }
    // Correction for velocity constraints
    // In the case of a acceleration these conditions must hold:
    //      Ve < Vc = Vx
    // In the case of a deceleration:
    //      Ve = Vc > Vx
    // in the case of "lump":
    //      Ve < Vc > Vx
    // if (bf->cruise_velocity < bf->pv->exit_velocity) { // deceleration case
    //     bf->cruise_velocity = bf->pv->exit_velocity;
    // } else {                                        // acceleration case
    //     ...
    // }
}

/*
 * _calculate_jerk() - calculate jerk given the dynamic state
 *
 *  Set the jerk scaling to the lowest axis with a non-zero unit vector.
 *  Go through the axes one by one and compute the scaled jerk, then pick
 *  the highest jerk that does not violate any of the axes in the move.
 *
 * Cost about ~65 uSec
 */

static void _calculate_jerk(mpBuf_t* bf) 
{
    // compute the jerk as the largest jerk that still meets axis constraints
    bf->jerk   = 8675309;  // a ridiculously large number
    float jerk = 0;

    for (uint8_t axis = 0; axis < AXES; axis++) {
        if (fabs(bf->unit[axis]) > 0) {  // if this axis is participating in the move
            float axis_jerk = 0;
#ifdef TRAVERSE_AT_HIGH_JERK
#warning using experimental feature TRAVERSE_AT_HIGH_JERK!
            switch (bf->gm.motion_mode) {
                case MOTION_MODE_STRAIGHT_TRAVERSE:
                //case MOTION_MODE_STRAIGHT_PROBE: // <-- not sure on this one
                    axis_jerk = cm.a[axis].jerk_high;
                    break;
                default:
                    axis_jerk = cm.a[axis].jerk_max;
            }
#else
            axis_jerk = cm.a[axis].jerk_max;
#endif

            jerk = axis_jerk / fabs(bf->unit[axis]);
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
    bf->q_recip_2_sqrt_j = q / (2 * sqrt_j);
}

/*
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
    float min_time  = 8675309;  // looking for fastest possible execution (seed w/arbitrarily large number)
    float block_time;           // resulting move time

    // compute feed time for feeds and probe motion
    if (bf->gm.motion_mode != MOTION_MODE_STRAIGHT_TRAVERSE) {
        if (bf->gm.feed_rate_mode == INVERSE_TIME_MODE) {
            feed_time             = bf->gm.feed_rate;  // NB: feed rate was un-inverted to minutes by cm_set_feed_rate()
            bf->gm.feed_rate_mode = UNITS_PER_MINUTE_MODE;
        } else {
            // compute length of linear move in millimeters. Feed rate is provided as mm/min
            feed_time = sqrt(axis_square[AXIS_X] + axis_square[AXIS_Y] + axis_square[AXIS_Z]) / bf->gm.feed_rate;
            // if no linear axes, compute length of multi-axis rotary move in degrees. Feed rate is provided as
            // degrees/min
            if (fp_ZERO(feed_time)) {
                feed_time = sqrt(axis_square[AXIS_A] + axis_square[AXIS_B] + axis_square[AXIS_C]) / bf->gm.feed_rate;
            }
        }
    }
    // compute rate limits and absolute maximum limit
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (bf->axis_flags[axis]) {
            if (bf->gm.motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE) {
                tmp_time = fabs(axis_length[axis]) / cm.a[axis].velocity_max;
            } else {  // gm.motion_mode == MOTION_MODE_STRAIGHT_FEED
                tmp_time = fabs(axis_length[axis]) / cm.a[axis].feedrate_max;
            }
            max_time = max(max_time, tmp_time);

            if (tmp_time > 0) {  // collect minimum time if this axis is not zero
                min_time = min(min_time, tmp_time);
            }
        }
    }
    block_time        = max3(feed_time, max_time, MIN_BLOCK_TIME);
    min_time          = max(min_time, MIN_BLOCK_TIME);
    bf->cruise_vset   = bf->length / block_time;  // target velocity requested
    bf->cruise_vmax   = bf->cruise_vset;          // starting value for cruise vmax
    bf->absolute_vmax = bf->length / min_time;    // absolute velocity limit
    bf->block_time    = block_time;               // initial estimate - used for ramp computations
}

/*
 * _calculate_junction_vmax() - Giseburt's Algorithm ;-)
 *
 *  WARNING: This description is out of date and needs updated.
 *
 *  Computes the maximum allowable junction speed by finding the velocity that will not
 *  violate the jerk value of any axis.
 *
 *  In order to achieve this we take the difference of the unit vectors of the two moves
 *  of the corner, at the point from vector a to vector b. The unit vectors of those two
 *  moves are provided as the current block (a_unit) and previous block (b_unit).
 *
 *      Delta[i]       = (b_unit[i] - a_unit[i])                   (1)
 *
 *  We take, axis by axis, the difference in "unit velocity" to get a vector that
 *  represents the direction of acceleration - which may be the opposite direction
 *  as that of the "a" vector to achieve deceleration. To get the actual acceleration,
 *  we use the corner velocity (what we intend to calculate) as the magnitude.
 *
 *      Acceleration[i] = UnitAccel[i] * Velocity[i]               (2)
 *
 *  Since we need the jerk value, which is defined as the "rate of change of acceleration,
 *  that is, the derivative of acceleration with respect to time" (Wikipedia), we need to
 *  have a quantum of time where the change in acceleration is actually carried out by the
 *  physics. That will give us the time over which to "apply" the change of acceleration
 *  in order to get a physically realistic jerk. The yields a fairly simple formula:
 *
 *      Jerk[i] = Acceleration[i] / Time                           (3)
 *
 *  Now that we can compute the jerk for a given corner, we need to know the maximum
 *  velocity that we can take the corner without violating that jerk for any axis.
 *  Let's incorporate formula (2) into formula (3), and solve for Velocity, using
 *  the known max Jerk and UnitAccel for this corner:
 *
 *      Velocity[i] = (Jerk[i] * Time) / UnitAccel[i]              (4)
 *
 *  We then compute (4) for each axis, and use the smallest (most limited) result or
 *  vmax, whichever is smaller.
 */
/* Note 1:
 *  junction_integration_time is the integration Time quantum expressed in minutes.
 *  This is roughly on the order of 1 DDA clock tick to integrate jerk to acceleration.
 *  This is a very small number, so we multiply JT by 1,000,000 for entry and display.
 *  A reasonable JA is therefore between 0.10 and 2.0
 *
 *  In formula 4 the jerk is multiplied by 1,000,000 and JT is divided by 1,000,000,
 *  so those terms cancel out.
 */

static void _calculate_junction_vmax(mpBuf_t* bf) 
{
    // ++++ RG If we change cruise_vmax, we'll need to recompute junction_vmax, if we do this:
    float velocity = min(bf->cruise_vmax, bf->nx->cruise_vmax);  // start with our maximum possible velocity

    // uint8_t jerk_axis = AXIS_X;
    // cmAxes jerk_axis = AXIS_X;

    for (uint8_t axis = 0; axis < AXES; axis++) {
        if (bf->axis_flags[axis] || bf->nx->axis_flags[axis]) {       // skip axes with no movement
            float delta = fabs(bf->unit[axis] - bf->nx->unit[axis]);  // formula (1)

            // Corner case: If an axis has zero delta, we might have a straight line.
            // Corner case: An axis doesn't change (and it's not a straight line).
            //   In either case, division-by-zero is bad, m'kay?
            if (delta > EPSILON) {
                // formula (4): (See Note 1, above)

                // velocity = min(velocity, (cm.a[axis].max_junction_accel / delta));
                if ((cm.a[axis].max_junction_accel / delta) < velocity) {
                    velocity = (cm.a[axis].max_junction_accel / delta);
                    // bf->jerk_axis = axis;
                }
            }
        }
    }
    bf->junction_vmax = velocity;
}
