/*
 * cycle_feedhold.cpp - canonical machine feedhold processing
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2017 Alden S Hart, Jr.
 * Copyright (c) 2014 - 2017 Robert Giseburt
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

#include "g2core.h"     // #1
#include "config.h"     // #2
#include "gcode.h"      // #3
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"
#include "spindle.h"
#include "coolant.h"
#include "util.h"

/***********************************************************************************
 **** Feedholds ********************************************************************
 ***********************************************************************************/
/*
 *  Feedholds, queue flushes and end_holds are all related and are in this file.
 *  Feedholds are implemented as a state machine (cmFeedholdState) that runs in the 
 *  planner (plan_exec.cpp, see Feedhold Processing around line 500) and in this file.
 *
 *  Feedholds also use the dual planner (secondary context, or hold context) where a
 *  new canonical machine and planner are spun up and entered when a feedhold is initiated. 
 *  This allows (almost) all of the machine operations to be accessible during a feedhold.
 *
 *  Feedholds are initiated and ended by a series of request flags (requests).
 *  The request functions set flags or change state to "REQUESTED". 
 *  The sequencing callback interprets the flags as so:
 *    - A feedhold request received during motion should be honored
 *    - A feedhold request received during a feedhold should be ignored
 *    - A feedhold request received during a motion stop should be ignored
 *
 *    - A queue flush request should only be honored while in a feedhold
 *    - Said queue flush request received during a feedhold should be deferred until
 *      the feedhold enters a HOLD state (i.e. until deceleration is complete and motors stop).
 *    - A queue flush request received during a motion stop should be honored
 *
 *    - An end_hold (cycle start) request should only be honored while in a feedhold
 *    - Said end_hold request received during a feedhold should be deferred until the
 *      feedhold enters a HOLD state (i.e. until deceleration is complete).
 *      If a queue flush request is also present the queue flush should be done first
 */
/*  Below the request level, feedholds work like this:
 *    - The hold is initiated by calling cm_start_hold(). cm->hold_state is set to
 *      FEEDHOLD_SYNC, motion_state is set to MOTION_HOLD, and the spindle is turned off
 *      (if it it on). The remainder of feedhold
 *      processing occurs in plan_exec.c in the mp_exec_aline() function.
 *
 *      - MOTION_HOLD and FEEDHOLD_SYNC tells mp_exec_aline() to begin feedhold processing
 *      after the current move segment is finished (< 5 ms later). (Cases handled by
 *      feedhold processing are listed in plan_exec.c).
 *
 *    - FEEDHOLD_SYNC causes the current move in mr to be replanned into a deceleration.
 *      If the distance remaining in the executing move is sufficient for a full deceleration
 *      then motion will stop in the current block. Otherwise the deceleration phase
 *      will extend across as many blocks necessary until one will stop.
 *
 *    - Once deceleration is complete hold state transitions to FEEDHOLD_FINALIZING and 
 *      the distance remaining in the bf last block is replanned up from zero velocity.
 *      The move in the bf block is NOT released (unlike normal operation), as it will
 *      be used again to restart from hold.
 *
 *    - When cm_end_hold() is called it releases the hold, restarts the move and restarts
 *      the spindle if the spindle is active.
 */

/* With the addition of the secondary CM, feedhold state management gets tricky.
   What you see below is a temporary solution until we decide the general solution.
   
   The general solution causes a feedhold from the primary context to switch 
   into the secondary context to perform feedhold actions. When in the secondary
   context an additional feedhold will perform the usual STOP operation, but 
   will remain in the secondary context, and therefore not perform any feedhold
   actions (lifts, spindle, etc.). This is needed to support homing and probing 
   operations from within the secondary context.
   
   Oddities of the general solution:
      - Should we allow a feedhold to be performed if the tool is not moving? 
        Right now we don't, but with the secondary context this might be useful
   
    What you see here is a Q&D to only allow feedholds from the primary context. 
    It has the following limitations:
      - Feedhold requests are only honored form the primary context
      - Queue flush requests are only honored form the primary context
      - Machine alarm state is not (yet) taken into account in feedhold sequencing and restart
 */

/***********************************************************************************
 * cm_request_feedhold()
 * cm_request_end_hold()
 * cm_request_queue_flush()
 * cm_feedhold_sequencing_callback() - sequence feedhold, queue_flush, and end_hold requests
 */

void cm_request_feedhold(void) 
{
    // do not generate a feedhold request from the secondary context
    if (cm_select != CM_PRIMARY) {
        return;
    }
    // only generate request if not already in a feedhold and the machine is in motion    
    if ((cm1.hold_state == FEEDHOLD_OFF) && (cm1.motion_state != MOTION_STOP)) {
        cm1.hold_state = FEEDHOLD_REQUESTED;
    }
}

void cm_request_end_hold(void)  // This is usually requested form the secondary context
{
    if (cm1.hold_state != FEEDHOLD_OFF) {
        cm1.end_hold_requested = true;
    }
}

void cm_request_queue_flush()
{
    // do not generate a queue flush request from the secondary context
    if (cm_select != CM_PRIMARY) {
        return;
    }    
    if ((cm1.hold_state != FEEDHOLD_OFF) &&          // don't honor request unless you are in a feedhold
        (cm1.queue_flush_state == FLUSH_OFF)) {      // ...and only once
        cm1.queue_flush_state = FLUSH_REQUESTED;     // request planner flush once motion has stopped

        // NOTE: we used to flush the input buffers, but this is handled in xio *prior* to queue flush now
    }
}

stat_t cm_feedhold_sequencing_callback()
{
    if (cm1.hold_state == FEEDHOLD_REQUESTED) {
        cm_start_hold();                            // feed won't run unless the machine is moving
    }
    if (cm1.hold_state == FEEDHOLD_FINALIZING) {
        cm1.hold_state = FEEDHOLD_HOLD;
        cm_switch_to_hold_context();                // perform Z lift, spindle & coolant operations
    }
    if (cm1.queue_flush_state == FLUSH_REQUESTED) {
        cm_queue_flush();                           // queue flush won't run until runtime is idle
    }
    if (cm1.end_hold_requested) {
        if (cm1.queue_flush_state == FLUSH_OFF) {   // either no flush or wait until it's done flushing
            cm_end_hold();
        }
    }
    return (STAT_OK);
}

/***********************************************************************************
 * cm_has_hold()   - return true if a hold condition exists (or a pending hold request)
 * cm_start_hold() - start a feedhhold by signalling the exec
 * cm_end_hold()   - end a feedhold by returning the system to normal operation
 */
bool cm_has_hold()
{
    return (cm1.hold_state != FEEDHOLD_OFF);
}

void cm_start_hold()
{
    if (mp_has_runnable_buffer(mp)) {         //+++++           // meaning there's something running
        cm_set_motion_state(MOTION_HOLD);
        cm->hold_state = FEEDHOLD_SYNC;                      // invokes hold from aline execution
    }
}

void cm_end_hold()
{
    if (cm1.hold_state == FEEDHOLD_HOLD) {
        cm1.end_hold_requested = false;
        cm_return_from_hold_context();
    }
}

/***********************************************************************************
 *  cm_switch_to_hold_context()   - switch to secondary machine context
 *
 *  Moving between contexts is only safe when the machine is completely stopped 
 *  either during a feedhold or when idle.
 */

stat_t cm_switch_to_hold_context()
{
    // Must be in the primary CM and fully stopped in a hold
    if ((cm != &cm1) || (cm->hold_state != FEEDHOLD_HOLD)) {
        return (STAT_COMMAND_NOT_ACCEPTED);
    }
    
    // copy the primary canonical machine to the secondary, 
    // fix the planner pointer, and reset the secondary planner
    memcpy(&cm2, &cm1, sizeof(cmMachine_t));
    cm2.mp = &mp2;
    planner_reset((mpPlanner_t *)cm2.mp);   // mp is a void pointer

    // set parameters in cm, gm and gmx so you can actually use it
    cmMachine_t *_cm = &cm2;
    _cm->hold_state = FEEDHOLD_OFF;
    _cm->gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;
    _cm->gm.absolute_override = ABSOLUTE_OVERRIDE_OFF;
    _cm->gm.feed_rate = 0;

    // clear the target and set the positions to the current hold position
    memset(&(_cm->gm.target), 0, sizeof(_cm->gm.target));
    copy_vector(_cm->gm.target_comp, cm->gm.target_comp); // preserve original Kahan compensation
    copy_vector(_cm->gmx.position, mr->position);
    copy_vector(mp2.position, mr->position);
    copy_vector(mr2.position, mr->position);

    // reassign the globals to the secondary CM
    cm = &cm2;
    mp = (mpPlanner_t *)cm->mp;     // mp is a void pointer
    mr = mp->mr;
    cm_select = CM_SECONDARY;

    // set motion state and ACTIVE_MODEL. This must be performed after cm is set to cm2
    cm_set_g30_position();
    cm_set_motion_state(MOTION_STOP);

    // optional Z lift
    if (fp_NOT_ZERO(cm->feedhold_z_lift)) {
        float stored_distance_mode = cm_get_distance_mode(MODEL);
        cm_set_distance_mode(INCREMENTAL_DISTANCE_MODE);
        bool flags[] = { 0,0,1,0,0,0 };
        float target[] = { 0,0, cm->feedhold_z_lift, 0,0,0 };
        cm_straight_traverse(target, flags);
        cm_set_distance_mode(stored_distance_mode);
    }
            
    // optional spindle stop
    if (spindle.pause_on_hold) {
        
    }
    return (STAT_OK);
}

/***********************************************************************************
 *  cm_return_from_hold_context()  - initiate return from secondary context
 *  cm_return_from_hold_callback() - main loop callback to finsh return once moves are done 
 *  _planner_done_callback()       - callback to sync to end of planner operations 
 *
 *  Moving between contexts is only safe when the machine is completely stopped 
 *  either during a feedhold or when idle.
 */

// Callback to run at when the G30 return move is finished
static void _planner_done_callback(float* vect, bool* flag)
{
    cm2.waiting_for_planner_done = false;
}

stat_t cm_return_from_hold_context()     // LATER: if value == true return with offset corrections
{
    // Must be in the secondary CM and fully stopped
    if ((cm != &cm2) || (cm->motion_state != MOTION_STOP)) {
        return (STAT_COMMAND_NOT_ACCEPTED);
    }

    // *** While still in secondary machine:
/*
    if (cm->machine_state == MACHINE_ALARM) {
        cm_spindle_off_immediate();
        cm_coolant_off_immediate();
*/
    // restart spindle (with optional dwell)
    
    // restart coolant
    
    // perform the G30 move and queue a wait
    float target[] = { 0,0,0,0,0,0 };       // LATER: Make this move return through XY, then Z
    bool flags[]   = { 0,0,0,0,0,0 };
    cm_goto_g30_position(target, flags);    // initiate a return move
    cm->waiting_for_planner_done = true;    // indicates running the final G30 move in the secondary
    mp_queue_command(_planner_done_callback, nullptr, nullptr);
    cm_select = CM_SECONDARY_RETURN;
    return (STAT_OK);
    
    // return_to_primary completes in cm_return_callback() after the wait 
}

stat_t cm_return_from_hold_callback()
{
    if (cm_select != CM_SECONDARY_RETURN) { // exit if not in secondary planner
        return (STAT_NOOP);
    }
    if (cm->waiting_for_planner_done) {     // sync to planner move ends (via _return_move_callback)
        return (STAT_EAGAIN);
    }
    
    // return to primary machine
    cm = &cm1;
    mp = (mpPlanner_t *)cm->mp;             // cm->mp is a void pointer
    mr = mp->mr;
    cm_select = CM_PRIMARY;

    cm->hold_state = FEEDHOLD_OFF;
    if (mp_has_runnable_buffer(mp)) {       //+++++ Should MP be passed or global?
        cm_set_motion_state(MOTION_RUN);
        cm_cycle_start();
        st_request_exec_move();
        sr_request_status_report(SR_REQUEST_IMMEDIATE);
    } else {
        cm_set_motion_state(MOTION_STOP);
        cm_cycle_end();
    }
    return (STAT_OK);
}

/***********************************************************************************
 * Queue Flush operations
 *
 * This one's complicated. See here first:
 * https://github.com/synthetos/g2/wiki/Alarm-Processing
 * https://github.com/synthetos/g2/wiki/Job-Exception-Handling
 *
 * We want to use queue flush for a few different use cases, as per the above wiki pages.
 * The % behavior implements Exception Handling cases 1 and 2 - Stop a Single Move and
 * Stop Multiple Moves. This is complicated further by the processing in single USB and
 * dual USB being different. Also, the state handling is located in xio.cpp / readline(),
 * controller.cpp _dispatch_kernel() and cm_request_queue_flush(), below.
 * So it's documented here.
 *
 * Single or Dual USB Channels:
 *  - If a % is received outside of a feed hold or ALARM state, ignore it.
 *      Change the % to a ; comment symbol (xio)
 *
 * Single USB Channel Operation:
 *  - Enter a feedhold (!)
 *  - Receive a queue flush (%) Both dispatch it and store a marker (ACK) in the input
 *      buffer in place of the the % (xio)
 *  - Execute the feedhold to a hold condition (plan_exec)
 *  - Execute the dispatched % to flush queues (canonical_machine)
 *  - Silently reject any commands up to the % in the input queue (controller)
 *  - When ETX is encountered transition to STOP state (controller/canonical_machine)
 *
 * Dual USB Channel Operation:
 *  - Same as above except that we expect the % to arrive on the control channel
 *  - The system will read and dump all commands in the data channel until either a
 *    clear is encountered ({clear:n} or $clear), or an ETX is encountered on either
 *    channel, but it really should be on the data channel to ensure all queued commands
 *    are dumped. It is the host's responsibility to both write the clear (or ETX), and
 *    to ensure that it either arrives on the data channel or that the data channel is
 *    empty before writing it to the control channel.
 */

/***********************************************************************************
 * cm_queue_flush() - Flush planner queue and correct model positions
 */

void cm_queue_flush()
{
    if (mp_runtime_is_idle()) {                     // can't flush planner during movement
        mp_flush_planner(mp);       // +++++ Active planner. Potential cleanup

        for (uint8_t axis = AXIS_X; axis < AXES; axis++) { // set all positions
            cm_set_position(axis, mp_get_runtime_absolute_position(axis));
        }
        if(cm->hold_state == FEEDHOLD_HOLD) {        // end feedhold if we're in one
            cm_end_hold();
        }
        cm->queue_flush_state = FLUSH_OFF;
        qr_request_queue_report(0);                 // request a queue report, since we've changed the number of buffers available
    }
}

