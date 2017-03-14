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
#include "plan_arc.h"
#include "stepper.h"
#include "spindle.h"
#include "coolant.h"
#include "util.h"
//#include "xio.h"        // DIAGNOSTIC

static void _initiate_feedhold(void);
static void _initiate_cycle_restart(void);
static void _initiate_queue_flush(void);

// Feedhold actions
static stat_t _feedhold_with_actions(void);
static stat_t _feedhold_with_no_actions(void);
static stat_t _feedhold_with_sync(void);
static stat_t _feedhold_restart_with_actions(void);
static stat_t _feedhold_restart_with_no_actions(void);

// Feedhold exits (finalization)
static stat_t _run_restart_cycle(void);
static stat_t _run_queue_flush(void);
static stat_t _run_program_stop(void);
static stat_t _run_program_end(void);
static stat_t _run_alarm(void);
static stat_t _run_shutdown(void);
static stat_t _run_interlock(void);

/****************************************************************************************
 * OPERATIONS AND ACTIONS
 *
 *  Operations work by queueing a set of actions, then running them in sequence until 
 *  the operation is complete or an error occurs.
 *
 *  Actions are coded to return:
 *    STAT_OK       - successful completion of the action
 *    STAT_EAGAIN   - ran to continuation - the action needs to be called again to complete
 *    STAT_XXXXX    - any other status is an error that quits the operation
 *
 *  run_operation returns:
 *    STAT_NOOP     - no operation is set up, but it's OK to call the operation runner
 *    STAT_OK       - operation has completed successfully
 *    STAT_EAGAIN   - operation needs to be re-entered to complete (via operation callback)
 *    STAT_XXXXX    - any other status is an error that quits the operation 
 *
 *  Constraints:
 *    - Operations run to completion. They are not preemptable (at this point)
 *    - Actions cannot be added to an operation once it is being run
 */

/*** Object Definitions ***/

#define ACTION_MAX 6                        // maximum actions that can be queued for an operation
typedef stat_t (*action_exec_t)();          // callback to action execution function

typedef struct cmAction {                   // struct to manage execution of operations
    uint8_t number;                         // DIAGNOSTIC for easier debugging. Not used functionally.
    struct cmAction *nx;                    // static pointer to next buffer
    action_exec_t func;                     // callback to operation action function. NULL == disabled

    void reset() {                          // clears function pointer
        func = NULL;
    };
} cmAction_t;

typedef struct cmOperation {                // operation runner object

    cmAction action[ACTION_MAX];            // singly linked list of action structures
    cmAction *add;                          // pointer to next action to be added
    cmAction *run;                          // pointer to action being executed
    bool in_operation;                      // set true when an operation is running

    void reset(void) {
        for (uint8_t i=0; i < ACTION_MAX; i++) {
            action[i].reset();              // reset the action controller object
            action[i].number = i;           // DIAGNOSTIC only. Otherwise not used
            action[i].nx = &action[i+1];    // link to the next action
        }
        action[ACTION_MAX-1].nx = NULL;     // set last action (end of list)
        add = action;                       // initialize pointers to first action struct
        run = action;
        in_operation = false;
    };

    stat_t add_action(stat_t(*action_exec)()) {
        if (in_operation) { return (STAT_COMMAND_NOT_ACCEPTED); }       // can't add
        if (add == NULL)  { return (STAT_INPUT_EXCEEDS_MAX_LENGTH); }   // no more room
        add->func = action_exec;
        add = add->nx;
        return (STAT_OK);
    };

    stat_t run_operation(void) {
        if (run->func == NULL) { return (STAT_NOOP); }  // not an error. This is normal.
        in_operation = true;                // disable add_action during operations
 
        stat_t status;
        while ((status = run->func()) == STAT_OK) {
            run = run->nx;
            if (run->func == NULL) {        // operation has completed
                reset();                    // setup for next operation
                return (STAT_OK);
            }
        }
        if (status == STAT_EAGAIN) { return (STAT_EAGAIN); }
        reset();                            // reset operation if action threw an error
        return (status);                    // return error code
    };
      
} cmOperation_t;

cmOperation_t op;   // operations runner object

/****************************************************************************************
 * cm_operation_init()
 */

void cm_operation_init()
{
    op.reset();
}

/****************************************************************************************
 * cm_operation_sequencing_callback() - run operations and sequence requests
 *
 *  The operation callback can hold a set of requests and makes decisions on which to run
 *  based on priorities and other factors.
 *
 *  Expected behaviors: (no-hold means machine is not in hold, etc)
 *
 *  (no-cycle) !    No action. Feedhold is not run (nothing to hold!)
 *  (no-hold)  ~    No action. Cannot exit a feedhold that does not exist
 *  (no-hold)  %    No action. Queue flush is not honored except during a feedhold
 *  (in-cycle) !    Start a feedhold
 *  (in-hold)  ~    Wait for feedhold actions to complete, exit feedhold, resume motion 
 *  (in-hold)  %    Wait for feedhold actions to complete, exit feedhold, do not resume motion
 *  (in-cycle) !~   Start a feedhold, do enter and exit actions, exit feedhold, resume motion
 *  (in-cycle) !%   Start a feedhold, do enter and exit actions, exit feedhold, do not resume motion
 *  (in-cycle) !%~  Same as above
 *  (in-cycle) !~%  Same as above (this one's an anomaly, but the intent would be to Q flush)
 */

stat_t cm_operation_sequencing_callback()
{
    if (cm1.hold_state == FEEDHOLD_REQUESTED) {
        _initiate_feedhold();
    }
    if (cm1.flush_state == FLUSH_REQUESTED) {
        _initiate_queue_flush();
    }
    if (cm1.cycle_state == CYCLE_START_REQUESTED) {
        _initiate_cycle_restart();
    }
    return (op.run_operation());
}

/****************************************************************************************
 **** Functions *************************************************************************
 ****************************************************************************************/

/*
 * cm_has_hold() - return true if a hold condition exists (or a pending hold request)
 */

bool cm_has_hold()
{
    return (cm1.hold_state != FEEDHOLD_OFF);
}

stat_t cm_feedhold_command_blocker()
{
    if (cm1.hold_state != FEEDHOLD_OFF) {
        return (STAT_EAGAIN);
    }
    return (STAT_OK);
}

/****************************************************************************************
 * cm_request_cycle_start() - set request enum only
 * _initiate_cycle_start()  - run the cycle start
 */

void cm_request_cycle_start()
{
    if (cm1.hold_state != FEEDHOLD_OFF) {       // restart from a feedhold
        cm1.cycle_state = CYCLE_START_REQUESTED; 
    } else {                                    // execute cycle start directly
        if (mp_has_runnable_buffer(&mp1)) {
            cm_cycle_start();
            st_request_exec_move();
        }
        cm1.cycle_state = CYCLE_START_OFF;        
    }    
}

static void _initiate_cycle_restart()
{
    // Feedhold cycle starts run an operation to complete multiple actions
    if (cm1.hold_state == FEEDHOLD_HOLD) {
        cm1.cycle_state = CYCLE_START_OFF;
        switch (cm1.hold_type) {
            case FEEDHOLD_TYPE_ACTIONS:    { op.add_action(_feedhold_restart_with_actions); break; }
            case FEEDHOLD_TYPE_NO_ACTIONS: { op.add_action(_feedhold_restart_with_no_actions); break; }
            default: {}
        }
        switch (cm1.hold_final) {
            case FEEDHOLD_EXIT_CYCLE: { 
                op.add_action(_run_restart_cycle); 
                break; 
            }
            case FEEDHOLD_EXIT_FLUSH: {
                op.add_action(_run_queue_flush);
                op.add_action(_run_program_stop);
                break; 
            }
            case FEEDHOLD_EXIT_STOP: { 
                op.add_action(_run_program_stop); 
                break;
            }
            case FEEDHOLD_EXIT_END: { 
                op.add_action(_run_program_end); 
                break; 
            }
            case FEEDHOLD_EXIT_ALARM: { 
                op.add_action(_run_alarm); 
                break; 
            }
            case FEEDHOLD_EXIT_SHUTDOWN: { 
                op.add_action(_run_shutdown); 
                break; 
            }
            case FEEDHOLD_EXIT_INTERLOCK: { 
                op.add_action(_run_interlock); 
                break; 
            }
            default: {}
        }
    } 
}

/****************************************************************************************
 * cm_request_queue_flush() - set request enum only
 * _initiate_queue_flush()  - run a queue flush from a %
 * _restart_flush()         - run a queue flush from an action
 *
 *  _restart_flush() assumes that the operations sequencing callback has resolved 
 *  all state and timing issues and it's OK to call this now. Do not call this 
 *  function directly. Always call using the operations runner.
 */

void cm_request_queue_flush()
{
    // Can only initiate a queue flush if in a feedhold
    if (cm->hold_state != FEEDHOLD_OFF) {
        cm->flush_state = FLUSH_REQUESTED;
    } else {
        cm->flush_state = FLUSH_OFF;        
    }
}

static void _initiate_queue_flush()
{
    // Don't initiate the queue until in HOLD state (this also means that runtime is idle)
    if ((cm1.flush_state == FLUSH_REQUESTED) && (cm1.hold_state == FEEDHOLD_HOLD)) {
        if (cm1.hold_type == FEEDHOLD_TYPE_ACTIONS) {
            op.add_action(_feedhold_restart_with_actions);
        } else {
            op.add_action(_feedhold_restart_with_no_actions);
        }
        op.add_action(_run_queue_flush);
        op.add_action(_run_program_stop);
    }
}

static stat_t _run_queue_flush()            // typically runs from cm1 planner
{
    cm_abort_arc(cm);                       // kill arcs so they don't just create more alines
    planner_reset((mpPlanner_t *)cm->mp);   // reset primary planner. also resets the mr under the planner
    cm_reset_position_to_absolute_position(cm);
    cm1.flush_state = FLUSH_OFF;
    qr_request_queue_report(0);             // request a queue report, since we've changed the number of buffers available
    return (STAT_OK);
}

static stat_t _run_program_stop()
{
    cm_cycle_end();                         // end cycle and run program stop
//    cm_program_stop();
    return (STAT_OK);
}

static stat_t _run_program_end()
{
    cm_program_end();
    return (STAT_OK);
}

static stat_t _run_alarm()
{
    return (STAT_OK);
}

static stat_t _run_shutdown()
{
    return (STAT_OK);
}

static stat_t _run_interlock()
{
    return (STAT_OK);
}

/****************************************************************************************
 *  cm_request_feedhold()    - request a feedhold - d0 not run it yet
 *  _initiate_feedhold()     - start feedhold of correct type and finalization
 *  _feedhold_sync_to_planner() - planner callback to reach sync point
 *  _feedhold_with_sync()
 *  _feedhold_with_no_actions()
 *  _feedhold_with_actions() - perform hold entry actions
 *
 *  Input arguments
 *    - See cmFeedholdType  - how the feedhold will execute
 *    - See cmFeedholdFinal - the final state when the feedhold is exited
 */

void cm_request_feedhold(cmFeedholdType type, cmFeedholdFinal final)
{    
    // Can only initiate a feedhold if you are in a machining cycle not already in a feedhold
    if ((cm->hold_state == FEEDHOLD_OFF) && (cm->machine_state == MACHINE_CYCLE)) {
        cm->hold_type = type;
        cm->hold_final = final;
        cm->hold_state = FEEDHOLD_REQUESTED;
        _initiate_feedhold();                   // attempt to run it immediately
    } else {
        cm->hold_state = FEEDHOLD_OFF;          // reset 
    }
}

static void _initiate_feedhold()
{
    // This function is "safe" and will not initiate a feedhold unless it's OK to.

    if ((cm1.hold_state == FEEDHOLD_REQUESTED) && (cm1.motion_state == MOTION_RUN)) {
        switch (cm1.hold_type) {
            case FEEDHOLD_TYPE_ACTIONS:    { op.add_action(_feedhold_with_actions); break; }
            case FEEDHOLD_TYPE_NO_ACTIONS: { op.add_action(_feedhold_with_no_actions); break; }
            case FEEDHOLD_TYPE_SYNC:       { op.add_action(_feedhold_with_sync); break; }
            default: { }
        }
        switch (cm1.hold_final) {
            case FEEDHOLD_EXIT_STOP:      { op.add_action(_run_program_stop); break; }
            case FEEDHOLD_EXIT_END:       { op.add_action(_run_program_end); break; }
            case FEEDHOLD_EXIT_ALARM:     { op.add_action(_run_alarm); break; }
            case FEEDHOLD_EXIT_SHUTDOWN:  { op.add_action(_run_shutdown); break; }
            case FEEDHOLD_EXIT_INTERLOCK: { op.add_action(_run_interlock); break; }
            default: { }
        }
        cm1.hold_state = FEEDHOLD_SYNC;     // start feedhold state machine in aline exec
        return;
    } 
    
    // P2 feedholds only allow feedhold sync types
    if ((cm2.hold_state == FEEDHOLD_REQUESTED) && (cm2.motion_state == MOTION_RUN)) {
        op.add_action(_feedhold_with_sync);
        cm2.hold_state = FEEDHOLD_SYNC;
    }
}

static void _feedhold_sync_to_planner(float* vect, bool* flag)
{
    cm1.hold_state = FEEDHOLD_HOLD_ACTIONS_COMPLETE;    // penultimate state before transitioning to FEEDHOLD_HOLD
    sr_request_status_report(SR_REQUEST_IMMEDIATE);
}

static stat_t _feedhold_with_no_actions()
{
    return (STAT_OK);
}

static stat_t _feedhold_with_sync()
{
    return (STAT_OK);
}

static stat_t _feedhold_with_actions()   // Execute Case (5)
{
    // Check to run first-time code
    if (cm1.hold_state == FEEDHOLD_HOLD_ACTIONS_START) {
        cm->hold_state = FEEDHOLD_HOLD_ACTIONS_PENDING;  // next state

        // copy the primary canonical machine to the secondary,
        // fix the planner pointer, and reset the secondary planner
        memcpy(&cm2, &cm1, sizeof(cmMachine_t));
        cm2.mp = &mp2;
        planner_reset((mpPlanner_t *)cm2.mp);   // mp is a void pointer

        // set parameters in cm, gm and gmx so you can actually use it
        cm2.hold_state = FEEDHOLD_OFF;
        cm2.gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;
        cm2.gm.absolute_override = ABSOLUTE_OVERRIDE_OFF;
        cm2.flush_state = FLUSH_OFF;
        cm2.gm.feed_rate = 0;

        // clear the target and set the positions to the current hold position
        memset(&(cm2.gm.target), 0, sizeof(cm2.gm.target));
        memset(&(cm2.return_flags), 0, sizeof(cm2.return_flags));
        copy_vector(cm2.gm.target_comp, cm1.gm.target_comp); // preserve original Kahan compensation
        copy_vector(cm2.gmx.position, mr1.position);
        copy_vector(mp2.position, mr1.position);
        copy_vector(mr2.position, mr1.position);

        // reassign the globals to the secondary CM
        cm = &cm2;
        mp = (mpPlanner_t *)cm->mp;     // mp is a void pointer
        mr = mp->mr;

        // set a return position
        cm_set_g30_position();

        // execute feedhold actions
        if (fp_NOT_ZERO(cm->feedhold_z_lift)) {                 // optional Z lift
            cm_set_distance_mode(INCREMENTAL_DISTANCE_MODE);
            bool flags[] = { 0,0,1,0,0,0 };
            float target[] = { 0,0, _to_inches(cm->feedhold_z_lift), 0,0,0 };   // convert to inches if in inches mode
            cm_straight_traverse(target, flags, PROFILE_NORMAL);
            cm_set_distance_mode(cm1.gm.distance_mode);         // restore distance mode to p1 setting
        }
        spindle_control_sync(SPINDLE_PAUSE);                    // optional spindle pause
        coolant_control_sync(COOLANT_PAUSE, COOLANT_BOTH);      // optional coolant pause
        mp_queue_command(_feedhold_sync_to_planner, nullptr, nullptr);
        return (STAT_EAGAIN);
    }

    // wait for hold actions to complete
    if (cm1.hold_state == FEEDHOLD_HOLD_ACTIONS_PENDING) {
        return (STAT_EAGAIN);
    }
    
    // finalize feedhold entry after callback (this is needed so we can return STAT_OK)
    if (cm1.hold_state == FEEDHOLD_HOLD_ACTIONS_COMPLETE) {
        cm1.hold_state = FEEDHOLD_HOLD;
        return (STAT_OK);
    }

    return (STAT_EAGAIN);                   // keep the compiler happy. Never executed.
}

/****************************************************************************************
 *  _feedhold_restart_sync_to_planner() - planner callback to reach sync point
 *  _feedhold_restart_with_no_actions() - perform hold restart with no actions
 *  _feedhold_restart_with_actions()    - perform hold restart with actions
 */

static void _feedhold_restart_sync_to_planner(float* vect, bool* flag)
{
    cm1.hold_state = FEEDHOLD_EXIT_ACTIONS_COMPLETE;        // penultimate state before transitioning to FEEDHOLD_OFF
    sr_request_status_report(SR_REQUEST_IMMEDIATE);
}

static stat_t _feedhold_restart_with_no_actions()
{
    return (STAT_OK);
}

static stat_t _feedhold_restart_with_actions()   // Execute Cases (6) and (7)
{
    // Check to run first-time code
    if (cm1.hold_state == FEEDHOLD_HOLD) {
        // perform end-hold actions --- while still in secondary machine
        coolant_control_sync(COOLANT_RESUME, COOLANT_BOTH); // resume coolant if paused
        spindle_control_sync(SPINDLE_RESUME);               // resume spindle if paused

        // do return move though an intermediate point; queue a wait
        cm2.return_flags[AXIS_Z] = false;
        cm_goto_g30_position(cm2.gmx.g30_position, cm2.return_flags);
        mp_queue_command(_feedhold_restart_sync_to_planner, nullptr, nullptr);
        cm1.hold_state = FEEDHOLD_EXIT_ACTIONS_PENDING;
        return (STAT_EAGAIN);
    }
    
    // wait for exit actions to complete
    if (cm1.hold_state == FEEDHOLD_EXIT_ACTIONS_PENDING) {
        return (STAT_EAGAIN);
    }
    
    // finalize feedhold exit
    if (cm1.hold_state == FEEDHOLD_EXIT_ACTIONS_COMPLETE) {
        cm = &cm1;                          // return to primary planner (p1)
        mp = (mpPlanner_t *)cm->mp;         // cm->mp is a void pointer
        mr = mp->mr;
        return (STAT_OK);
    }
    
    return (STAT_EAGAIN);                   // keep the compiler happy. Never executed.
}

static stat_t _run_restart_cycle(void)
{
    cm1.hold_state = FEEDHOLD_OFF;          // must precede st_request_exec_move()
    if (mp_has_runnable_buffer(&mp1)) {
        cm_cycle_start();
        st_request_exec_move();
    } else {
        cm_cycle_end();
    }
    return (STAT_OK);
}

























/****************************************************************************************
 * Queue Flush operations
 *
 * This one's complicated. See here first:
 * https://github.com/synthetos/g2/wiki/Job-Exception-Handling
 * https://github.com/synthetos/g2/wiki/Alarm-Processing
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

/*
static stat_t _parking_move(float *param);
static stat_t _return_move(float *param);
static stat_t _spindle_control(float *param);
static stat_t _coolant_control(float *param);
static stat_t _heater_control(float *param);
static stat_t _output_control(float *param);
static stat_t _queue_flush(float *param);
static stat_t _input_function(float *param);
static stat_t _finalize_program(float *param);
static stat_t _trigger_alarm(float *param);
*/

/*
typedef enum {                  // cycle start and feedhold requests
    OPERATION_NULL = 0,         // no operation; reverts here when complete
    OPERATION_CYCLE_START,      // perform a cycle start with no other actions
    OPERATION_HOLD,             // feedhold in p1 or p2 with no entry actions
//    OPERATION_HOLD_WITH_ACTIONS,// feedhold into p2 with entry actions
//    OPERATION_P1_FAST_HOLD,     // perform a fast hold in p1
//    OPERATION_HOLD_TO_SYNC,     // hold in p1 or p2, skip remainder of move; exec SYNC command or flush
    OPERATION_EXIT_HOLD_RESUME, // exit p1 or p2 hold and resume motion in p1 planner
    OPERATION_EXIT_HOLD_FLUSH,  // exit p1 or p2 hold and flush p1 planner
    OPERATION_JOB_KILL,         // fast hold followed by queue flush and program end
    OPERATION_END_JOG,          // fast hold followed by program stop
    OPERATION_SOFT_LIMIT_HIT,   // actions to run when a soft limit is hit
    OPERATION_HARD_LIMIT_HIT,   // actions to run when a hard limit is hit
    OPERATION_SHUTDOWN,         // actions to run when a shutdown is received
    OPERATION_PANIC,            // actions to run when a panic is received
    OPERATION_INTERLOCK_START,  // enter an interlock state
    OPERATION_INTERLOCK_END,    // exit interlocked state
    OPERATION_DI_FUNC_STOP,     // run digital input STOP function
    OPERATION_DI_FUNC_FAST_STOP,// run digital input FAST_STOP function
    OPERATION_DI_FUNC_HALT,     // run digital input HALT function
    OPERATION_TOOL_CHANGE       // run a tool change sequence
} cmOperationType;
*/

/*
typedef enum {                  // Operation Actions
    // Initiation actions
    ACTION_NULL = 0,            // no pending action; reverts here when complete (read-only; cannot be set)
    ACTION_HOLD,                // p1/p2 feedhold at selected jerk ending in HOLD state
    //    ACTION_P1_HOLD,             // p1 feedhold at normal jerk ending in HOLD state in p1
    //    ACTION_P2_HOLD,             // p2 feedhold at normal jerk ending in HOLD state in p2 (not used)
    //    ACTION_P1_FAST_HOLD,        // p1 feedhold at high jerk ending in HOLD state in p1
    //    ACTION_P2_FAST_HOLD,        // p2 feedhold at high jerk ending in HOLD state in p2
    ACTION_HALT_MOTION,         // halt all motion immediately (regardless of p1 or p2)
    
    // Hold Entry Actions - run when motion has stopped
    ACTION_P2_ENTRY,            // Z lift, spindle and coolant actions (bundled)
    ACTION_PARKING_MOVE,        // perform a pre-defined toolhead parking move
    ACTION_PAUSE_SPINDLE,
    ACTION_PAUSE_COOLANT,
    ACTION_PAUSE_HEATERS,
    ACTION_PAUSE_OUTPUTS,       // programmed special purpose outputs
    ACTION_STOP_SPINDLE,
    ACTION_STOP_COOLANT,
    ACTION_STOP_HEATERS,
    ACTION_STOP_OUTPUTS,
    
    // In-Hold actions
    ACTION_TOOL_CHANGE,
    
    // Feedhold Exit Actions    // This set is kept in a separate vector, so the bit shifts start over.
    ACTION_P2_EXIT,             // coolant, spindle, return move actions (bundled)
    ACTION_RESUME_HEATERS,
    ACTION_RESUME_COOLANT,
    ACTION_RESUME_SPINDLE,
    ACTION_RESUME_OUTPUTS,      // programmed special purpose outputs
    ACTION_RETURN_MOVE,         // returns to hold pint an re-enters p1
    
    // Cycle start, restart, exit hold
    ACTION_SKIP_TO_SYNC,        // discard remaining length, execute next block if SYNC command, otherwise flush buffer
    ACTION_CYCLE_START,         // (~) exit feedhold, perform exit actions if in p2, resume p1 motion
    ACTION_QUEUE_FLUSH,         // (%) exit feedhold, perform exit actions if in p2, flush planner queue, enter p1 PROGRAM_STOP
    
    // Finalization actions
    ACTION_DI_FUNCTION,         // function as assigned by digital input configuration (TBD)
    ACTION_PROGRAM_STOP,        // invoke PROGRAM_STOP
    ACTION_PROGRAM_END,         // invoke PROGRAM_END
    ACTION_TRIGGER_ALARM,       // trigger ALARM
    ACTION_TRIGGER_SHUTDOWN,    // trigger SHUTDOWN
    ACTION_TRIGGER_PANIC,       // trigger PANIC state
    ACTION_PERFORM_RESET,       // defined, but not implemented
} cmOpAction;
*/

/****************************************************************************************
 * cm_request_feedhold()    - reqeust a feedhold
 * cm_request_exit_hold()   - reqeust feedhold exit with resume motion
 * cm_request_queue_flush() - request feedhold exit with queue flush
 * cm_start_hold()          - start a feedhhold external to feedhold request equencing
 * cm_feedhold_command_blocker() - prevents new Gcode commands from queueing to p2 planner
 *
 *  p1 is the primary planner, p2 is the secondary planner, which is active if the 
 *  primary planner is in hold. IOW p2 can only be in a hold if p1 is already in one.
 *  Request_feedhold, request_end_hold, and request_queue_flush are contextual:
 *
 *  It's OK to call start_hold directly in order to get a hold quickly (see gpio.cpp)
 * 
 *  request_feedhold:
 *    - If p1 is not in HOLD & is in motion, request_feedhold requests a p1 hold
 *    - If p1 is in HOLD & p2 is in motion, request_feedhold requests a p2 hold
 *    - If both p1 and p2 are in HOLD, request_feedhold is ignored
 *
 *  request_end_hold:
 *    - If p1 is not in HOLD, request_end_hold is ignored
 *    - If p1 is in HOLD request_end_hold will end p1 hold & resume motion.
 *      Pre-defined exit actions (coolant, spindle, Z move) are completed first
 *      Any executing or pending "in-hold" moves are stopped prior to the exit actions
 *
 *  request_queue_flush:
 *    - If p1 is not in HOLD, request_queue_flush is ignored
 *    - If p1 is in HOLD request_queue_flush will end p1 hold & queue flush (stop motion).
 *      Pre-defined exit actions (coolant, spindle, Z move) are completed first
 *      Any executing or pending "in-hold" moves are stopped prior to the exit actions
 */

/****************************************************************************************
 * cm_request_operation()
 *
 *  Invoke an operation by calling cm_request_operation(); may require one or more parameters
 *    - The operation runner must be idle: an operation cannot interrupt a currently running operation
 *    - Future may need Cancel Operation semantics, but this could get overcomplicated quickly
 *
 *  When a new operation is requested the operations runner object is cleared and one or
 *  more actions are queued by calling add_action() on the object.
 *
 *  To start the operation immediately call run_action() at the end of the request.
 *  Otherwise the operation will begin the next time cm_operation_callback().
 *
 *  It is assumed that all actions are added at once, and that this cannot be interrupted
 *  by a run request. So no attempt is made at mutual exclusion. Just behave.
 *
 *  Operations are defined as:
 *
 *    - OPERATION_CYCLE_START - start cycle from STOP or restart from hold
 *
 *    - OPERATION_HOLD - initiate a hold of a given form. Parameters:
 *        - param[0] - p1/p2    0=hold-into-p1, 1=hold-into-p2 (with entry actions) 
 *        - param[1] - jerk     0=normal-jerk, 1=high-jerk (fast hold)
 *        - param[2] - endstate 
 */

/****************************************************************************************
 **** Feedholds *************************************************************************
 ****************************************************************************************/
/*
 *  Feedholds, queue flushes and end_holds are all related and are performed in this
 *  file and in plan_exec.cpp. Feedholds are implemented as a state machine 
 *  (cmFeedholdState) that runs in these files. 
 *
 *  There are 2 planners: p1 (primary planner) and p2 (secondary planner). A feedhold
 *  received while in p1 stops motion in p1 and transitions to p2, where entry actions
 *  like Z lift, spindle and coolant pause occur. While in p2 (almost) all machine 
 *  operations are available. 
 *
 *  A feedhold received while in p2 (a feedhold within a feedhold - very Inception)
 *  stops motion in p2 and flushes the p2 planner. Control remains in p2.
 *
 *  A feedhold exit request (~) received while in either p1 or p2 will execute the 
 *  feedhold exit actions:
 *    - Resume coolant (if paused)
 *    - Resume spindle (if paused) with spinup delay
 *    - Move back to starting location in XY, then plunge in Z
 *  Motion will resume in p1 after the exit actions complete
 *
 *  A feedhold flush request (%) received while in either p1 or p2 will execute the
 *  exit actions, flush the p1 and p2 queues, then stop motion at the hold point.
 */
/*
 * Feedhold Processing - Performs the following cases (listed in rough sequence order):
 *
 *  (0) - Feedhold request arrives or cm_start_hold() is called
 *
 * Control transfers to plan_exec.cpp feedhold functions:
 *
 *  (1) - Feedhold arrives while we are in the middle executing of a block
 *   (1a) - The block is currently accelerating - wait for the end of acceleration
 *   (1b) - The block is in a head, but has not started execution yet - start deceleration
 *    (1b1) - The deceleration fits into the current block
 *    (1b2) - The deceleration does not fit and needs to continue in the next block
 *   (1c) - The block is in a body - start deceleration
 *    (1c1) - The deceleration fits into the current block
 *    (1c2) - The deceleration does not fit and needs to continue in the next block
 *   (1d) - The block is currently in the tail - wait until the end of the block
 *   (1e) - We have a new block and a new feedhold request that arrived at EXACTLY the same time
 *          (unlikely, but handled as 1b).
 *
 *  (2) - The block has decelerated to some velocity > zero, so needs continuation into next block
 *  (3) - The end of deceleration is detected inline in mp_exec_aline()
 *  (4) - Finished all runtime work, now wait for the motors to stop on HOLD point. When they do:
 *   (4a) - It's a homing or probing feedhold - ditch the remaining buffer & go directly to OFF
 *   (4b) - It's a p2 feedhold - ditch the remaining buffer & signal we want a p2 queue flush
 *   (4c) - It's a normal feedhold - signal we want the p2 entry actions to execute
 *
 * Control transfers back to cycle_feedhold.cpp feedhold functions:
 *
 *  (5) - Run the P2 entry actions and transition to HOLD state when complete
 *  (6) - Remove the hold state / there is queued motion - see cycle_feedhold.cpp
 *  (7) - Remove the hold state / there is no queued motion - see cycle_feedhold.cpp
 */

/****************************************************************************************
 * cm_feedhold_sequencing_callback() - sequence feedhold, queue_flush, and end_hold requests
 *
 * Expected behaviors: (no-hold means machine is not in hold, etc)
 *
 *  (no-cycle) !    No action. Feedhold is not run (nothing to hold!)
 *  (no-hold)  ~    No action. Cannot exit a feedhold that does not exist
 *  (no-hold)  %    No action. Queue flush is not honored except during a feedhold
 *  (in-cycle) !    Start a feedhold
 *  (in-hold)  ~    Wait for feedhold actions to complete, exit feedhold, resume motion 
 *  (in-hold)  %    Wait for feedhold actions to complete, exit feedhold, do not resume motion
 *  (in-cycle) !~   Start a feedhold, do enter and exit actions, exit feedhold, resume motion
 *  (in-cycle) !%   Start a feedhold, do enter and exit actions, exit feedhold, do not resume motion
 *  (in-cycle) !%~  Same as above
 *  (in-cycle) !~%  Same as above (this one's an anomaly, but the intent would be to Q flush)
 */

/****************************************************************************************
 * _feedhold_abort() - used to exit a feedhold without completing exit actions
 *
 *  Valid entry states (all must be handled):
 *    Case (1)  Not in a feedhold (FEEDHOLD_OFF). Ignore the request
 *
 *    Case (2)  In a feedhold but have not yet hit the hold point.
 *              Leave the abort request pending to be picked up by p2 entry actions,
 *              which are not allowed to proceed. 
 *
 *    Case (3) In a feedhold and currently executing P2 entry actions
 *    Case (4) In a feedhold and currently idle in P2
 *    Case (5) In a feedhold and currently moving in P2
 *    Case (6) In a feedhold and currently executing P2 exit actions
 */
/*
static void _feedhold_abort()
{
    // Exit if not in a feedhold
    if (cm1.hold_state == FEEDHOLD_OFF) {
        cm1.hold_abort_requested = false;
        return;
    }

    // No action if waiting for HOLD point - let P2_START run the abort
    if ((cm1.hold_state > FEEDHOLD_OFF) && (cm1.hold_state < FEEDHOLD_P2_START)) {
        return;
    }            
    
    // If in p2 perform the p2 exit first
    if (cm == &cm2) {
        _feedhold_p2_exit();
    }

    // perform a complete exit from p1
    cm = &cm1;                              // return to p1 if not already here
    mp = (mpPlanner_t *)cm->mp;             // cm->mp is a void pointer
    mr = mp->mr;

    // execute this block if a queue flush was performed
    // adjust p1 planner positions to runtime positions
    if (cm1.flush_state == FLUSH_WAS_RUN) {
        cm_reset_position_to_absolute_position(cm);
        cm1.flush_state = FLUSH_OFF;
    }

    // end cycle
//    cm_set_motion_state(MOTION_STOP);
//    cm_cycle_end();
    cm1.hold_state = FEEDHOLD_OFF;
    cm1.hold_abort_requested = false;
}    
*/

