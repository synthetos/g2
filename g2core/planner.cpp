/*
 * planner.cpp - Cartesian trajectory planning and motion execution
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2012 - 2016 Rob Giseburt
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
 *  The planner works below the canonical machine and above the motor mapping and stepper
 *  execution layers. A rudimentary multitasking capability is implemented for long-running
 *  commands such as lines, arcs, and dwells. These functions are coded as non-blocking
 *  continuations - which are simple state machines that are re-entered multiple times
 *  until a particular operation is complete. These functions have 2 parts - the initial call,
 *  which sets up the local context (closure), and callbacks (continuations) that are called
 *  from the main loop (in controller.c). These tasks only support a single instantiation
 *  and are therefore also not re-entrant - as they rely on singletons for closure.
 *
 *  One important concept is isolation of state at the three layers of the data model -
 *  the Gcode model (gm), motion planner model (bf queue & mm), and motion runtime model (mr).
 *  These are designated as "model", "planner" and "runtime" in function names.
 *
 *  The Gcode model is owned by the canonical machine and should only be accessed by cm_xxxx()
 *  functions. Data from the Gcode model is transferred to the motion planner by the mp_xxxx()
 *  functions called by the canonical machine.
 *
 *  The planner should only use data in the planner model. When a move (buffer) is ready for
 *  execution the relevant data from the planner is transferred to the runtime model,
 *  which should also be isolated.
 *
 *  Models at different levels should never use data from other levels as the data may have
 *  changed or be out-of-sync and lead to unpredictable results.
 */
#include "g2core.h"
#include "config.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "kinematics.h"
#include "stepper.h"
#include "encoder.h"
#include "report.h"
#include "util.h"
#include "json_parser.h"
#include "xio.h"    //+++++ DIAGNOSTIC - only needed if xio_writeline() direct prints are used

// Allocate planner structures
mpBufferPool_t mb;                  // buffer pool management
mpMotionPlannerSingleton_t mp;      // context for block planning
mpMotionRuntimeSingleton_t mr;      // context for block runtime

#define JSON_COMMAND_BUFFER_SIZE 3

struct json_command_buffer_t {
    char buf[RX_BUFFER_SIZE];
    json_command_buffer_t *pv;
    json_command_buffer_t *nx;
};

struct _json_commands_t {
    json_command_buffer_t _json_bf[JSON_COMMAND_BUFFER_SIZE]; // storage of all buffers
    json_command_buffer_t *_json_r;   // pointer to the next "run" buffer
    json_command_buffer_t *_json_w;   // pointer tot he next "write" buffer

    int8_t available;

    // Constructor (initializer)
    // Note, reset routines handle zeroing out all of the above
    _json_commands_t() {
        json_command_buffer_t *js_pv = &_json_bf[JSON_COMMAND_BUFFER_SIZE - 1];
        for (uint8_t i=0; i < JSON_COMMAND_BUFFER_SIZE; i++) {
            _json_bf[i].nx = &_json_bf[((i+1 == JSON_COMMAND_BUFFER_SIZE) ? 0 : i+1)];
            _json_bf[i].pv = js_pv;
            js_pv = &_json_bf[i];
        }
        _json_r = &_json_bf[0];
        _json_w = _json_r;
        available = JSON_COMMAND_BUFFER_SIZE;
    };

    // Write a json command to the buffer, using up one slot
    void write_buffer(char * new_json) {
        strcpy(_json_w->buf, new_json);
        available--;
        _json_w = _json_w->nx;
    };

    // Read a buffer out, but do NOT free it (so it can be used directly)
    char *read_buffer() {
        return _json_r->buf;
    };

    // Free the last read buffer.
    void free_buffer() {
        _json_r = _json_r->nx;
        available++;
    }
};

_json_commands_t jc;

// Local Scope Data and Functions
#define spindle_speed block_time    // local alias for spindle_speed to the time variable
#define value_vector gm.target      // alias for vector of values

//static void _planner_time_accounting();
static void _audit_buffers();

// Execution routines (NB: These are called from the LO interrupt)
static void _exec_json_command(float *value, bool *flag);
static stat_t _exec_dwell(mpBuf_t *bf);
static stat_t _exec_command(mpBuf_t *bf);
static stat_t _exec_json_wait(mpBuf_t *bf);

/*
 * planner_init()
 * planner_reset()
 */
void planner_init()
{
// If you know all memory has been zeroed by a hard reset you don't need these next 2 lines
    memset(&mp, 0, sizeof(mp));     // clear all values, pointers and status
    memset(&mr, 0, sizeof(mr));     // clear all values, pointers and status
    planner_init_assertions();
    mp_init_buffers();
    mp.mfo_factor = 1.00;
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
    // Note: mb magic numbers set up by mp_init_buffers()
    mp.magic_start = MAGICNUM;
    mp.magic_end = MAGICNUM;
    mr.magic_start = MAGICNUM;
    mr.magic_end = MAGICNUM;
}

stat_t planner_test_assertions()
{
    if ((BAD_MAGIC(mb.magic_start)) || (BAD_MAGIC(mb.magic_end)) ||
        (BAD_MAGIC(mp.magic_start)) || (BAD_MAGIC(mp.magic_end)) ||
        (BAD_MAGIC(mr.magic_start)) || (BAD_MAGIC(mr.magic_end))) {
        return(cm_panic(STAT_PLANNER_ASSERTION_FAILURE, "planner_test_assertions()"));
    }
//    for (uint8_t i=0; i < PLANNER_BUFFER_POOL_SIZE; i++) {
//        if (mb.bf[i].nx == nullptr) {
//            _debug_trap("buffer has nullptr for nx");
//        }
//        if (mb.bf[i].pv == nullptr) {
//            _debug_trap("buffer has nullptr for pv");
//        }
//    }
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
 *    Does not affect the move currently running in mr.
 *    Does not affect mm or gm model positions
 *    This function is designed to be called during a hold to reset the planner
 *    This function should not generally be called; call cm_queue_flush() instead
 */
void mp_flush_planner()
{
    cm_abort_arc();
    mp_init_buffers();
    mr.block_state = BLOCK_INACTIVE;   // invalidate mr buffer to prevent subsequent motion
}

/*
 * mp_set_planner_position() - set planner position for a single axis
 * mp_set_runtime_position() - set runtime position for a single axis
 * mp_set_steps_to_runtime_position() - set encoder counts to the runtime position
 *
 *  Since steps are in motor space you have to run the position vector through inverse
 *  kinematics to get the right numbers. This means that in a non-Cartesian robot changing
 *  any position can result in changes to multiple step values. So this operation is provided
 *  as a single function and always uses the new position vector as an input.
 *
 *  Keeping track of position is complicated by the fact that moves exist in several reference
 *  frames. The scheme to keep this straight is:
 *
 *     - mm.position    - start and end position for planning
 *     - mr.position    - current position of runtime segment
 *     - mr.target    - target position of runtime segment
 *
 *  The runtime keeps a lot more data, such as waypoints, step vectors, etc.
 *  See struct mpMoveRuntimeSingleton for details.
 *
 *  Note that position is set immediately when called and may not be not an accurate representation
 *  of the tool position. The motors are still processing the action and the real tool position is
 *  still close to the starting point.
 */

void mp_set_planner_position(uint8_t axis, const float position) { mp.position[axis] = position; }
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
    bf->block_type = BLOCK_TYPE_COMMAND;
    bf->bf_func = _exec_command;      // callback to planner queue exec function
    bf->cm_func = cm_exec;            // callback to canonical machine exec function

    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        bf->value_vector[axis] = value[axis];
        bf->axis_flags[axis] = flag[axis];
    }
    mp_commit_write_buffer(BLOCK_TYPE_COMMAND);     // must be final operation before exit
}

static stat_t _exec_command(mpBuf_t *bf)
{
    st_prep_command(bf);
    return (STAT_OK);
}

stat_t mp_runtime_command(mpBuf_t *bf)
{
    bf->cm_func(bf->value_vector, bf->axis_flags);  // 2 vectors used by callbacks
    if (mp_free_run_buffer()) {
        cm_cycle_end();                             // free buffer & perform cycle_end if planner is empty
    }
    return (STAT_OK);
}


/*************************************************************************
 * mp_json_command()    - queue a json command
 * _exec_json_command() - execute json string (from exec system)
 */

stat_t mp_json_command(char *json_string)
{
    // Never supposed to fail, since we stopped parsing when we were full
    jc.write_buffer(json_string);

    // We don't actually use these...
    float value[] = { 0,0,0,0,0,0 };
    bool flags[]  = { 0,0,0,0,0,0 };

    mp_queue_command(_exec_json_command, value, flags);
    return (STAT_OK);
}

static void _exec_json_command(float *value, bool *flag)
{
    char *json_string = jc.read_buffer();
    json_parse_for_exec(json_string, true); // process it
    jc.free_buffer();
}

/*************************************************************************
 * mp_json_command_immediate()    - execute a json command with response suppressed
 */

stat_t mp_json_command_immediate(char *json_string)
{
    return json_parser(json_string);
}

/*************************************************************************
 * mp_json_wait()    - queue a json wait command
 * _exec_json_wait() - execute json wait string
 */

stat_t mp_json_wait(char *json_string)
{
    // Never supposed to fail, since we stopped parsing when we were full
    jc.write_buffer(json_string);

    mpBuf_t *bf;

    // Never supposed to fail as buffer availability was checked upstream in the controller
    if ((bf = mp_get_write_buffer()) == NULL) {
        cm_panic(STAT_FAILED_GET_PLANNER_BUFFER, "mp_json_wait()");
        return STAT_ERROR;
    }
    bf->block_type = BLOCK_TYPE_COMMAND;
    bf->bf_func = _exec_json_wait;      // callback to planner queue exec function
    mp_commit_write_buffer(BLOCK_TYPE_COMMAND);            // must be final operation before exit
    return (STAT_OK);
}

static stat_t _exec_json_wait(mpBuf_t *bf)
{
    char *json_string = jc.read_buffer();

    // process it
    json_parse_for_exec(json_string, false); // do NOT execute

    nvObj_t *nv = nv_exec;
    while ((nv != NULL) && (nv->valuetype != TYPE_EMPTY)) {
        // For now we ignore non-BOOL
        if (nv->valuetype == TYPE_BOOL) {
            bool old_value = !fp_ZERO(nv->value); // force it to bool

            nv_get_nvObj(nv);
            bool new_value = !fp_ZERO(nv->value);
            if (old_value != new_value) {
                st_prep_dwell((uint32_t)(0.1 * 1000000.0));// 1ms converted to uSec
                return STAT_OK;
            }
        }
        nv = nv->nx;
    }
    jc.free_buffer();

    if (mp_free_run_buffer()) {
        cm_cycle_end();                                    // free buffer & perform cycle_end if planner is empty
    }
    return (STAT_OK);
}


/*************************************************************************
 * mp_dwell()    - queue a dwell
 * _exec_dwell() - dwell execution
 *
 * Dwells are performed by passing a dwell move to the stepper drivers.
 * When the stepper driver sees a dwell it times the dwell on a separate
 * timer than the stepper pulse timer.
 */
stat_t mp_dwell(float seconds)
{
    mpBuf_t *bf;

    if ((bf = mp_get_write_buffer()) == NULL) {     // get write buffer or fail
        return(cm_panic(STAT_FAILED_GET_PLANNER_BUFFER, "mp_dwell()")); // not ever supposed to fail
    }
    bf->bf_func = _exec_dwell;                      // register callback to dwell start
    bf->block_time = seconds;                       // in seconds, not minutes
    bf->block_state = BLOCK_INITIAL_ACTION;
    mp_commit_write_buffer(BLOCK_TYPE_DWELL);       // must be final operation before exit
    return (STAT_OK);
}

static stat_t _exec_dwell(mpBuf_t *bf)
{
    st_prep_dwell((uint32_t)(bf->block_time * 1000000.0));// convert seconds to uSec
    if (mp_free_run_buffer()) {
        cm_cycle_end();                 // free buffer & perform cycle_end if planner is empty
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
 * mp_get_planner_buffers()  - return # of available planner buffers
 * mp_planner_is_full()      - true if planner has no room for a new block
 * mp_has_runnable_buffer()  - true if next buffer is runnable, indicating motion has not stopped.
 * mp_is_it_phat_city_time() - test if there is time for non-essential processes
 */
uint8_t mp_get_planner_buffers()
{
    return (mb.buffers_available);
}

bool mp_planner_is_full()
{
    // We also need to ensure we have room for another JSON command
    return ((mb.buffers_available < PLANNER_BUFFER_HEADROOM) || (jc.available == 0));
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
    return ((mp.plannable_time <= 0.0) || (PHAT_CITY_TIME < mp.plannable_time));
}

/*
 * mp_planner_callback()
 *
 *  mp_planner_callback()'s job is to invoke backward planning intelligently.
 *  The flow of control and division of responsibilities for planning is:
 *
 *  - mp_aline() receives new Gcode moves and initializes the local variables
 *    for the new buffer.
 *
 *  - mp_planner_callback() is called regularly from the main loop.
 *    It's job is to determine whether or not to call mp_plan_block_list(),
 *    which will back-plan as many blocks as are ready for processing.
 *
 *    mp_planner_callback() also manages planner state - whether the planner
 *    is IDLE, in STARTUP or in one of the running states.
 *
 *  - _plan_block() is the backward planning function for a single buffer.
 *
 *  - Just-in-time forward planning is performed by mp_plan_move() in the plan_exec.cpp runtime executive
 *
 *  Some Items to note:
 *
 *  - At the start of a job the planner should fill up with unplanned blocks before
 *    motion starts. This eliminates an initial move that plans to zero and ensures
 *    the planner gets a "head start" on managing the time in the planner queue.
 *
 *  - It's important to distinguish between the case where the new block is actually
 *    a startup condition and where it's the first block after a stop or a stall.
 *    The planner wants to perform a STARTUP in the first case, but start planning
 *    immediately in the latter cases.
 *
 *  - Feedholds require replanning to occur
 */

stat_t mp_planner_callback()
{
    // Test if the planner has transitioned to an IDLE state
    if ((mb.buffers_available == PLANNER_BUFFER_POOL_SIZE) &&   // detect and set IDLE state
        (cm.motion_state == MOTION_STOP) &&
        (cm.hold_state == FEEDHOLD_OFF)) {
        mp.planner_state = PLANNER_IDLE;
        return (STAT_OK);
    }

    bool _timed_out = mp.block_timeout.isPast();
    if (_timed_out) {
        mp.block_timeout.clear();                   // timer is set on commit_write_buffer()
    }

    if (!mp.request_planning && !_timed_out) {      // Exit if no request or timeout
        return (STAT_OK);
    }

    // Process a planner request or timeout
    if (mp.planner_state == PLANNER_IDLE) {
        mp.p = mb.r;                                // initialize planner pointer to run buffer
        mp.planner_state = PLANNER_STARTUP;
    }
    if (mp.planner_state == PLANNER_STARTUP) {
        if (!mp_planner_is_full() && !_timed_out) {
            return (STAT_OK);                       // remain in STARTUP
        }
        mp.planner_state = PLANNER_PRIMING;
    }
    mp_plan_block_list();
    return (STAT_OK);
}

/*
 *  mp_replan_queue() - reset the blocks in the planner queue and request a planner run
 *
 *  We don't actually need to invalidate back-planning. Only forward planning.
 *
 */
void mp_replan_queue(mpBuf_t *bf)
{
    do {
        if (bf->buffer_state >= MP_BUFFER_PLANNED) {
            bf->buffer_state = MP_BUFFER_PREPPED;            // revert from PLANNED state
        } else {        // If it's not "planned" then it's either PREPPED or earlier.
            break;      // We don't need to adjust it.
        }
    } while ((bf = mp_get_next_buffer(bf)) != mb.r);

    mp.request_planning = true;
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
 *  The override takes effect as close to real-time as possible. Practically, this means about 20 or
 *  so behind the current running move. How it works:
 *
 *    - If the planner is idle just apply the override factor and be done with it. That's easy.
 *    - Otherwise look for the "break point" at 20 ms
 */

void mp_start_feed_override(const float ramp_time, const float override_factor)
{
    cm.mfo_state = MFO_REQUESTED;

    if (mp.planner_state == PLANNER_IDLE) {
        mp.mfo_factor = override_factor;             // that was easy
        return;
    }

    // Assume that the min and max values for override_factor have been validated upstream
    // SUVAT: V = U+AT ==> A = (V-U)/T
    mp.ramp_target = override_factor;
    mp.ramp_dvdt = (override_factor - mp.c->override_factor) / ramp_time;
    mp.mfo_active = true;

    if (fp_NOT_ZERO(mp.ramp_dvdt)) {    // do these things only if you actually have a ramp to run
        mp.p = mp.c;                    // re-position the planner pointer
        mp.ramp_active = true;
        mp.request_planning = true;
    }
}

void mp_end_feed_override(const float ramp_time)
{
    mp_start_feed_override (FEED_OVERRIDE_RAMP_TIME, 1.00);
}

void mp_start_traverse_override(const float ramp_time, const float override_factor)
{
    return;
}

void mp_end_traverse_override(const float ramp_time)
{
    return;
}

/*
 * mp_planner_time_accounting() - gather time in planner
 */

void mp_planner_time_accounting()
{
    mpBuf_t *bf = mb.r;                             // start with run buffer

    // check the run buffer to see if anything is running. Might not be
    if (bf->buffer_state != MP_BUFFER_RUNNING) {    // this is not an error condition
        return;
    }
    mp.plannable_time = 0; //UPDATE_BF_MS(bf); //+++++
    while ((bf = bf->nx) != mb.r) {
        if (bf->buffer_state == MP_BUFFER_EMPTY || bf->plannable == true) {
            break;
        }
        mp.plannable_time += bf->block_time;
    }
    UPDATE_MP_DIAGNOSTICS //+++++
}

/**** PLANNER BUFFER PRIMITIVES ************************************************************
 *
 *  Planner buffers are used to queue and operate on Gcode blocks. Each buffer contains
 *  one Gcode block which may be a move, an M code, or other command that must be
 *  executed synchronously with movement.
 *
 *  The planner queue (mb) is a circular queue of planner buffers (bf's). Each block has a
 *  pointer to the next block (nx), and one to the previous block (pv).
 *
 *  It's useful to get terms straight or it can get confusing.
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
 *  New blocks are populated by (1) getting a write buffer, (2) populating the buffer,
 *  then (3) placing it in the queue (commit write buffer). If an exception occurs
 *  during step (2) you can unget the write buffer before queuing it, which returns
 *  it to the pool of available buffers. (NB: Unget is currently unused but left in.)
 *
 *  The RUN buffer may be retrieved once for simple commands, or multiple times for
 *  long-running commands such as moves that get called multiple times. The first
 *  retrieval (get run buffer) will return the new run buffer. Subsequent retrievals
 *  will return the same buffer until it's state changes to complete. When the command
 *  is complete the run buffer is returned to the pool by freeing it.
 *
 * Notes:
 *  The write buffer pointer only moves forward on mp_commit_write_buffer, and the
 *  run buffer pointer only moves forward on mp_free_run_buffer().
 *  Tests, gets and unget have no effect on the pointers.
 *
 * Functions Provided:
 *   _clear_buffer(bf)        Zero the contents of a buffer
 *
 *   mp_init_buffers()        Initialize or reset buffers
 *
 *   mp_get_prev_buffer(bf)   Return pointer to the previous buffer in the linked list
 *   mp_get_next_buffer(bf)   Return pointer to the next buffer in the linked list
 *
 *   mp_get_write_buffer()    Get pointer to next available write buffer
 *                            Return pointer or NULL if no buffer available.
 *
 *   mp_unget_write_buffer()  Free write buffer if you decide not to commit it.
 *
 *   mp_commit_write_buffer() Commit the write buffer to the queue.
 *                            Advance write pointer & changes buffer state.
 *
 *                            *** WARNING *** The calling routine must NOT use the write
 *                            buffer once it has been committed as it may be processed
 *                            and freed (cleared) before the commit function returns.
 *
 *   mp_get_run_buffer()      Get pointer to the next or current run buffer.
 *                            Return a new run buffer if prev buf was ENDed.
 *                            Return same buf if called again before ENDing.
 *                            Return NULL if no buffer available.
 *                            This behavior supports continuations (iteration).
 *
 *   mp_free_run_buffer()     Release the run buffer & return to buffer pool.
 *                            Return true if queue is empty, false otherwise.
 *                            This is useful for doing queue empty / end move functions.
 *
 * UNUSED BUT PROVIDED FOR REFERENCE:
 *   mp_copy_buffer(bf,bp)    Copy the contents of bp into bf - preserves links.
 */

// Also clears unlocked, so the buffer cannot be used
static inline void _clear_buffer(mpBuf_t *bf)
{
    // Note: bf->bf_func is the first address we wish to clear as we must preserve
    // the pointers and buffer number during interrupts

    // We'll have to figure something else out for C, sorry.
    bf->reset();
}

void mp_init_buffers(void)
{
    mpBuf_t *pv, *nx;

    //memset(&mb, 0, sizeof(mb));                     // clear all values, pointers and status
    mb.magic_start = MAGICNUM;
    mb.magic_end = MAGICNUM;

    mb.w = &mb.bf[0];                               // init all buffer pointers
    mb.r = &mb.bf[0];
    pv = &mb.bf[PLANNER_BUFFER_POOL_SIZE-1];
    for (uint8_t i=0; i < PLANNER_BUFFER_POOL_SIZE; i++) {
        _clear_buffer(&mb.bf[i]);
        uint8_t nx_i = ((i<(PLANNER_BUFFER_POOL_SIZE-1))?(i+1):0); // buffer incr & wrap

        mb.bf[i].buffer_number = i;                 //+++++ number it for diagnostics only (otherwise not used)

        nx = &mb.bf[nx_i];
        mb.bf[i].nx = nx;                           // setup ring pointers
        mb.bf[i].pv = pv;

        pv = &mb.bf[i];
    }
    mb.buffers_available = PLANNER_BUFFER_POOL_SIZE;

//    mb.entry_changed = false;

    // Now handle the two "stub buffers" in the runtime structure.
    mr.bf[0].nx = &mr.bf[1];
    mr.bf[1].nx = &mr.bf[0];
    mr.r = &mr.bf[0];
    mr.p = &mr.bf[1];
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
        _clear_buffer(mb.w);        // ++++RG this is redundant, it was just cleared in mp_free_run_buffer
        mb.w->buffer_state = MP_BUFFER_INITIALIZING;
        mb.buffers_available--;
        return (mb.w);
    }
    // The no buffer condition always causes a panic - invoked by the caller
    rpt_exception(STAT_FAILED_TO_GET_PLANNER_BUFFER, "mp_get_write_buffer()");
    return (NULL);
}

void mp_unget_write_buffer()        // mark buffer as empty and adjust free buffer count
{
    if (mb.w->buffer_state != MP_BUFFER_EMPTY) {  // safety. Can't unget an empty buffer
        mb.w->buffer_state = MP_BUFFER_EMPTY;
        mb.buffers_available++;
    }
}

/*** WARNING ***
* The function calling mp_commit_write_buffer() must NOT use the write buffer once it has
* been committed. Interrupts may use the buffer immediately, invalidating its contents.
*/

void mp_commit_write_buffer(const blockType block_type)
{
    mb.w->block_type = block_type;
    mb.w->block_state = BLOCK_INITIAL_ACTION;

    if (block_type == BLOCK_TYPE_ALINE) {
        if (cm.motion_state == MOTION_STOP) {
            cm_set_motion_state(MOTION_PLANNING);
        }
    } else {
        if ((mp.planner_state > PLANNER_STARTUP) && (cm.hold_state == FEEDHOLD_OFF)) {
            // NB: BEWARE! the exec may result in the planner buffer being
            // processed IMMEDIATELY and then freed - invalidating the contents
            st_request_forward_plan();                // request an exec if the runtime is not busy
        }
    }
    mb.w->plannable = true;                     // enable block for planning
    mp.request_planning = true;
    mb.w = mb.w->nx;                            // advance write buffer pointer
    mp.block_timeout.set(BLOCK_TIMEOUT_MS);     // reset the block timer
    qr_request_queue_report(+1);                // request QR and add to "added buffers" count
}

// Note: mp_get_run_buffer() is only called by mp_exec_move(), which is inside an interrupt
mpBuf_t * mp_get_run_buffer()
{
    // EMPTY is the one case where nothing is returned. This is not an error
    if (mb.r->buffer_state == MP_BUFFER_EMPTY) {
        return (NULL);
    }
    // Otherwise return the buffer. Let mp_exec_move() manage the state machine to sort out:
    //  (1) is the the first time the run buffer has been retrieved?
    //  (2) is the buffer in error - i.e. not yet ready for running?
    return (mb.r);
}

// Note: mp_free_run_buffer() is only called from mp_exec_XXX, which are within an interrupt
bool mp_free_run_buffer()           // EMPTY current run buffer & advance to the next
{
    _audit_buffers();               // diagnostic audit for buffer chain integrity (only runs in DEBUG mode)

    mpBuf_t *r = mb.r;
    mb.r = mb.r->nx;                // advance to next run buffer
    _clear_buffer(r);               // clear it out (& reset unlocked and set MP_BUFFER_EMPTY)

    mb.buffers_available++;
    qr_request_queue_report(-1);    // request a QR and add to the "removed buffers" count
    return (mb.w == mb.r);          // return true if the queue emptied
}

/* UNUSED FUNCTIONS - left in for completeness and for reference
void mp_copy_buffer(mpBuf_t *bf, const mpBuf_t *bp)
{
    // copy contents of bp to by while preserving pointers in bp
    memcpy((void *)(&bf->bf_func), (&bp->bf_func), sizeof(mpBuf_t) - (sizeof(void *) * 2));
}
*/

/************************************************************************************
 * mp_dump_planner
 * _planner_report()
 * _audit_buffers() - a DEBUG diagnostic
 */

//#define __DUMP_PLANNER

#ifdef __DUMP_PLANNER
void mp_dump_planner(mpBuf_t *bf_start)   // starting at bf
{
    mpBuf_t *bf = bf_start;

    printf ("Buf, Line, State, Hint, Planbl, Iter, Tmove, Tplan, Ovr, Thr, Len, Ve, Vc, Vx, Vemax, Vcset, Vcmax, Vxmax, Vjt\n");

    do {
        printf ("%d,",    (int)bf->buffer_number);
        printf ("%d,",    (int)bf->linenum);
        printf ("%d,",    (int)bf->buffer_state);
        printf ("%d,",    (int)bf->hint);
        printf ("%d,",    (int)bf->plannable);
        printf ("%d,",    (int)bf->iterations);

        printf ("%1.2f,", bf->block_time_ms);
        printf ("%1.2f,", bf->plannable_time_ms);
        printf ("%1.3f,", bf->override_factor);
        printf ("%1.3f,", bf->throttle);
        printf ("%1.5f,", bf->length);
        printf ("%1.0f,", bf->pv->exit_velocity);
        printf ("%1.0f,", bf->cruise_velocity);
        printf ("%1.0f,", bf->exit_velocity);
        printf ("%1.0f,", bf->pv->exit_vmax);
        printf ("%1.0f,", bf->cruise_vset);
        printf ("%1.0f,", bf->cruise_vmax);
        printf ("%1.0f,", bf->exit_vmax);
        printf ("%1.0f\n", bf->junction_vmax);

        bf = bf->nx;
    } while (bf != bf_start);
}
#endif // __DUMP_PLANNER

#if 0 && defined(DEBUG)

#warning DEBUG TRAPS ENABLED

#pragma GCC optimize ("O0")

//static void _planner_report(const char *msg)
//{
//    rpt_exception(STAT_PLANNER_ASSERTION_FAILURE, msg);
//
//    for (uint8_t i=0; i<PLANNER_BUFFER_POOL_SIZE; i++) {
//        printf("{\"er\":{\"stat\":%d, \"type\":%d, \"lock\":%d, \"plannable\":%d",
//                mb.bf[i].buffer_state,
//                mb.bf[i].block_type,
//                mb.bf[i].locked,
//                mb.bf[i].plannable);
//        if (&mb.bf[i] == mb.r) {
//            printf(", \"RUN\":t");}
//        if (&mb.bf[i] == mb.w) {
//            printf(", \"WRT\":t");}
//        printf("}}\n");
//    }
//}

/*
 * _audit_buffers() - diagnostic to determine if buffers are sane
 */

static void _audit_buffers()
{
    __disable_irq();

    // Current buffer should be in the running state.
    if (mb.r->buffer_state != MP_BUFFER_RUNNING) {
//        _planner_report("buffer audit1");
        __NOP();
//        _debug_trap();
    }

    // Check that the next from the previous is correct.
    if (mb.r->pv->nx != mb.r || mb.r->nx->pv != mb.r){
//        _planner_report("buffer audit2");
        _debug_trap("buffer audit2");
    }

    // Now check every buffer, in order we would execute them.
    mpBuf_t *bf = mb.r->nx;
    while (bf != mb.r) {
        // Check that the next from the previous is correct.
        if (bf->pv->nx != bf || bf->nx->pv != bf){
//            _planner_report("buffer audit3");
            _debug_trap("buffer audit3");
        }

        // Order should be:
        //  - MP_BUFFER_RUNNING
        //  - MP_BUFFER_PLANNED (zero or more)
        //  - MP_BUFFER_NOT_PLANNED (zero or more)
        //  - MP_BUFFER_EMPTY (zero or more up until mb.r)
        //  - no more

        // After RUNNING, we can PREPPED, PLANNED, INITED, IN_PROCESS, or EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_RUNNING &&
            bf->buffer_state != MP_BUFFER_PREPPED &&
            bf->buffer_state != MP_BUFFER_PLANNED &&
            bf->buffer_state != MP_BUFFER_INITIALIZING &&
            bf->buffer_state != MP_BUFFER_IN_PROCESS &&
            bf->buffer_state != MP_BUFFER_EMPTY) {
            // Exception: MP_BUFFER_INITIALIZING and MP_BUFFER_IN_PROCESS are allowed, but we may want to watch for it:
            if ((bf->buffer_state == MP_BUFFER_INITIALIZING) || (bf->buffer_state == MP_BUFFER_IN_PROCESS)) {
                __NOP();
            } else {
//                _planner_report("buffer audit4");
                _debug_trap("buffer audit4");
            }
        }

        // After PLANNED, we can see PREPPED, INITED, IN_PROCESS, or EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_PLANNED &&
            bf->buffer_state != MP_BUFFER_PREPPED &&
            bf->buffer_state != MP_BUFFER_INITIALIZING &&
            bf->buffer_state != MP_BUFFER_IN_PROCESS &&
            bf->buffer_state != MP_BUFFER_EMPTY) {
//            _planner_report("buffer audit5");
            _debug_trap("buffer audit5");
        }

        // After PREPPED, we can see PREPPED, INITED, IN_PROCESS, or EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_PREPPED &&
            bf->buffer_state != MP_BUFFER_PREPPED &&
            bf->buffer_state != MP_BUFFER_INITIALIZING &&
            bf->buffer_state != MP_BUFFER_IN_PROCESS &&
            bf->buffer_state != MP_BUFFER_EMPTY) {
//            _planner_report("buffer audit6");
            _debug_trap("buffer audit6");
        }

        // After EMPTY, we should only see EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_EMPTY && bf->buffer_state != MP_BUFFER_EMPTY) {
//            _planner_report("buffer audit7");
            _debug_trap("buffer audit7");
        }
        // Now look at the next one.
        bf = bf->nx;
    }
    __enable_irq();
}

#pragma GCC reset_options

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
