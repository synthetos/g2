/*
 * planner.h - cartesian trajectory planning and motion execution
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2016 Robert Giseburt
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
/*x
 * --- Planner Background ---
 *
 *  The planner is a complicated beast that takes a lot of things into account. 
 *  Planner documentation is scattered about and co-located with the functions
 *  that perform the actions. Key files are:
 *
 *  - planner.h     - This file has defines, structures and prototypes. What you would expect
 *  - planner.cpp   - Core and common functions, queue handling, JSON and command handlers
 *  - plan_line.cpp - Move planning and queuing, backward planning functions
 *  - plan_zoid.cpp - Move forward planning, velocity contour calculations and crazy math
 *  - plan_exec.cpp - Runtime execution functions, calls zoid's forward planning functions
 *  - stepper.cpp/h - Real-time step generation, segment loading, pulls from plan_exec
 *  - plan_arc.cpp/h- Arc calculation and runtime functions - layer above the rest of this
 *
 * --- Planner Overview ---
 *
 * At high level the planner's job is to reconstruct smooth motion from a set of linear 
 * approximations while observing and operating within the physical constraints of the 
 * machine and the physics of motion. Gcode - which consists of a series of into linear 
 * motion segments - is interpreted, queued to the planner, and joined together to produce 
 * continuous, synchronized motion. Non-motion commands such as pauses (dwells) and 
 * peripheral controls such as spindles can also be synchronized in the queue. Arcs are 
 * just a special case consisting of many linear moves. Arcs are not interpreted directly.
 *
 * The planner sits in the middle of three system layers: 
 *  - The Gcode interpreter and canonical machine (the 'model'), which feeds...
 *  - The planner - taking generic commands from the model and queuing them for...
 *  - The runtime layer - pulling from the planner and driving stepper motors or other devices
 * 
 * The planner queue is the heart of the planner. It's a circular list of ~48 complex structures
 * that carry the state of the system needs to execute a linear motion, run a pre-planned command,
 * like turning on a spindle, or executing an arbitrary JSON command such as an active comment.
 *
 * The queue can be viewed as a list of instructions that will execute in exact sequence.
 * Some instructions control motion and need to be joined to their forward and backwards 
 * neighbors so that position, velocity, acceleration, and jerk constraints are not 
 * violated when moving from one motion to the next. 
 *
 * Others are "commands" that are actually just function callbacks that happen to execute
 * at a particular point in time (synchronized with motion commands). Commands can control
 * anything you can reasonably program, such as digital IO, serial communications, or 
 * interpreted commands encoded in JSON.
 *
 * The buffers in the planner queue are treated as a 'closure' - with all state needed for
 * proper execution carried in the planner structure. This is important as it keeps
 * model state coherent in a heavily pipelined system. The local copy of the Gcode
 * model is carried in the gm structure that is part of each planner buffer.
 * See header notes in planner.cpp for more details.
 *
 * The planner is entered by calling one of:
 *  - mp_aline()         - plan and queue a move with acceleration management
 *  - mp_dwell()         - plan and queue a pause (dwell) to the planner queue
 *  - mp_queue_command() - queue a canned command
 *  - mp_json_command()  - queue a JSON command for run-time interpretation and execution (M100)  
 *  - mp_json_wait()     - queue a JSON wait for run-time interpretation and execution (M101)
 *  - 
 * In addition, cm_arc_feed() valaidates and sets up a arc paramewters and calls mp_aline() 
 * repeatedly to spool out the arc segments into the planner queue.
 *
 * All the above queueing commands other than mp_aline() are relatively trivial; they just
 * post callbacks into the next available planner buffer. Command functions are in 2 parts: 
 * the part that posts to the queue, and the callback that is executed when the command is 
 * finally reached in the queue - the _exec().
 *
 * All mp_aline() does is some preliminary math and then posts an initialized buffer to 
 * the planner queue. The rest of the move planning operations takes place in background;
 * via mp_planner_callback() called from the main loop, and as 'pulls' from the runtime 
 * stepper operations.
 *
 * Motion planning is separated into backward planning and forward planning stages. 
 * Backward planning is initiated by mp_planner_callback() which is called repeatedly 
 * from the main loop. Backwards planning is performed by mp_plan_block_list() and 
 * _plan_block(). It starts at the most recently arrived Gcode block. Backward 
 * planning can occur multiple times for a given buffer, as new moves arriving 
 * can make the motion profile more optimal.
 *
 * Backward planning uses velocity and jerk constraints to set maximum entry,
 * travel (cruise) and exit velocities for the moves in the queue. In addition, 
 * it observes the maximum cornering velocities that adjoining moves can sustain 
 * in a corner or a 'kink' to ensure that the jerk limit of any axis participating 
 * in the move is not violated. See mp_planner_callback() header comments for more detail.
 *
 * Forward planning is performed just-in-time and only once, right before the 
 * planner runtime needs the next buffer. Forward planning provides the final
 * contouring of the move. It is invoked by mp_forward_plan() and executed by 
 * mp_calculate_ramps() in plan_zoid.cpp.
 *
 * Planner timing operates at a few different levels:
 *
 *  - New lines of ASCII containing commands and moves arriving from the USB are 
 *    parsed and executed as the lowest priority background task from the main loop.
 *
 *  - Backward planning is invoked by a main loop callback, so it also executes as 
 *    a background task, albeit a higher priority one.
 *
 *  - Forward planning and the ultimate preparation of the move for the runtime runs 
 *    as an interrupt as a 'pull' from the planner queue that uses a series of 
 *    interrupts at progressively lower priorities to ensure that the next planner 
 *    buffer is ready before the runtime runs out of forward-planned moves and starves.
 *
 * Some other functions performed by the planner include:
 *
 *  - Velocity throttling to ensure that very short moves do not execute faster 
 *    than the serial interface can deliver them
 *
 *  - Feed hold and cycle start (resume) operations
 *
 *  - Feed rate override functions and replanning
 *
 * Some terms that are useful that we try to use consistently:
 *
 *  - buffer  - in this context a planner buffer holding a move or a command: mb._ or bf
 *  - block   - a data structure for planning or runtime control. See mp_calculate_ramps() comments
 *  - move    - a linear Gcode move, typically from a G0 or G1 code
 *  - command - a non-move executable in the planner
 *  - group   - a collection of moves or commands that are treated as a unit
 *  - line    - a line of ASCII gcode or arbitrary text
 *  - bootstrap - the startup period where the planner collects moves but does not yet execute them
 */

#ifndef PLANNER_H_ONCE
#define PLANNER_H_ONCE

#include "canonical_machine.h"    // used for GCodeState_t

using Motate::Timeout;

/*
 * Enums and other type definitions
 *
 * All the enums that equal zero must be zero. Don't change them
 */

typedef void (*cm_exec_t)(float[], bool[]); // callback to canonical_machine execution function

typedef enum {                      // planner operating state
    PLANNER_IDLE = 0,               // planner and movement are idle
    PLANNER_STARTUP,                // ingesting blocks before movement is started
    PLANNER_PRIMING,                // preparing new moves for planning ("stitching")
    PLANNER_BACK_PLANNING           // plan by planning all blocks, from the newest added to the running block
} plannerState;

typedef enum {                      // bf->buffer_state values in incresing order so > and < can be used
    MP_BUFFER_EMPTY = 0,            // buffer is available for use (MUST BE 0)
    MP_BUFFER_INITIALIZING,         // buffer has been checked out and is being initialzed by aline() or a command
    MP_BUFFER_IN_PROCESS,           // planning is in progress - at least vmaxes have been set
    MP_BUFFER_PREPPED,              // buffer ready for final planning; velocities have been set
    MP_BUFFER_PLANNED,              // buffer fully planned. May still be replanned
    MP_BUFFER_RUNNING,              // current running buffer
    MP_BUFFER_POLAND,               // Hitler used Poland as a buffer state
    MP_BUFFER_UKRAINE               // Later Stalin did the same to Ukraine
} bufferState;

typedef enum {                      // bf->block_type values
    BLOCK_TYPE_NULL = 0,            // MUST=0  null move - does a no-op
    BLOCK_TYPE_ALINE = 1,           // MUST=1  acceleration planned line
    BLOCK_TYPE_COMMAND = 2,         // MUST=2  general command
    BLOCK_TYPE_DWELL,               // Gcode dwell
    BLOCK_TYPE_JSON_WAIT,           // JSON wait command
    BLOCK_TYPE_TOOL,                // T command (T, not M6 tool change)
    BLOCK_TYPE_SPINDLE_SPEED,       // S command
    BLOCK_TYPE_STOP,                // program stop
    BLOCK_TYPE_END                  // program end
} blockType;

typedef enum {
    BLOCK_INACTIVE = 0,             // block is inactive (MUST BE ZERO)
    BLOCK_INITIAL_ACTION,           // initial value if you need an initialization
    BLOCK_ACTIVE                    // run state
} blockState;

typedef enum {
    SECTION_HEAD = 0,               // acceleration
    SECTION_BODY,                   // cruise
    SECTION_TAIL                    // deceleration
} moveSection;
#define SECTIONS 3

typedef enum {
    SECTION_OFF = 0,                // section inactive
    SECTION_NEW,                    // uninitialized section
    SECTION_RUNNING                 // initilized and running
} sectionState;

typedef enum {                      // code blocks for planning and trapezoid generation
    NO_HINT = 0,                    // block is not hinted
    COMMAND_BLOCK,                  // this block is a command
    PERFECT_ACCELERATION,           // head-only acceleration at jerk or cannot be improved
    PERFECT_DECELERATION,           // tail-only deceleration at jerk or cannot be improved
    PERFECT_CRUISE,                 // body-only cruise: Ve = Vc = Vx != 0
    MIXED_ACCELERATION,             // kinked acceleration reaches and holds cruise (HB)
    MIXED_DECELERATION,             // kinked deceleration starts with a cruise region (BT)

    // The rest are for reporting from the zoid function what was selected.
    ZERO_VELOCITY,                  // Ve = Vc = Vx = 0
    ZERO_BUMP,                      // Ve = Vx = 0, Vc != 0
    SYMMETRIC_BUMP,                 // (Ve = Vx) < Vc
    ASYMMETRIC_BUMP,                // (Ve != Vx) < Vc
} blockHint;

typedef enum {
    ZOID_EXIT_NULL = 0,
    ZOID_EXIT_1a,
    ZOID_EXIT_1c,
    ZOID_EXIT_1d,
    ZOID_EXIT_2a,
    ZOID_EXIT_2c,
    ZOID_EXIT_2d,
    ZOID_EXIT_3c,
    ZOID_EXIT_3s,
    ZOID_EXIT_3s2,
    ZOID_EXIT_3d2,
    ZOID_EXIT_3a2
} zoidExitPoint;

/*** Most of these factors are the result of a lot of tweaking. Change with caution.***/

#define PLANNER_BUFFER_POOL_SIZE    (48)                // Suggest 12 min. Limit is 255
#define PLANNER_BUFFER_HEADROOM     (4)                 // Buffers to reserve in planner before processing new input line
#define JERK_MULTIPLIER             ((float)1000000)    // DO NOT CHANGE - must always be 1 million

#define JUNCTION_INTEGRATION_MIN    (0.05)              // minimum allowable setting
#define JUNCTION_INTEGRATION_MAX    (5.00)              // maximum allowable setting

#define MIN_SEGMENT_MS              ((float)0.75)       // minimum segment milliseconds
#define NOM_SEGMENT_MS              ((float)1.5)        // nominal segment ms (at LEAST MIN_SEGMENT_MS * 2)
#define MIN_BLOCK_MS                ((float)1.5)        // minimum block (whole move) milliseconds
#define BLOCK_TIMEOUT_MS            ((float)30.0)       // MS before deciding there are no new blocks arriving
#define PHAT_CITY_MS                ((float)100.0)      // if you have at least this much time in the planner

#define NOM_SEGMENT_TIME            ((float)(NOM_SEGMENT_MS / 60000))       // DO NOT CHANGE - time in minutes
#define NOM_SEGMENT_USEC            ((float)(NOM_SEGMENT_MS * 1000))        // DO NOT CHANGE - time in microseconds
#define MIN_SEGMENT_TIME            ((float)(MIN_SEGMENT_MS / 60000))       // DO NOT CHANGE - time in minutes
#define MIN_BLOCK_TIME              ((float)(MIN_BLOCK_MS / 60000))         // DO NOT CHANGE - time in minutes
#define PHAT_CITY_TIME              ((float)(PHAT_CITY_MS / 60000))         // DO NOT CHANGE - time in minutes

#define FEED_OVERRIDE_ENABLE        false               // initial value
#define FEED_OVERRIDE_MIN           (0.05)              // 5% minimum
#define FEED_OVERRIDE_MAX           (2.00)              // 200% maximum
#define FEED_OVERRIDE_RAMP_TIME     (0.500/60)          // ramp time for feed overrides
#define FEED_OVERRIDE_FACTOR        (1.00)              // initial value

#define TRAVERSE_OVERRIDE_ENABLE    false               // initial value
#define TRAVERSE_OVERRIDE_MIN       (0.05)              // 5% minimum
#define TRAVERSE_OVERRIDE_MAX       (1.00)              // 100% maximum
#define TRAVERSE_OVERRIDE_FACTOR    (1.00)              // initial value

//// Specialized equalities for comparing velocities with tolerances
//// These determine allowable velocity discontinuities between blocks (among other tests)
//// RG: Simulation shows +-0.001 is about as much as we should allow.
//      VELOCITY_EQ(v0,v1) reads: "True if v0 is within 0.0001 of v1"
//      VELOCITY_LT(v0,v1) reads: "True if v0 is less than v1 by at least 0.0001"
#define VELOCITY_EQ(v0,v1) ( fabs(v0-v1) < 0.0001 )
#define VELOCITY_LT(v0,v1) ( (v1 - v0) > 0.0001 )

#define Vthr2 300.0
#define Veq2_hi 10.0
#define Veq2_lo 1.0
#define VELOCITY_ROUGHLY_EQ(v0,v1) ( (v0 > Vthr2) ? fabs(v0-v1) < Veq2_hi : fabs(v0-v1) < Veq2_lo )

//#define ASCII_ART(s)            xio_writeline(s)
#define ASCII_ART(s)
//#define UPDATE_BF_DIAGNOSTICS(bf) { bf->block_time_ms = bf->block_time*60000; bf->plannable_time_ms = bf->plannable_time*60000; }
#define UPDATE_MP_DIAGNOSTICS     { mp.plannable_time_ms = mp.plannable_time*60000; }

/*
 *  Planner structures
 *
 *  You should be aware of the distinction between 'buffers' and 'blocks'
 *  Please refer to header comments in for important details on buffers and blocks
 *    - plan_zoid.cpp / mp_calculate_ramps()
 *    - plan_exec.cpp / mp_exec_aline()
 */

struct mpBuffer_to_clear {
    // Note: _clear_buffer() zeros all data from this point down
    stat_t (*bf_func)(struct mpBuffer *bf); // callback to buffer exec function
    cm_exec_t cm_func;              // callback to canonical machine execution function

    //+++++ DIAGNOSTICS for easier debugging
    uint32_t linenum;               // mirror of bf->gm.linenum
    int iterations;
    float block_time_ms;
    float plannable_time_ms;        // time in planner
    float plannable_length;         // length in planner
    int8_t meet_iterations;         // iterations needed in _get_meet_velocity
    //+++++ to here

    bufferState buffer_state;       // used to manage queuing/dequeuing
    blockType block_type;           // used to dispatch to run routine
    blockState block_state;         // move state machine sequence
    blockHint hint;                 // hint the block for zoid and other planning operations. Must be accurate or NO_HINT

    // block parameters
    float unit[AXES];               // unit vector for axis scaling & planning
    bool axis_flags[AXES];          // set true for axes participating in the move & for command parameters

    bool plannable;                 // set true when this block can be used for planning

    float length;                   // total length of line or helix in mm
    float block_time;               // computed move time for entire block (move)
    float override_factor;          // feed rate or rapid override factor for this block ("override" is a reserved word)

    // We are removing all entry_* values.
    // To get the entry_* values, look at pv->exit_* or mr.exit_*

    // *** SEE NOTES ON THESE VARIABLES, in aline() ***
    float cruise_velocity;          // cruise velocity requested & achieved
    float exit_velocity;            // exit velocity requested for the move
                                    // is also the entry velocity of the *next* move

    float cruise_vset;              // cruise velocity requested for move - prior to overrides
    float cruise_vmax;              // cruise max velocity adjusted for overrides
    float exit_vmax;                // max exit velocity possible for this move
                                    // is also the maximum entry velocity of the next move

    float absolute_vmax;            // fastest this block can move w/o exceeding constraints
    float junction_vmax;            // maximum the exit velocity can be to go through the junction
                                    // between the NEXT BLOCK AND THIS ONE

    float jerk;                     // maximum linear jerk term for this move
    float jerk_sq;                  // Jm^2 is used for planning (computed and cached)
    float recip_jerk;               // 1/Jm used for planning (computed and cached)
    float sqrt_j;                   // sqrt(jM) used for planning (computed and cached)
    float q_recip_2_sqrt_j;         // (q/(2 sqrt(jM))) where q = (sqrt(10)/(3^(1/4))), used in length computations (computed and cached)

    GCodeState_t gm;                // Gcode model state - passed from model, used by planner and runtime

    void reset() {
        //memset((void *)(this), 0, sizeof(mpBuffer_to_clear));
        
        bf_func = nullptr;
        cm_func = nullptr;

        linenum = 0;
        iterations = 0;
        block_time_ms = 0;
        plannable_time_ms = 0;
        plannable_length = 0;
        meet_iterations = 0;

        buffer_state = MP_BUFFER_EMPTY;
        block_type = BLOCK_TYPE_NULL;
        block_state = BLOCK_INACTIVE;
        hint = NO_HINT;

        for (uint8_t i = 0; i< AXES; i++) {
            unit[i] = 0;
            axis_flags[i] = 0;
        }

        plannable = false;
        length  = 0.0;
        block_time = 0.0;
        override_factor = 0.0;
        cruise_velocity = 0.0;
        exit_velocity = 0.0;
        cruise_vset = 0.0;
        cruise_vmax = 0.0;
        exit_vmax = 0.0;
        absolute_vmax = 0.0;
        junction_vmax = 0.0;
        jerk = 0.0;
        jerk_sq = 0.0;
        recip_jerk = 0.0;
        sqrt_j = 0.0;
        q_recip_2_sqrt_j = 0.0;
        gm.reset();
    }
};

typedef struct mpBuffer : mpBuffer_to_clear { // See Planning Velocity Notes for variable usage

    // *** CAUTION *** These two pointers are not reset by _clear_buffer()
    struct mpBuffer *pv;            // static pointer to previous buffer
    struct mpBuffer *nx;            // static pointer to next buffer
    uint8_t buffer_number;          //+++++ DIAGNOSTIC for easier debugging
} mpBuf_t;

typedef struct mpBufferPool {       // ring buffer for sub-moves
    magic_t magic_start;            // magic number to test memory integrity

    mpBuf_t *r;                     // run buffer pointer
    mpBuf_t *w;                     // write buffer pointer
    uint8_t buffers_available;      // running count of available buffers
    mpBuf_t bf[PLANNER_BUFFER_POOL_SIZE];// buffer storage

    magic_t magic_end;
} mpBufferPool_t;

typedef struct mpMotionPlannerSingleton {  // common variables for planning (move master)
    magic_t magic_start;            // magic number to test memory integrity

    //+++++ DIAGNOSTICS
    float run_time_remaining_ms;
    float plannable_time_ms;

    // planner position
    float position[AXES];           // final move position for planning purposes

    // timing variables
    float run_time_remaining;       // time left in runtime (including running block)
    float plannable_time;           // time in planner that can actually be planned

    // planner state variables
    plannerState planner_state;     // current state of planner
    bool request_planning;          // set true to request backplanning
    bool backplanning;              // true if planner is in a back-planning pass
    bool mfo_active;                // true if mfo override is in effect
    bool ramp_active;               // true when a ramp is occurring
    bool entry_changed;             // mark if exit_velocity changed to invalidate next block's hint

    // feed overrides and ramp variables (these extend the variables in cm.gmx)
    float mfo_factor;               // runtime override factor
    float ramp_target;
    float ramp_dvdt;

    // objects
    Timeout block_timeout;          // Timeout object for block planning

    // planner pointers
    mpBuf_t *p;                     // planner buffer pointer
    mpBuf_t *c;                     // pointer to buffer immediately following critical region
    mpBuf_t *planning_return;       // buffer to return to once back-planning is complete

    magic_t magic_end;
} mpMotionPlannerSingleton_t;

typedef struct mpBlockRuntimeBuf {  // Data structure for just the parts of RunTime that we need to plan a BLOCK
    struct mpBlockRuntimeBuf *nx;   // singly-linked-list

    float head_length;              // copies of bf variables of same name
    float body_length;
    float tail_length;

    float head_time;                // copies of bf variables of same name
    float body_time;
    float tail_time;

    float cruise_velocity;          // velocity at the end of the head and the beginning of the tail
    float exit_velocity;            // velocity at the end of the move
} mpBlockRuntimeBuf_t;

typedef struct mpMotionRuntimeSingleton {    // persistent runtime variables
//  uint8_t (*run_move)(struct mpMoveRuntimeSingleton *m); // currently running move - left in for reference
    magic_t magic_start;                // magic number to test memory integrity
    blockState block_state;             // state of the overall move
    moveSection section;                // what section is the move in?
    sectionState section_state;         // state within a move section

    float unit[AXES];                   // unit vector for axis scaling & planning
    bool axis_flags[AXES];              // set true for axes participating in the move
    float target[AXES];                 // final target for bf (used to correct rounding errors)
    float position[AXES];               // current move position
    float waypoint[SECTIONS][AXES];     // head/body/tail endpoints for correction

    float target_steps[MOTORS];         // current MR target (absolute target as steps)
    float position_steps[MOTORS];       // current MR position (target from previous segment)
    float commanded_steps[MOTORS];      // will align with next encoder sample (target from 2nd previous segment)
    float encoder_steps[MOTORS];        // encoder position in steps - ideally the same as commanded_steps
    float following_error[MOTORS];      // difference between encoder_steps and commanded steps

    mpBlockRuntimeBuf_t *r;             // block that is running
    mpBlockRuntimeBuf_t *p;             // block that is being planned, p might == r
    mpBlockRuntimeBuf_t bf[2];          // buffer holding the two blocks

    float entry_velocity;               // entry values for the currently running block

    float segments;                     // number of segments in line (also used by arc generation)
    uint32_t segment_count;             // count of running segments
    float segment_velocity;             // computed velocity for aline segment
    float segment_time;                 // actual time increment per aline segment

    float forward_diff_1;               // forward difference level 1
    float forward_diff_2;               // forward difference level 2
    float forward_diff_3;               // forward difference level 3
    float forward_diff_4;               // forward difference level 4
    float forward_diff_5;               // forward difference level 5

    GCodeState_t gm;                    // gcode model state currently executing

    magic_t magic_end;
} mpMotionRuntimeSingleton_t;

// Reference global scope structures
extern mpBufferPool_t mb;               // buffer pool management
extern mpMotionPlannerSingleton_t mp;   // context for block planning
extern mpMotionRuntimeSingleton_t mr;   // context for block runtime

/*
 * Global Scope Functions
 */

//planner.cpp functions

void planner_init(void);
void planner_reset(void);
void planner_init_assertions(void);
stat_t planner_test_assertions(void);

void mp_halt_runtime(void);
void mp_flush_planner(void);
void mp_set_planner_position(uint8_t axis, const float position);
void mp_set_runtime_position(uint8_t axis, const float position);
void mp_set_steps_to_runtime_position(void);

void mp_queue_command(void(*cm_exec_t)(float[], bool[]), float *value, bool *flag);
stat_t mp_runtime_command(mpBuf_t *bf);

stat_t mp_json_command(char *json_string);
stat_t mp_json_wait(char *json_string);
stat_t mp_json_command_immediate(char *json_string);

stat_t mp_dwell(const float seconds);
void mp_end_dwell(void);
void mp_request_out_of_band_dwell(float seconds);
stat_t mp_exec_out_of_band_dwell(void);

// planner functions and helpers
uint8_t mp_get_planner_buffers(void);
bool mp_planner_is_full(void);
bool mp_has_runnable_buffer(void);
bool mp_is_phat_city_time(void);

stat_t mp_planner_callback();
void mp_replan_queue(mpBuf_t *bf);
void mp_start_feed_override(const float ramp_time, const float override);
void mp_end_feed_override(const float ramp_time);
void mp_start_traverse_override(const float ramp_time, const float override);
void mp_end_traverse_override(const float ramp_time);
void mp_planner_time_accounting(void);

// planner buffer primitives
void mp_init_buffers(void);

//mpBuf_t * mp_get_prev_buffer(const mpBuf_t *bf);      // Use the following macro instead
//mpBuf_t * mp_get_next_buffer(const mpBuf_t *bf);      // Use the following macro instead
#define mp_get_prev_buffer(b) ((mpBuf_t *)(b->pv))
#define mp_get_next_buffer(b) ((mpBuf_t *)(b->nx))

mpBuf_t * mp_get_write_buffer(void);
void mp_commit_write_buffer(const blockType block_type);
mpBuf_t * mp_get_run_buffer(void);
bool mp_free_run_buffer(void);

// plan_line.c functions
void mp_zero_segment_velocity(void);                    // getters and setters...
float mp_get_runtime_velocity(void);
float mp_get_runtime_absolute_position(uint8_t axis);
float mp_get_runtime_work_position(uint8_t axis);
void mp_set_runtime_work_offset(float offset[]);
bool mp_get_runtime_busy(void);
bool mp_runtime_is_idle(void);

stat_t mp_aline(GCodeState_t *gm_in);                   // line planning...
void mp_plan_block_list(void);
void mp_plan_block_forward(mpBuf_t *bf);

// plan_zoid.c functions
void mp_calculate_ramps(mpBlockRuntimeBuf_t *block, mpBuf_t *bf, const float entry_velocity);
float mp_get_target_length(const float v_0, const float v_1, const mpBuf_t *bf);
float mp_get_target_velocity(const float v_0, const float L, const mpBuf_t *bf); // acceleration ONLY
float mp_get_decel_velocity(const float v_0, const float L, const mpBuf_t *bf);  // deceleration ONLY
float mp_find_t(const float v_0, const float v_1, const float L, const float totalL, const float initial_t, const float T);

float mp_calc_v(const float t, const float v_0, const float v_1);                // compute the velocity along the curve accelerating from v_0 to v_1, at position t=[0,1]
float mp_calc_a(const float t, const float v_0, const float v_1, const float T); // compute acceleration over curve accelerating from v_0 to v_1, at position t=[0,1], total time T
float mp_calc_j(const float t, const float v_0, const float v_1, const float T); // compute jerk over curve accelerating from v_0 to v_1, at position t=[0,1], total time T
//float mp_calc_l(const float t, const float v_0, const float v_1, const float T); // compute length over curve accelerating from v_0 to v_1, at position t=[0,1], total time T

// plan_exec.c functions
stat_t mp_forward_plan(void);
stat_t mp_exec_move(void);
stat_t mp_exec_aline(mpBuf_t *bf);
void mp_exit_hold_state(void);

void mp_dump_planner(mpBuf_t *bf_start);

#endif    // End of include Guard: PLANNER_H_ONCE
