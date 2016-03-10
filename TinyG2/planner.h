/*
 * planner.h - cartesian trajectory planning and motion execution
 * This file is part of the TinyG project
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

#ifndef PLANNER_H_ONCE
#define PLANNER_H_ONCE

#include "canonical_machine.h"	// used for GCodeState_t

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

typedef enum {				        // bf->block_type values
    BLOCK_TYPE_NULL = 0,            // null move - does a no-op
    BLOCK_TYPE_ALINE,		        // acceleration planned line
    BLOCK_TYPE_DWELL,               // delay with no movement
    BLOCK_TYPE_COMMAND,             // general command
    BLOCK_TYPE_JSON_WAIT,           // general command
    BLOCK_TYPE_TOOL,                // T command
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

#define PLANNER_BUFFER_POOL_SIZE    ((uint8_t)48)       // Suggest 12 min. Limit is 255
#define PLANNER_BUFFER_HEADROOM     ((uint8_t)4)        // Buffers to reserve in planner before processing new input line
#define JERK_MULTIPLIER             ((float)1000000)    // DO NOT CHANGE - must always be 1 million

#define JUNCTION_AGGRESSION_MIN     (0.001)             // minimum allowable setting
#define JUNCTION_AGGRESSION_MAX     (10.00)             // maximum allowable setting

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
 *	Planner structures
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
    uint8_t meet_iterations;        // iterations needed in _get_meet_velocity
    //+++++ to here

    bufferState buffer_state;       // used to manage queuing/dequeuing
    blockType block_type;           // used to dispatch to run routine
    blockState block_state;         // move state machine sequence
    blockHint hint;                 // hint the block for zoid and other planning operations. Must be accurate or NO_HINT

    // block parameters
    float unit[AXES];               // unit vector for axis scaling & planning
    bool axis_flags[AXES];          // set true for axes participating in the move & for command parameters

    bool plannable;                 // set true when this block can be used for planning
    bool mergeable;                 // set true when this block can be merged into

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

    GCodeState_t gm;				// Gcode model state - passed from model, used by planner and runtime

    void clear() {
        memset((void *)(this), 0, sizeof(mpBuffer_to_clear));
    }
};

typedef struct mpBuffer : mpBuffer_to_clear { // See Planning Velocity Notes for variable usage

    // *** CAUTION *** These two pointers are not reset by _clear_buffer()
    struct mpBuffer *pv;            // static pointer to previous buffer
    struct mpBuffer *nx;            // static pointer to next buffer
    uint8_t buffer_number;          //+++++ DIAGNOSTICS for easier debugging
} mpBuf_t;

typedef struct mpBufferPool {		// ring buffer for sub-moves
	magic_t magic_start;			// magic number to test memory integrity

	mpBuf_t *r;						// run buffer pointer
	mpBuf_t *w;						// write buffer pointer
	uint8_t buffers_available;		// running count of available buffers
	mpBuf_t bf[PLANNER_BUFFER_POOL_SIZE];// buffer storage

	magic_t magic_end;
} mpBufferPool_t;

typedef struct mpMotionPlannerSingleton {  // common variables for planning (move master)
	magic_t magic_start;                // magic number to test memory integrity

    //+++++ DIAGNOSTICS
    float run_time_remaining_ms;
    float plannable_time_ms;

    // planner position
	float position[AXES];               // final move position for planning purposes

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

    // block merge controls
    bool block_merge_enable;
    float block_merge_ratio;
    float block_merge_velocity_max;
    float block_merge_length_max;
    float block_merge_cosine_min;

    // feed overrides and ramp variables (these extend the variables in cm.gmx)
    float mfo_factor;               // runtime override factor
    float ramp_target;
    float ramp_dvdt;

    // objects
    Timeout block_timeout;          // Timeout object used to time out blocks emerging form the annealer

    // planner pointers
	mpBuf_t *p;						// planner buffer pointer
	mpBuf_t *c;						// pointer to buffer immediately following critical region
	mpBuf_t *planning_return;       // buffer to return to once back-planning is complete

	magic_t magic_end;
} mpMotionPlannerSingleton_t;

typedef struct mpBlockRuntimeBuf {  // Data structure for just the parts of RunTime that we need to plan a BLOCK
    struct mpBlockRuntimeBuf *nx;       // singly-linked-list

    float head_length;                  // copies of bf variables of same name
    float body_length;
    float tail_length;

    float head_time;                    // copies of bf variables of same name
    float body_time;
    float tail_time;

    float cruise_velocity;              // velocity at the end of the head and the beginning of the tail
    float exit_velocity;                // velocity at the end of the move
} mpBlockRuntimeBuf_t;

typedef struct mpMotionRuntimeSingleton {	// persistent runtime variables
//	uint8_t (*run_move)(struct mpMoveRuntimeSingleton *m); // currently running move - left in for reference
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

    mpBlockRuntimeBuf_t *r;              // what's running
    mpBlockRuntimeBuf_t *p;              // what's being planned, p might == r
    mpBlockRuntimeBuf_t bf[2];           // the buffer

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
float mp_get_decel_velocity(const float v_0, const float L, const mpBuf_t *bf); // decelleration ONLY
float mp_find_t(const float v_0, const float v_1, const float L, const float totalL, const float initial_t, const float T);

float mp_calc_v(const float t, const float v_0, const float v_1);                // compute the velocity along the curve accelerating from v_0 to v_1, at position t=[0,1]
float mp_calc_a(const float t, const float v_0, const float v_1, const float T); // compute acceleration over curve accelerating from v_0 to v_1, at position t=[0,1], total time T
float mp_calc_j(const float t, const float v_0, const float v_1, const float T); // compute jerk over curve accelerating from v_0 to v_1, at position t=[0,1], total time T
//float mp_calc_l(const float t, const float v_0, const float v_1, const float T); // compute length over curve accelerating from v_0 to v_1, at position t=[0,1], total time T

// plan_exec.c functions
stat_t mp_exec_move(void);
stat_t mp_plan_move(void);
stat_t mp_exec_aline(mpBuf_t *bf);
void mp_exit_hold_state(void);

void mp_dump_planner(mpBuf_t *bf_start);

#endif	// End of include Guard: PLANNER_H_ONCE
