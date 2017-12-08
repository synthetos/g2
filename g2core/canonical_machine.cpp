/*
 * canonical_machine.cpp - rs274/ngc canonical machine.
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
/*
 *  This code is a loose implementation of Kramer, Proctor and Messina's canonical
 *  machining functions as described in the NIST RS274/NGC v3
 *
 *  The canonical machine is the layer between the Gcode parser and the motion control
 *  code for a specific robot. It keeps state and executes commands - passing the
 *  stateless commands to the motion planning layer.
 */
/* --- System state contexts - Gcode models ---
 *
 *  Useful reference for doing C callbacks http://www.newty.de/fpt/fpt.html
 *
 *  There are 3 temporal contexts for system state:
 *      - The gcode model in the canonical machine (the MODEL context, held in gm)
 *      - The gcode model used by the planner (PLANNER context, held in bf's and mm)
 *      - The gcode model used during motion for reporting (RUNTIME context, held in mr)
 *
 *  It's a bit more complicated than this. The 'gm' struct contains the core Gcode model
 *  context. This originates in the canonical machine and is copied to each planner buffer
 *  (bf buffer) during motion planning. Finally, the gm context is passed to the runtime
 *  (mr) for the RUNTIME context. So at last count the Gcode model exists in as many as
 *  30 copies in the system. (1+28+1)
 *
 *  Depending on the need, any one of these contexts may be called for reporting or by
 *  a function. Most typically, all new commends from the gcode parser work form the MODEL
 *  context, and status reports pull from the RUNTIME while in motion, and from MODEL when
 *  at rest. A convenience is provided in the ACTIVE_MODEL pointer to point to the right
 *  context.
 */
/* --- Synchronizing command execution ---
 *
 *  Some gcode commands only set the MODEL state for interpretation of the current Gcode
 *  block. For example, cm_set_feed_rate(). This sets the MODEL so the move time is
 *  properly calculated for the current (and subsequent) blocks, so it's effected
 *  immediately.
 *
 *  "Synchronous commands" are commands that affect the runtime need to be synchronized
 *  with movement. Examples include G4 dwells, program stops and ends, and most M commands.
 *  These are queued into the planner queue and execute from the queue. Synchronous commands
 *  work like this:
 *
 *    - Call the cm_xxx_xxx() function which will do any input validation and return an
 *      error if it detects one.
 *
 *    - The cm_ function calls mp_queue_command(). Arguments are a callback to the _exec_...()
 *      function, which is the runtime execution routine, and any arguments that are needed
 *      by the runtime. See typedef for *exec in planner.h for details
 *
 *    - mp_queue_command() stores the callback and the args in a planner buffer.
 *
 *    - When planner execution reaches the buffer it executes the callback w/ the args.
 *      Take careful note that the callback executes under an interrupt, so beware of
 *      variables that may need to be volatile.
 *
 *  Note:
 *    - The synchronous command execution mechanism uses 2 vectors in the bf buffer to store
 *      and return values for the callback. It's obvious, but impractical to pass the entire
 *      bf buffer to the callback as some of these commands are actually executed locally
 *      and have no buffer.
 */

#include "g2core.h"  // #1
#include "config.h"  // #2
#include "canonical_machine.h"
#include "hardware.h"
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "settings.h"

#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "spindle.h"
#include "coolant.h"
#include "pwm.h"
#include "report.h"
#include "gpio.h"
#include "temperature.h"
#include "hardware.h"
#include "util.h"
#include "settings.h"
#include "xio.h"            // for serial queue flush

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

cmSingleton_t cm;        // canonical machine controller singleton

/***********************************************************************************
 **** GENERIC STATIC FUNCTIONS AND VARIABLES ***************************************
 ***********************************************************************************/

// command execution callbacks from planner queue
static void _exec_offset(float *value, bool *flag);
static void _exec_change_tool(float *value, bool *flag);
static void _exec_select_tool(float *value, bool *flag);
static void _exec_absolute_origin(float *value, bool *flag);
static void _exec_program_finalize(float *value, bool *flag);

static int8_t _get_axis(const index_t index);

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/

/*************************************
 * Internal getters and setters      *
 * Canonical Machine State functions *
 *************************************/
/*
 * cm_set_motion_state() - adjusts active model pointer as well
 */
void cm_set_motion_state(const cmMotionState motion_state)
{
    cm.motion_state = motion_state;

    switch (motion_state) {
        case (MOTION_STOP):     { ACTIVE_MODEL = MODEL; break; }
        case (MOTION_PLANNING): { ACTIVE_MODEL = RUNTIME; break; }
        case (MOTION_RUN):      { ACTIVE_MODEL = RUNTIME; break; }
        case (MOTION_HOLD):     { ACTIVE_MODEL = RUNTIME; break; }
    }
}

/*
 * cm_get_machine_state()
 * cm_get_motion_state()
 * cm_get_cycle_state()
 * cm_get_hold_state()
 * cm_get_homing_state()
 */
cmMachineState  cm_get_machine_state() { return cm.machine_state;}
cmCycleState    cm_get_cycle_state()   { return cm.cycle_state;}
cmMotionState   cm_get_motion_state()  { return cm.motion_state;}
cmFeedholdState cm_get_hold_state()    { return cm.hold_state;}
cmHomingState   cm_get_homing_state()  { return cm.homing_state;}

/*
 * cm_get_combined_state() - combines raw states into something a user might want to see
 *
 *  Note:
 *  On issuing a gcode command we call cm_cycle_start() before the motion gets queued. We don't go
 *  to MOTION_RUN until the command is executed by mp_exec_aline(), planned, queued, and started.
 *  So MOTION_STOP must actually return COMBINED_RUN to address this case, even though under some
 *  circumstances it might actually ne an exception case. Therefore this assertion isn't valid:
 *      cm_panic(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, "mots2"));//"mots is stop but machine is in cycle"
 *      return (COMBINED_PANIC);
 */
cmCombinedState cm_get_combined_state()
{
    if (cm.machine_state <= MACHINE_PROGRAM_END) {  // replaces first 5 cm.machine_state cases
        return ((cmCombinedState)cm.machine_state); //...where MACHINE_xxx == COMBINED_xxx
    }
    switch(cm.machine_state) {
        case MACHINE_INTERLOCK:     { return (COMBINED_INTERLOCK); }
        case MACHINE_SHUTDOWN:      { return (COMBINED_SHUTDOWN); }
        case MACHINE_PANIC:         { return (COMBINED_PANIC); }
        case MACHINE_CYCLE: {
            switch(cm.cycle_state) {
                case CYCLE_HOMING:  { return (COMBINED_HOMING); }
                case CYCLE_PROBE:   { return (COMBINED_PROBE); }
                case CYCLE_JOG:     { return (COMBINED_JOG); }
                case CYCLE_MACHINING: case CYCLE_OFF: {
                    switch(cm.motion_state) {
                        case MOTION_STOP:     { return (COMBINED_RUN); }    // See NOTE_1, above
                        case MOTION_PLANNING: { return (COMBINED_RUN); }
                        case MOTION_RUN:      { return (COMBINED_RUN); }
                        case MOTION_HOLD:     { return (COMBINED_HOLD); }
                        default: {
                            cm_panic(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, "cm_get_combined_state() mots bad");// "mots has impossible value"
                            return (COMBINED_PANIC);
                        }
                    }
                }
                default: {
                    cm_panic(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, "cm_get_combined_state() cycs bad");    // "cycs has impossible value"
                    return (COMBINED_PANIC);
                }
            }
        }
        default: {
            cm_panic(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, "cm_get_combined_state() macs bad");    // "macs has impossible value"
            return (COMBINED_PANIC);
        }
    }
}

/***********************************
 * Model State Getters and Setters *
 ***********************************/

/*  These getters and setters will work on any gm model with inputs:
 *    MODEL         (GCodeState_t *)&cm.gm          // absolute pointer from canonical machine gm model
 *    PLANNER       (GCodeState_t *)&bf->gm         // relative to buffer *bf is currently pointing to
 *    RUNTIME       (GCodeState_t *)&mr.gm          // absolute pointer from runtime mm struct
 *    ACTIVE_MODEL   cm.am                          // active model pointer is maintained by state management
 */
uint32_t cm_get_linenum(const GCodeState_t *gcode_state) { return gcode_state->linenum;}
cmMotionMode cm_get_motion_mode(const GCodeState_t *gcode_state) { return gcode_state->motion_mode;}
uint8_t cm_get_coord_system(const GCodeState_t *gcode_state) { return gcode_state->coord_system;}
uint8_t cm_get_units_mode(const GCodeState_t *gcode_state) { return gcode_state->units_mode;}
uint8_t cm_get_select_plane(const GCodeState_t *gcode_state) { return gcode_state->select_plane;}
uint8_t cm_get_path_control(const GCodeState_t *gcode_state) { return gcode_state->path_control;}
uint8_t cm_get_distance_mode(const GCodeState_t *gcode_state) { return gcode_state->distance_mode;}
uint8_t cm_get_arc_distance_mode(const GCodeState_t *gcode_state) { return gcode_state->arc_distance_mode;}
uint8_t cm_get_feed_rate_mode(const GCodeState_t *gcode_state) { return gcode_state->feed_rate_mode;}
uint8_t cm_get_tool(const GCodeState_t *gcode_state) { return gcode_state->tool;}
uint8_t cm_get_block_delete_switch() { return cm.gmx.block_delete_switch;}
uint8_t cm_get_runtime_busy() { return (mp_get_runtime_busy());}
float cm_get_feed_rate(const GCodeState_t *gcode_state) { return gcode_state->feed_rate;}

void cm_set_motion_mode(GCodeState_t *gcode_state, const uint8_t motion_mode)
{
    gcode_state->motion_mode = (cmMotionMode)motion_mode;
}

void cm_set_tool_number(GCodeState_t *gcode_state, const uint8_t tool)
{
    gcode_state->tool = tool;
}

void cm_set_absolute_override(GCodeState_t *gcode_state, const uint8_t absolute_override)
{
    gcode_state->absolute_override = (cmAbsoluteOverride)absolute_override;
    cm_set_work_offsets(MODEL);         // must reset offsets if you change absolute override
}

void cm_set_model_linenum(const uint32_t linenum)
{
    cm.gm.linenum = linenum;            // you must first set the model line number,
    nv_add_object((const char *)"n");   // then add the line number to the nv list
}

stat_t cm_check_linenum() {
    if (cm.gmx.last_line_number != cm.gm.linenum) {
        _debug_trap("line number out of sequence");
        return STAT_LINE_NUMBER_OUT_OF_SEQUENCE;
    }
    cm.gmx.last_line_number = cm.gm.linenum;
    return STAT_OK;
}

/***********************************************************************************
 * COORDINATE SYSTEMS AND OFFSETS
 * Functions to get, set and report coordinate systems and work offsets
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/
/*
 * Notes on Coordinate System and Offset functions
 *
 * All positional information in the canonical machine is kept as absolute coords and in
 *    canonical units (mm). The offsets are only used to translate in and out of canonical form
 *    during interpretation and response.
 *
 * Managing the coordinate systems & offsets is somewhat complicated. The following affect offsets:
 *    - coordinate system selected. 1-9 correspond to G54-G59
 *    - absolute override: forces current move to be interpreted in machine coordinates: G53 (system 0)
 *    - G92 offsets are added "on top of" the coord system offsets -- if origin_offset_enable == true
 *    - G28 and G30 moves; these are run in absolute coordinates
 *
 * The offsets themselves are considered static, are kept in cm, and are supposed to be persistent.
 *
 * To reduce complexity and data load the following is done:
 *    - Full data for coordinates/offsets is only accessible by the canonical machine, not the downstream
 *    - A fully resolved set of coord and G92 offsets, with per-move exceptions can be captured as "work_offsets"
 *    - The core gcode context (gm) only knows about the active coord system and the work offsets
 */

/*
 * cm_get_active_coord_offset() - return the currently active coordinate offset for an axis
 *
 *    Takes G5x, G92 and absolute override into account to return the active offset for this move
 *
 *    This function is typically used to evaluate and set offsets, as opposed to cm_get_work_offset()
 *    which merely returns what's in the work_offset[] array.
 */

float cm_get_active_coord_offset(const uint8_t axis)
{
    if (cm.gm.absolute_override == ABSOLUTE_OVERRIDE_ON) {  // no offset if in absolute override mode
        return (0.0);
    }
    float offset = cm.offset[cm.gm.coord_system][axis] + cm.tl_offset[axis];
    if (cm.gmx.origin_offset_enable == true) {
        offset += cm.gmx.origin_offset[axis];               // includes G5x and G92 components
    }
    return (offset);
}

/*
 * cm_get_work_offset() - return a coord offset from the gcode_state
 *
 *    MODEL         (GCodeState_t *)&cm.gm          // absolute pointer from canonical machine gm model
 *    PLANNER       (GCodeState_t *)&bf->gm         // relative to buffer *bf is currently pointing to
 *    RUNTIME       (GCodeState_t *)&mr.gm          // absolute pointer from runtime mm struct
 *    ACTIVE_MODEL   cm.am                          // active model pointer is maintained by state management
 */

float cm_get_work_offset(const GCodeState_t *gcode_state, const uint8_t axis)
{
    return (gcode_state->work_offset[axis]);
}

/*
 * cm_set_work_offsets() - capture coord offsets from the model into absolute values in the gcode_state
 *
 *    MODEL         (GCodeState_t *)&cm.gm          // absolute pointer from canonical machine gm model
 *    PLANNER       (GCodeState_t *)&bf->gm         // relative to buffer *bf is currently pointing to
 *    RUNTIME       (GCodeState_t *)&mr.gm          // absolute pointer from runtime mm struct
 *    ACTIVE_MODEL   cm.am                          // active model pointer is maintained by state management
 */

void cm_set_work_offsets(GCodeState_t *gcode_state)
{
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        gcode_state->work_offset[axis] = cm_get_active_coord_offset(axis);
    }
}

/*
 * cm_get_absolute_position() - get position of axis in absolute coordinates
 *
 *  This function accepts as input:
 *    MODEL         (GCodeState_t *)&cm.gm          // absolute pointer from canonical machine gm model
 *    RUNTIME       (GCodeState_t *)&mr.gm          // absolute pointer from runtime mm struct
 *
 *    NOTE: Only MODEL and RUNTIME are supported (no PLANNER or bf's)
 *    NOTE: Machine position is always returned in mm mode. No units conversion is performed
 */

float cm_get_absolute_position(const GCodeState_t *gcode_state, const uint8_t axis)
{
    if (gcode_state == MODEL) {
        return (cm.gmx.position[axis]);
    }
    return (mp_get_runtime_absolute_position(axis));
}

/*
 * cm_get_work_position() - return work position in external form
 *
 *    ... that means in prevailing units (mm/inch) and with all offsets applied
 *
 * NOTE: This function only works after the gcode_state struct as had the work_offsets setup by
 *       calling cm_get_model_coord_offset_vector() first.
 *
 *  This function accepts as input:
 *    MODEL         (GCodeState_t *)&cm.gm          // absolute pointer from canonical machine gm model
 *    RUNTIME       (GCodeState_t *)&mr.gm          // absolute pointer from runtime mm struct
 *
 * NOTE: Only MODEL and RUNTIME are supported (no PLANNER or bf's)
 */

float cm_get_work_position(const GCodeState_t *gcode_state, const uint8_t axis)
{
    float position;

    if (gcode_state == MODEL) {
        position = cm.gmx.position[axis] - cm_get_active_coord_offset(axis);
    } else {
        position = mp_get_runtime_work_position(axis);
    }
    if (axis <= AXIS_Z) {
        if (gcode_state->units_mode == INCHES) {
            position /= MM_PER_INCH;
        }
    }
    return (position);
}

/***********************************************************************************
 * CRITICAL HELPERS
 * Core functions supporting the canonical machining functions
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/
/*
 * cm_finalize_move() - perform final operations for a traverse or feed
 * cm_update_model_position_from_runtime() - set endpoint position from final runtime position
 *
 *  These routines set the point position in the gcode model.
 *
 *  Note: As far as the canonical machine is concerned the final position of a Gcode block (move)
 *    is achieved as soon as the move is planned and the move target becomes the new model position.
 *    In reality the planner will (in all likelihood) have only just queued the move for later
 *    execution, and the real tool position is still close to the starting point.
 */

void cm_finalize_move()
{
    copy_vector(cm.gmx.position, cm.gm.target);     // update model position
}

void cm_update_model_position_from_runtime()
{
    copy_vector(cm.gmx.position, mr.gm.target);
}

/*
 * cm_deferred_write_callback() - write any changed G10 values back to persistence
 *
 *  Only runs if there is G10 data to write, there is no movement, and the serial queues are quiescent
 *  This could be made tighter by issuing an XOFF or ~CTS beforehand and releasing it afterwards.
 */

stat_t cm_deferred_write_callback()
{
    if ((cm.cycle_state == CYCLE_OFF) && (cm.deferred_write_flag == true)) {
        cm.deferred_write_flag = false;
        nvObj_t nv;
        for (uint8_t i=1; i<=COORDS; i++) {
            for (uint8_t j=0; j<AXES; j++) {
                sprintf((char *)nv.token, "g%2d%c", 53+i, ("xyzabc")[j]);
                nv.index = nv_get_index((const char *)"", nv.token);
                nv.value = cm.offset[i][j];
                nv_persist(&nv);                // Note: only writes values that have changed
            }
        }
    }
    return (STAT_OK);
}

/*
 * cm_set_tram() - JSON command to trigger computing the rotation matrix
 * cm_get_tram() - JSON query to determine if the rotation matrix is set (non-identity)
 *
 * There MUST be three valid probes stored.
 */

stat_t cm_set_tram(nvObj_t *nv)
{
    if ((nv->valuetype == TYPE_BOOL) ||
        (nv->valuetype == TYPE_INT) ||
        (nv->valuetype == TYPE_FLOAT))
    {
        bool do_set = !!((uint32_t)nv->value);

        // if passed false/0, we will clear the rotation matrix
        if (!do_set) {
            canonical_machine_reset_rotation();
            return (STAT_OK);
        }

        // check to make sure we have three valid probes in a row
        if ((cm.probe_state[0] == PROBE_SUCCEEDED) &&
            (cm.probe_state[1] == PROBE_SUCCEEDED) &&
            (cm.probe_state[2] == PROBE_SUCCEEDED))
        {

            // Step 1: Get the normal of the plane formed by the three probes. Naming:
            //    d0_{xyz} is the delta between point 0 and point 1
            //    d2_{xyz} is the delta between point 2 and point 1
            //    n_{xyz} is the unit normal

            // Step 1a: get the deltas
            float d0_x = cm.probe_results[0][0] - cm.probe_results[1][0];
            float d0_y = cm.probe_results[0][1] - cm.probe_results[1][1];
            float d0_z = cm.probe_results[0][2] - cm.probe_results[1][2];
            float d2_x = cm.probe_results[2][0] - cm.probe_results[1][0];
            float d2_y = cm.probe_results[2][1] - cm.probe_results[1][1];
            float d2_z = cm.probe_results[2][2] - cm.probe_results[1][2];

            // Step 1b: compute the combined magnitude
            // since sqrt(a)*sqrt(b) = sqrt(a*b), we can save a sqrt in making the unit normal
            float combined_magnitude_inv = 1.0/sqrt(
                                            (d0_x*d0_x + d0_y*d0_y + d0_z*d0_z)*
                                            (d2_x*d2_x + d2_y*d2_y + d2_z*d2_z)
                                            );

            // Step 1c: compute the cross product and normalize
            float n_x = (d0_z*d2_y - d0_y*d2_z)*combined_magnitude_inv;
            float n_y = (d0_x*d2_z - d0_z*d2_x)*combined_magnitude_inv;
            float n_z = (d0_y*d2_x - d0_x*d2_y)*combined_magnitude_inv;

            // Step 1d: flip the normal if it's negative
            if (n_z < 0.0) {
                n_x = -n_x;
                n_y = -n_y;
                n_z = -n_z;
            }

            // Step 2: make the quaternion for the rotation to {0,0,1}
            float p = sqrt(n_x*n_x + n_y*n_y + n_z*n_z);
            float m = sqrt(2.0)*sqrt(p*(p+n_z));
            float q_w = (n_z + p) / m;
            float q_x = -n_y / m;
            float q_y = n_x / m;
            //float q_z = 0; // already optimized out

            // Step 3: compute the rotation matrix
            float q_wx_2 = q_w * q_x * 2.0;
            float q_wy_2 = q_w * q_y * 2.0;
            float q_xx_2 = q_x * q_x * 2.0;
            float q_xy_2 = q_x * q_y * 2.0;
            float q_yy_2 = q_y * q_y * 2.0;

            /*
             matrix = {
                {1 - q_yy_2, q_xy_2,      q_wy_2,             0},
                {q_xy_2,     1 - q_xx_2, -q_wx_2,             0},
                {-q_wy_2,    q_wx_2,     1 - q_xx_2 - q_yy_2, i},
                {0,          0,          0,                   1}
             }
            */
            cm.rotation_matrix[0][0] = 1 - q_yy_2;
            cm.rotation_matrix[0][1] = q_xy_2;
            cm.rotation_matrix[0][2] = q_wy_2;

            cm.rotation_matrix[1][0] = q_xy_2;
            cm.rotation_matrix[1][1] = 1 - q_xx_2;
            cm.rotation_matrix[1][2] = -q_wx_2;

            cm.rotation_matrix[2][0] = -q_wy_2;
            cm.rotation_matrix[2][1] = q_wx_2;
            cm.rotation_matrix[2][2] = 1 - q_xx_2 - q_yy_2;

            // Step 4: compute the z-offset
            cm.rotation_z_offset = (n_x*cm.probe_results[1][0] + n_y*cm.probe_results[1][1]) / n_z + cm.probe_results[1][2];
        } else {
            return (STAT_COMMAND_NOT_ACCEPTED);
        }
    } else {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
    return (STAT_OK);
}

stat_t cm_get_tram(nvObj_t *nv)
{
    nv->value = true;

    if (fp_NOT_ZERO(cm.rotation_z_offset) ||
        fp_NOT_ZERO(cm.rotation_matrix[0][1]) ||
        fp_NOT_ZERO(cm.rotation_matrix[0][2]) ||
        fp_NOT_ZERO(cm.rotation_matrix[1][0]) ||
        fp_NOT_ZERO(cm.rotation_matrix[1][2]) ||
        fp_NOT_ZERO(cm.rotation_matrix[2][0]) ||
        fp_NOT_ZERO(cm.rotation_matrix[2][1]) ||
        fp_NE(1.0,  cm.rotation_matrix[0][0]) ||
        fp_NE(1.0,  cm.rotation_matrix[1][1]) ||
        fp_NE(1.0,  cm.rotation_matrix[2][2]))
    {
        nv->value = false;
    }
    nv->valuetype = TYPE_BOOL;
    return (STAT_OK);
}


/*
 * cm_set_nxt_line() - JSON command to set the next line number
 * cm_get_nxt_line() - JSON query to get the next expected line number
 *
 * There MUST be three valid probes stored.
 */

stat_t cm_set_nxln(nvObj_t *nv)
{
    if (nv->valuetype == TYPE_INT || nv->valuetype == TYPE_FLOAT)
    {
        cm.gmx.last_line_number = ((int32_t)nv->value) - 1;
        return (STAT_OK);
    }
    return (STAT_INPUT_VALUE_RANGE_ERROR);
}

stat_t cm_get_nxln(nvObj_t *nv)
{
    nv->value = cm.gmx.last_line_number+1;
    nv->valuetype = TYPE_INT;
    return (STAT_OK);
}


/*
 * cm_set_model_target() - set target vector in GM model
 *
 * This is a core routine. It handles:
 *    - conversion of linear units to internal canonical form (mm)
 *    - conversion of relative mode to absolute (internal canonical form)
 *    - translation of work coordinates to machine coordinates (internal canonical form)
 *    - computation and application of axis modes as so:
 *
 *    DISABLED  - Incoming value is ignored. Target value is not changed
 *    ENABLED   - Convert axis values to canonical format and store as target
 *    INHIBITED - Same processing as ENABLED, but axis will not actually be run
 *    RADIUS    - ABC axis value is provided in Gcode block in linear units
 *              - Target is set to degrees based on axis' Radius value
 *              - Radius mode is only processed for ABC axes. Application to XYZ is ignored.
 *
 *  Target coordinates are provided in target[]
 *  Axes that need processing are signaled in flag[]
 */

// ESTEE: _calc_ABC is a fix to workaround a gcc compiler bug wherein it runs out of spill
//        registers we moved this block into its own function so that we get a fresh stack push
// ALDEN: This shows up in avr-gcc 4.7.0 and avr-libc 1.8.0

static float _calc_ABC(const uint8_t axis, const float target[])
{
    if ((cm.a[axis].axis_mode == AXIS_STANDARD) || (cm.a[axis].axis_mode == AXIS_INHIBITED)) {
        return(target[axis]);    // no mm conversion - it's in degrees
    }

    // radius mode

    return (_to_millimeters(target[axis]) * 360.0 / (2.0 * M_PI * cm.a[axis].radius));
}

void cm_set_model_target(const float target[], const bool flags[])
{
    uint8_t axis;
    float tmp = 0;

    // copy position to target so it always starts correctly
    copy_vector(cm.gm.target, cm.gmx.position);

    // process XYZABC for lower modes
    for (axis=AXIS_X; axis<=AXIS_Z; axis++) {
        if (!flags[axis] || cm.a[axis].axis_mode == AXIS_DISABLED) {
            continue;        // skip axis if not flagged for update or its disabled
        } else if ((cm.a[axis].axis_mode == AXIS_STANDARD) || (cm.a[axis].axis_mode == AXIS_INHIBITED)) {
            if (cm.gm.distance_mode == ABSOLUTE_DISTANCE_MODE) {
                cm.gm.target[axis] = cm_get_active_coord_offset(axis) + _to_millimeters(target[axis]);
            } else {
                cm.gm.target[axis] += _to_millimeters(target[axis]);
            }
        }
    }
    // FYI: The ABC loop below relies on the XYZ loop having been run first
    for (axis=AXIS_A; axis<=AXIS_C; axis++) {
        if (!flags[axis] || cm.a[axis].axis_mode == AXIS_DISABLED) {
            continue;        // skip axis if not flagged for update or its disabled
        } else {
            tmp = _calc_ABC(axis, target);
        }
#if MARLIN_COMPAT_ENABLED == true
        // If we are in absolute mode (generally), but the extruder is relative,
        // then we adjust the extruder to a relative position
        if (mst.marlin_flavor && (cm.a[axis].axis_mode == AXIS_RADIUS)) {
            if ((cm.gm.distance_mode == INCREMENTAL_DISTANCE_MODE) || (mst.extruder_mode == EXTRUDER_MOVES_RELATIVE)) {
                cm.gm.target[axis] += tmp;
            }
            else { // if (cm.gmx.extruder_mode == EXTRUDER_MOVES_NORMAL)
                cm.gm.target[axis] = tmp + cm_get_active_coord_offset(axis);
            }
            // TODO
//            else {
//                cm.gm.target[axis] += tmp * cm.gmx.volume_to_filament_length[axis-3];
//            }
        }
        else
#endif // MARLIN_COMPAT_ENABLED
        if (cm.gm.distance_mode == ABSOLUTE_DISTANCE_MODE) {
            cm.gm.target[axis] = tmp + cm_get_active_coord_offset(axis); // sacidu93's fix to Issue #22
        }
        else {
            cm.gm.target[axis] += tmp;
        }
    }
}

/*
 * cm_get_soft_limits()
 * cm_set_soft_limits()
 * cm_test_soft_limits() - return error code if soft limit is exceeded
 *
 *  The target[] arg must be in absolute machine coordinates. Best done after cm_set_model_target().
 *
 *  Tests for soft limit for any homed axis if min and max are different values. You can set min
 *  and max to the same value (e.g. 0,0) to disable soft limits for an axis. Also will not test
 *  a min or a max if the value is more than +/- 1000000 (plus or minus 1 million ).
 *  This allows a single end to be tested w/the other disabled, should that requirement ever arise.
 */

bool cm_get_soft_limits() { return (cm.soft_limit_enable); }
void cm_set_soft_limits(bool enable) { cm.soft_limit_enable = enable; }

static stat_t _finalize_soft_limits(const stat_t status)
{
    cm.gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;     // cancel motion
    copy_vector(cm.gm.target, cm.gmx.position);             // reset model target
    return (cm_alarm(status, "soft_limits"));               // throw an alarm
}

stat_t cm_test_soft_limits(const float target[])
{
    if (cm.soft_limit_enable == true) {
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            if (cm.homed[axis] != true) { continue; }                               // skip axis if not homed
            if (fp_EQ(cm.a[axis].travel_min, cm.a[axis].travel_max)) { continue; }  // skip axis if identical
            if (std::abs(cm.a[axis].travel_min) > DISABLE_SOFT_LIMIT) { continue; }     // skip min test if disabled
            if (std::abs(cm.a[axis].travel_max) > DISABLE_SOFT_LIMIT) { continue; }     // skip max test if disabled

            if (target[axis] < cm.a[axis].travel_min) {
                return (_finalize_soft_limits(STAT_SOFT_LIMIT_EXCEEDED_XMIN + 2*axis));
            }
            if (target[axis] > cm.a[axis].travel_max) {
                return (_finalize_soft_limits(STAT_SOFT_LIMIT_EXCEEDED_XMAX + 2*axis));
            }
        }
    }
    return (STAT_OK);
}

/*************************************************************************
 * CANONICAL MACHINING FUNCTIONS
 *  Values are passed in pre-unit_converted state (from gn structure)
 *  All operations occur on gm (current model state)
 *
 *  These are organized by section number (x.x.x) in the order they are
 *  found in NIST RS274 NGCv3
 ************************************************************************/

/******************************************
 * Initialization and Termination (4.3.2) *
 ******************************************/
/*
 * canonical_machine_init()  - initialize cm struct
 * canonical_machine_reset() - apply startup settings or reset to startup
 *                             run profile initialization beforehand
 */

void canonical_machine_init()
{
// If you can assume all memory has been zeroed by a hard reset you don't need this code:
//    memset(&cm, 0, sizeof(cm));                 // do not reset canonicalMachineSingleton once it's been initialized
    memset(&cm, 0, sizeof(cmSingleton_t));      // do not reset canonicalMachineSingleton once it's been initialized
    cm.gm.reset();                              // clear all values, pointers and status -- not ALL to zero, however

    canonical_machine_init_assertions();        // establish assertions
    ACTIVE_MODEL = MODEL;                       // setup initial Gcode model pointer
    cm_arc_init();                              // Note: spindle and coolant inits are independent
}

void canonical_machine_reset_rotation() {
    memset(&cm.rotation_matrix, 0, sizeof(float)*3*3);
    // We must make it an identity matrix for no rotation
    cm.rotation_matrix[0][0] = 1.0;
    cm.rotation_matrix[1][1] = 1.0;
    cm.rotation_matrix[2][2] = 1.0;
    cm.rotation_z_offset = 0.0;
}

void canonical_machine_reset()
{
    // set gcode defaults
    cm_set_units_mode(cm.default_units_mode);
    cm_set_coord_system(cm.default_coord_system);   // NB: queues a block to the planner with the coordinates
    cm_select_plane(cm.default_select_plane);
    cm_set_path_control(MODEL, cm.default_path_control);
    cm_set_distance_mode(cm.default_distance_mode);
    cm_set_arc_distance_mode(INCREMENTAL_DISTANCE_MODE);// always the default
    cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);       // always the default
    cm_reset_overrides();                               // set overrides to initial conditions

    // NOTE: Should unhome axes here

    // reset requests and flags 
    cm.queue_flush_state = FLUSH_OFF;
    cm.end_hold_requested = false;
    cm.limit_requested = 0;                     // resets switch closures that occurred during initialization
    cm.safety_interlock_disengaged = 0;         // ditto
    cm.safety_interlock_reengaged = 0;          // ditto
    cm.shutdown_requested = 0;                  // ditto
    cm.probe_report_enable = PROBE_REPORT_ENABLE;

    // set initial state and signal that the machine is ready for action
    cm.cycle_state = CYCLE_OFF;
    cm.motion_state = MOTION_STOP;
    cm.hold_state = FEEDHOLD_OFF;
    cm.esc_boot_timer = SysTickTimer.getValue();
    cm.gmx.block_delete_switch = true;
    cm.gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE; // never start in a motion mode
    cm.machine_state = MACHINE_READY;

    canonical_machine_reset_rotation();

    memset(&cm.probe_state, 0, sizeof(cmProbeState)*PROBES_STORED);
    memset(&cm.probe_results, 0, sizeof(float)*PROBES_STORED*AXES);
}

/*
 * canonical_machine_init_assertions()
 * canonical_machine_test_assertions() - test assertions, return error code if violation exists
 */

void canonical_machine_init_assertions(void)
{
    cm.magic_start = MAGICNUM;
    cm.magic_end = MAGICNUM;
    cm.gmx.magic_start = MAGICNUM;
    cm.gmx.magic_end = MAGICNUM;
    arc.magic_start = MAGICNUM;
    arc.magic_end = MAGICNUM;
}

stat_t canonical_machine_test_assertions(void)
{
    if ((BAD_MAGIC(cm.magic_start)) || (BAD_MAGIC(cm.magic_end)) ||
        (BAD_MAGIC(cm.gmx.magic_start)) || (BAD_MAGIC(cm.gmx.magic_end)) ||
        (BAD_MAGIC(arc.magic_start)) || (BAD_MAGIC(arc.magic_end))) {
        return(cm_panic(STAT_CANONICAL_MACHINE_ASSERTION_FAILURE, "canonical_machine_test_assertions()"));
    }
    return (STAT_OK);
}

/**************************
 * Alarms                 *
 **************************/

/********************************************************************************
 * ALARM, SHUTDOWN, and PANIC are nested dolls.
 *
 * cm_alrm()  - invoke alarm from command
 * cm_shutd() - invoke shutdown from command
 * cm_pnic()  - invoke panic from command
 * cm_clr()   - clear alarm or shutdown from command
 *
 * The alarm states can be invoked from the above commands for testing and clearing
 */
stat_t cm_alrm(nvObj_t *nv)               // invoke alarm from command
{
    cm_alarm(STAT_ALARM, "sent by host");
    return (STAT_OK);
}

stat_t cm_shutd(nvObj_t *nv)              // invoke shutdown from command
{
    cm_shutdown(STAT_SHUTDOWN, "sent by host");
    return (STAT_OK);
}

stat_t cm_pnic(nvObj_t *nv)               // invoke panic from command
{
    cm_panic(STAT_PANIC, "sent by host");
    return (STAT_OK);
}

stat_t cm_clr(nvObj_t *nv)                // clear alarm or shutdown from command line
{
    cm_clear();
    return (STAT_OK);
}

/*
 * cm_clear() - clear ALARM and SHUTDOWN states
 * cm_parse_clear() - parse incoming gcode for M30 or M2 clears if in ALARM state
 *
 * Parse clear interprets an M30 or M2 PROGRAM_END as a $clear condition and clear ALARM
 * but not SHUTDOWN or PANIC. Assumes Gcode string has no leading or embedded whitespace
 */

void cm_clear()
{
    if (cm.machine_state == MACHINE_ALARM) {
        cm.machine_state = MACHINE_PROGRAM_STOP;
        xio_flush_to_command();
    } else if (cm.machine_state == MACHINE_SHUTDOWN) {
        cm.machine_state = MACHINE_READY;
    }
}

void cm_parse_clear(const char *s)
{
    if (cm.machine_state == MACHINE_ALARM) {
        if (toupper(s[0]) == 'M') {
            if (( (s[1]=='3') && (s[2]=='0') && (s[3]==NUL)) || ((s[1]=='2') && (s[2]==NUL) )) {
                cm_clear();
            }
        }
    }
}

/*
 * cm_is_alarmed() - return alarm status code or OK if no alarms
 */

stat_t cm_is_alarmed()
{
    if (cm.machine_state == MACHINE_ALARM)    { return (STAT_COMMAND_REJECTED_BY_ALARM); }
    if (cm.machine_state == MACHINE_SHUTDOWN) { return (STAT_COMMAND_REJECTED_BY_SHUTDOWN); }
    if (cm.machine_state == MACHINE_PANIC)    { return (STAT_COMMAND_REJECTED_BY_PANIC); }
    return (STAT_OK);
}

/*
 * cm_halt_all() - stop, spindle and coolant immediately
 * cm_halt_motion() - stop motion immediately. Does not affect spindle, coolant, or other IO
 *
 * Stop motors and reset all system states accordingly.
 * Does not de-energize motors as in some cases the motors must remain energized
 * in order to prevent an axis from crashing.
 */

void cm_halt_all(void)
{
    cm_halt_motion();
    cm_spindle_off_immediate();
    cm_coolant_off_immediate();
}

void cm_halt_motion(void)
{
    mp_halt_runtime();                  // stop the runtime. Do this immediately. (Reset is in cm_clear)
    canonical_machine_reset();          // reset Gcode model
    cm.cycle_state = CYCLE_OFF;         // Note: leaves machine_state alone
    cm.motion_state = MOTION_STOP;
    cm.hold_state = FEEDHOLD_OFF;
}

/*
 * cm_alarm() - enter ALARM state
 *
 * An ALARM sets the ALARM machine state, starts a feedhold to stop motion, stops the
 * spindle, turns off coolant, clears out queued planner moves and serial input,
 * and rejects new action commands (gcode blocks, SET commands, and other actions)
 * until the alarm is cleared.
 *
 * ALARM is typically entered by a soft limit or a limit switch being hit. In the
 * limit switch case the INPUT_ACTION will override the feedhold - i.e. if the
 * input action is "FAST_STOP" or "HALT" that setting will take precedence over
 * the feedhold native to the alarm function.
 *
 * Gcode and machine state is preserved. It may be possible to recover the job from
 * an alarm, but in many cases this is not possible. Since ALARM attempts to preserve
 * Gcode and machine state it does not END the job.
 *
 * ALARM may also be invoked from the command line using {alarm:n} or $alarm
 * ALARM can be manually cleared by entering: {clear:n}, {clr:n}, $clear, or $clr
 * ALARMs will also clear on receipt of an M30 or M2 command if one is received
 * while draining the host command queue.
 */

stat_t cm_alarm(const stat_t status, const char *msg)
{
    if ((cm.machine_state == MACHINE_ALARM) || (cm.machine_state == MACHINE_SHUTDOWN) ||
        (cm.machine_state == MACHINE_PANIC)) {
        return (STAT_OK);                       // don't alarm if already in an alarm state
    }
    cm.machine_state = MACHINE_ALARM;
    cm_request_feedhold();                      // stop motion
    cm_request_queue_flush();                   // do a queue flush once runtime is not busy

//  TBD - these functions should probably be called - See cm_shutdown()
//  cm_spindle_control_immediate(SPINDLE_OFF);
//  cm_coolant_off_immediate();
//  cm_spindle_optional_pause(spindle.pause_on_hold);
//  cm_coolant_optional_pause(coolant.pause_on_hold);
    rpt_exception(status, msg);                    // send alarm message

    // If "stat" is in the status report, we need to poke it to send.
    sr_request_status_report(SR_REQUEST_TIMED);
    return (status);
}
/*
 * cm_shutdown() - enter shutdown state
 *
 * SHUTDOWN stops all motion, spindle and coolant immediately, sets a SHUTDOWN machine
 * state, clears out queued moves and serial input, and rejects new action commands
 * (gcode blocks, SET commands, and some others).
 *
 * Shutdown is typically invoked as an electrical input signal sent to the board as
 * part of an external emergency stop (Estop). Shutdown is meant to augment but not
 * replace the external Estop functions that shut down power to motors, spindles and
 * other moving parts.
 *
 * Shutdown may also be invoked from the command line using {shutd:n} or $shutd
 * Shutdown must be manually cleared by entering: {clear:n}, {clr:n}, $clear, or $clr
 * Shutdown does not clear on M30 or M2 Gcode commands
 */

stat_t cm_shutdown(const stat_t status, const char *msg)
{
    if ((cm.machine_state == MACHINE_SHUTDOWN) || (cm.machine_state == MACHINE_PANIC)) {
        return (STAT_OK);                       // don't shutdown if shutdown or panic'd
    }
    cm_halt_motion();                           // halt motors (may have already been done from GPIO)
    spindle_reset();                            // stop spindle immediately and set speed to 0 RPM
    coolant_reset();                            // stop coolant immediately
    temperature_reset();                        // turn off heaters and fans
    cm_queue_flush();                           // flush all queues and reset positions

    for (uint8_t i = 0; i < HOMING_AXES; i++) { // unhome axes and the machine
        cm.homed[i] = false;
    }
    cm.homing_state = HOMING_NOT_HOMED;

    cm.machine_state = MACHINE_SHUTDOWN;        // do this after all other activity
    rpt_exception(status, msg);                 // send exception report
    return (status);
}

/*
 * cm_panic() - enter panic state
 *
 * PANIC occurs if the firmware has detected an unrecoverable internal error
 * such as an assertion failure or a code condition that should never occur.
 * It sets PANIC machine state, and leaves the system inspect able (if possible).
 *
 * PANIC can only be exited by a hardware reset or soft reset (^x)
 */

stat_t cm_panic(const stat_t status, const char *msg)
{
    _debug_trap(msg);

    if (cm.machine_state == MACHINE_PANIC) {    // only do this once
        return (STAT_OK);
    }
    cm_halt_motion();                           // halt motors (may have already been done from GPIO)
    spindle_reset();                            // stop spindle immediately and set speed to 0 RPM
    coolant_reset();                            // stop coolant immediately
    temperature_reset();                        // turn off heaters and fans
    cm_queue_flush();                           // flush all queues and reset positions

    cm.machine_state = MACHINE_PANIC;           // don't reset anything. Panics are not recoverable
    rpt_exception(status, msg);                 // send panic report
    return (status);
}

/**************************
 * Representation (4.3.3) *
 **************************/

/**************************************************************************
 * Representation functions that affect the Gcode model only (asynchronous)
 *
 *  cm_select_plane()           - G17,G18,G19 select axis plane
 *  cm_set_units_mode()         - G20, G21
 *  cm_set_distance_mode()      - G90, G91
 *  cm_set_arc_distance_mode()  - G90.1, G91.1
 *  cm_set_g10_data()           - G10 (delayed persistence)
 *
 *  These functions assume input validation occurred upstream.
 */

stat_t cm_select_plane(const uint8_t plane)
{
    cm.gm.select_plane = (cmCanonicalPlane)plane;
    return (STAT_OK);
}

stat_t cm_set_units_mode(const uint8_t mode)
{
    cm.gm.units_mode = (cmUnitsMode)mode;               // 0 = inches, 1 = mm.
    return(STAT_OK);
}

stat_t cm_set_distance_mode(const uint8_t mode)
{
    cm.gm.distance_mode = (cmDistanceMode)mode;         // 0 = absolute mode, 1 = incremental
    return (STAT_OK);
}

stat_t cm_set_arc_distance_mode(const uint8_t mode)
{
    cm.gm.arc_distance_mode = (cmDistanceMode)mode;     // 0 = absolute mode, 1 = incremental
    return (STAT_OK);
}

/*
 * cm_set_g10_data() - G10 L1/L2/L10/L20 Pn (affects MODEL only)
 *
 *  This function applies the offset to the GM model but does not persist the offsets
 *  during the Gcode cycle. The persist flag is used to persist offsets once the cycle
 *  has ended. You can also use $g54x - $g59c config functions to change offsets.
 *
 *  It also does not reset the work_offsets which may be accomplished by calling
 *  cm_set_work_offsets() immediately afterwards.
 */

stat_t cm_set_g10_data(const uint8_t P_word, const bool P_flag,
                       const uint8_t L_word, const bool L_flag,
                       const float offset[], const bool flag[])
{
    if (!L_flag) {
        return (STAT_L_WORD_IS_MISSING);
    }

    if ((L_word == 2) || (L_word == 20)) {
        // coordinate system offset command
        if ((P_word < G54) || (P_word > COORD_SYSTEM_MAX)) {
            // you can't set G53
            return (STAT_P_WORD_IS_INVALID);
        }
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            if (flag[axis]) {
                if (L_word == 2) {
                    cm.offset[P_word][axis] = _to_millimeters(offset[axis]);
                } else {
                    // Should L20 take into account G92 offsets?
                    cm.offset[P_word][axis] = 
                        cm.gmx.position[axis] -
                        _to_millimeters(offset[axis]) -
                        cm.tl_offset[axis];
                }
                // persist offsets once machining cycle is over
                cm.deferred_write_flag = true;
            }
        }
    }
    else if ((L_word == 1) || (L_word == 10)) {
        // tool table offset command. L11 not supported atm.
        if ((P_word < 1) || (P_word > TOOLS)) {
            return (STAT_P_WORD_IS_INVALID);
        }
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            if (flag[axis]) {
                if (L_word == 1) {
                    cm.tt_offset[P_word][axis] = _to_millimeters(offset[axis]);
                } else {
                    // L10 should also take into account G92 offset
                    cm.tt_offset[P_word][axis] =
                        cm.gmx.position[axis] - _to_millimeters(offset[axis]) -
                        cm.offset[cm.gm.coord_system][axis] -
                        (cm.gmx.origin_offset[axis] * cm.gmx.origin_offset_enable);
                }
                // persist offsets once machining cycle is over
                cm.deferred_write_flag = true;
            }
        }
    }
    else {
        return (STAT_L_WORD_IS_INVALID);
    }
    return (STAT_OK);
}

/******************************************************************************************
 * Representation functions that affect gcode model and are queued to planner (synchronous)
 */
/*
 * cm_set_tl_offset()    - G43
 * cm_cancel_tl_offset() - G49
 * cm_set_coord_system() - G54-G59
 * _exec_offset() - callback from planner
 */

stat_t cm_set_tl_offset(const uint8_t H_word, const bool H_flag, const bool apply_additional)
{
    uint8_t tool;
    if (H_flag) {
        if (H_word > TOOLS) {
            return (STAT_H_WORD_IS_INVALID);
        }
        if (H_word == 0) {    // interpret H0 as "current tool", just like no H at all.
            tool = cm.gm.tool;
        } else {
            tool = H_word;
        }
        } else {
        tool = cm.gm.tool;
    }
    if (apply_additional) {
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            cm.tl_offset[axis] += cm.tt_offset[tool][axis];
        }
        } else {
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            cm.tl_offset[axis] = cm.tt_offset[tool][axis];
        }
    }
    float value[] = { (float)cm.gm.coord_system,0,0,0,0,0 };// pass coordinate system in value[0] element
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_offset, value, flags);			// second vector (flags) is not used, so fake it
    return (STAT_OK);
}

stat_t cm_cancel_tl_offset()
{
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        cm.tl_offset[axis] = 0;
    }
    float value[] = { (float)cm.gm.coord_system,0,0,0,0,0 };// pass coordinate system in value[0] element
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_offset, value, flags);			// second vector (flags) is not used, so fake it
    return (STAT_OK);
}

stat_t cm_set_coord_system(const uint8_t coord_system)      // set coordinate system sync'd with planner
{
    cm.gm.coord_system = (cmCoordSystem)coord_system;

    float value[] = { (float)coord_system,0,0,0,0,0 };      // pass coordinate system in value[0] element
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_offset, value, flags);           // second vector (flags) is not used, so fake it
    return (STAT_OK);
}

static void _exec_offset(float *value, bool *flag)
{
    uint8_t coord_system = ((uint8_t)value[0]);             // coordinate system is passed in value[0] element
    float offsets[AXES];
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {

        offsets[axis] = cm.offset[coord_system][axis] + cm.tl_offset[axis] + 
                        (cm.gmx.origin_offset[axis] * cm.gmx.origin_offset_enable);
    }
    mp_set_runtime_work_offset(offsets);
    cm_set_work_offsets(MODEL);                             // set work offsets in the Gcode model
}

/*
 * cm_set_position() - set the position of a single axis in the model, planner and runtime
 *
 *    This command sets an axis/axes to a position provided as an argument.
 *    This is useful for setting origins for homing, probing, and other operations.
 *
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *  !!!!! DO NOT CALL THIS FUNCTION WHILE IN A MACHINING CYCLE !!!!!
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *    More specifically, do not call this function if there are any moves in the planner or
 *    if the runtime is moving. The system must be quiescent or you will introduce positional
 *    errors. This is true because the planned / running moves have a different reference frame
 *    than the one you are now going to set. These functions should only be called during
 *    initialization sequences and during cycles (such as homing cycles) when you know there
 *    are no more moves in the planner and that all motion has stopped.
 *    Use cm_get_runtime_busy() to be sure the system is quiescent.
 */

void cm_set_position(const uint8_t axis, const float position)
{
    // TODO: Interlock involving runtime_busy test
    cm.gmx.position[axis] = position;
    cm.gm.target[axis] = position;
    mp_set_planner_position(axis, position);
    mp_set_runtime_position(axis, position);
    mp_set_steps_to_runtime_position();
}

/*** G28.3 functions and support ***
 *
 * cm_set_absolute_origin() - G28.3 - model, planner and queue to runtime
 * _exec_absolute_origin()  - callback from planner
 *
 *  cm_set_absolute_origin() takes a vector of origins (presumably 0's, but not necessarily)
 *  and applies them to all axes where the corresponding position in the flag vector is true (1).
 *
 *  This is a 2 step process. The model and planner contexts are set immediately, the runtime
 *  command is queued and synchronized with the planner queue. This includes the runtime position
 *  and the step recording done by the encoders. At that point any axis that is set is also marked
 *  as homed.
 */

stat_t cm_set_absolute_origin(const float origin[], bool flag[])
{
    float value[AXES];

    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (flag[axis]) {
// REMOVED  value[axis] = cm.offset[cm.gm.coord_system][axis] + _to_millimeters(origin[axis]);    // G2 Issue #26
            value[axis] = _to_millimeters(origin[axis]);
            cm.gmx.position[axis] = value[axis];        // set model position
            cm.gm.target[axis] = value[axis];           // reset model target
            mp_set_planner_position(axis, value[axis]); // set mm position
        }
    }
    mp_queue_command(_exec_absolute_origin, value, flag);
    return (STAT_OK);
}

static void _exec_absolute_origin(float *value, bool *flag)
{
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (flag[axis]) {
            mp_set_runtime_position(axis, value[axis]);
            cm.homed[axis] = true;    // G28.3 is not considered homed until you get here
        }
    }
    mp_set_steps_to_runtime_position();
}

/*
 * cm_set_origin_offsets()     - G92
 * cm_reset_origin_offsets()   - G92.1
 * cm_suspend_origin_offsets() - G92.2
 * cm_resume_origin_offsets()  - G92.3
 *
 * G92's behave according to NIST 3.5.18 & LinuxCNC G92
 * http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G92-G92.1-G92.2-G92.3
 */

stat_t cm_set_origin_offsets(const float offset[], const bool flag[])
{
    // set offsets in the Gcode model extended context
    cm.gmx.origin_offset_enable = true;
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (flag[axis]) {
            cm.gmx.origin_offset[axis] = cm.gmx.position[axis] -
                                         cm.offset[cm.gm.coord_system][axis] - 
                                         cm.tl_offset[axis] -
                                         _to_millimeters(offset[axis]);
        }
    }
    // now pass the offset to the callback - setting the coordinate system also applies the offsets
    float value[] = { (float)cm.gm.coord_system,0,0,0,0,0 }; // pass coordinate system in value[0] element
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_offset, value, flags);
    return (STAT_OK);
}

stat_t cm_reset_origin_offsets()
{
    cm.gmx.origin_offset_enable = false;
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        cm.gmx.origin_offset[axis] = 0;
    }
    float value[] = { (float)cm.gm.coord_system,0,0,0,0,0 };
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_offset, value, flags);
    return (STAT_OK);
}

stat_t cm_suspend_origin_offsets()
{
    cm.gmx.origin_offset_enable = false;
    float value[] = { (float)cm.gm.coord_system,0,0,0,0,0 };
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_offset, value, flags);
    return (STAT_OK);
}

stat_t cm_resume_origin_offsets()
{
    cm.gmx.origin_offset_enable = true;
    float value[] = { (float)cm.gm.coord_system,0,0,0,0,0 };
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_offset, value, flags);
    return (STAT_OK);
}

/*****************************
 * Free Space Motion (4.3.4) *
 *****************************/
/*
 * cm_straight_traverse() - G0 linear rapid
 */

stat_t cm_straight_traverse(const float target[], const bool flags[])
{
    cm.gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;

    // it's legal for a G0 to have no axis words but we don't want to process it
    if (!(flags[AXIS_X] | flags[AXIS_Y] | flags[AXIS_Z] | flags[AXIS_A] | flags[AXIS_B] | flags[AXIS_C])) {
        return(STAT_OK);
    }

    cm_set_model_target(target, flags);
    ritorno (cm_test_soft_limits(cm.gm.target));    // test soft limits; exit if thrown
    cm_set_work_offsets(&cm.gm);                    // capture the fully resolved offsets to the state
    cm_cycle_start();                               // required for homing & other cycles
    stat_t status = mp_aline(&cm.gm);               // send the move to the planner
    cm_finalize_move();
    
    if (status == STAT_MINIMUM_LENGTH_MOVE) {
        if (!mp_has_runnable_buffer()) {            // handle condition where zero-length move is last or only move
            cm_cycle_end();                         // ...otherwise cycle will not end properly
        }
        status = STAT_OK;
    }
    return (status);
}

/*
 * cm_set_g28_position()   - G28.1
 * cm_goto_g28_position()  - G28
 * cm_set_g30_position()   - G30.1
 * cm_goto_g30_position()  - G30
 * _goto_stored_position() - helper
 */

stat_t _goto_stored_position(const float stored_position[],     // always in mm
                             const float intermediate_target[], // in current units (G20/G21)
                             const bool flags[])                // all false if no intermediate move
{
    // Go through intermediate point if one is provided
    while (mp_planner_is_full());                               // Make sure you have available buffers
    ritorno(cm_straight_traverse(intermediate_target, flags));  // w/no action if no axis flags

    // If G20 adjust stored position (always in mm) to inches so traverse will be correct
    float target[AXES]; // make a local stored position as it may be modified
    copy_vector(target, stored_position);
    if (cm.gm.units_mode == INCHES) {
        for (uint8_t i=0; i<AXES; i++) {
            target[i] *= INCHES_PER_MM;
        }
    }
    
    // Run the stored position move
    while (mp_planner_is_full());                           // Make sure you have available buffers

    uint8_t saved_distance_mode = cm_get_distance_mode(MODEL);
    cm_set_absolute_override(MODEL, ABSOLUTE_OVERRIDE_ON);  // Position was stored in absolute coords
    cm_set_distance_mode(ABSOLUTE_DISTANCE_MODE);           // Must run in absolute distance mode

    bool flags2[] = { 1,1,1,1,1,1 };
    stat_t status = cm_straight_traverse(target, flags2);   // Go to stored position
    cm_set_absolute_override(MODEL, ABSOLUTE_OVERRIDE_OFF);
    cm_set_distance_mode(saved_distance_mode);              // Restore distance mode
    return (status);
}

stat_t cm_set_g28_position(void)
{
    copy_vector(cm.gmx.g28_position, cm.gmx.position); // in MM and machine coordinates
    return (STAT_OK);
}

stat_t cm_goto_g28_position(const float target[], const bool flags[])
{
    return (_goto_stored_position(cm.gmx.g28_position, target, flags));
}

stat_t cm_set_g30_position(void)
{
    copy_vector(cm.gmx.g30_position, cm.gmx.position); // in MM and machine coordinates
    return (STAT_OK);
}

stat_t cm_goto_g30_position(const float target[], const bool flags[])
{
    return (_goto_stored_position(cm.gmx.g30_position, target, flags));
}

/********************************
 * Machining Attributes (4.3.5) *
 ********************************/
/*
 * cm_set_feed_rate() - F parameter (affects MODEL only)
 *
 * Normalize feed rate to mm/min or to minutes if in inverse time mode
 */

stat_t cm_set_feed_rate(const float feed_rate)
{
    if (cm.gm.feed_rate_mode == INVERSE_TIME_MODE) {
        if (fp_ZERO(feed_rate)) {
            return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
        }
        cm.gm.feed_rate = 1/feed_rate;    // normalize to minutes (NB: active for this gcode block only)
    } else {
        cm.gm.feed_rate = _to_millimeters(feed_rate);
    }
    return (STAT_OK);
}

/*
 * cm_set_feed_rate_mode() - G93, G94 (affects MODEL only)
 *
 *  INVERSE_TIME_MODE = 0,          // G93
 *  UNITS_PER_MINUTE_MODE,          // G94
 *  UNITS_PER_REVOLUTION_MODE       // G95 (unimplemented)
 */

stat_t cm_set_feed_rate_mode(const uint8_t mode)
{
    cm.gm.feed_rate_mode = (cmFeedRateMode)mode;
    return (STAT_OK);
}

/*
 * cm_set_path_control() - G61, G61.1, G64
 */

stat_t cm_set_path_control(GCodeState_t *gcode_state, const uint8_t mode)
{
    gcode_state->path_control = (cmPathControl)mode;
    return (STAT_OK);
}


/*******************************
 * Machining Functions (4.3.6) *
 *******************************/
/*
 * cm_arc_feed() - SEE plan_arc.cpp
 */


/*
 * cm_dwell() - G4, P parameter (seconds)
 */
stat_t cm_dwell(const float seconds)
{
    cm.gm.parameter = seconds;
    mp_dwell(seconds);
    return (STAT_OK);
}

/*
 * cm_straight_feed() - G1
 */
stat_t cm_straight_feed(const float target[], const bool flags[])
{
    // trap zero feed rate condition
    if (fp_ZERO(cm.gm.feed_rate)) {
        return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
    }
    cm.gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;

    if (!(flags[AXIS_X] | flags[AXIS_Y] | flags[AXIS_Z] | flags[AXIS_A] | flags[AXIS_B] | flags[AXIS_C])) {
        return(STAT_OK);
    }

    cm_set_model_target(target, flags);
    ritorno (cm_test_soft_limits(cm.gm.target));    // test soft limits; exit if thrown
    cm_set_work_offsets(&cm.gm);                    // capture the fully resolved offsets to the state
    cm_cycle_start();                               // required for homing & other cycles
    stat_t status = mp_aline(&cm.gm);               // send the move to the planner

    cm_finalize_move(); // <-- ONLY safe because we don't care about status...

    if (status == STAT_MINIMUM_LENGTH_MOVE) {
        if (!mp_has_runnable_buffer()) {            // handle condition where zero-length move is last or only move
            cm_cycle_end();                         // ...otherwise cycle will not end properly
        }
        status = STAT_OK;
    }
    return (status);
}

/*****************************
 * Spindle Functions (4.3.7) *
 *****************************/
// see spindle.cpp/.h

/**************************
 * Tool Functions (4.3.8) *
 **************************/
/*
 * cm_select_tool()     - T parameter
 * _exec_select_tool()  - execution callback
 *
 * cm_change_tool()     - M6 (This might become a complete tool change cycle)
 * _exec_change_tool()  - execution callback
 *
 * Note: These functions don't actually do anything for now, and there's a bug
 *       where T and M in different blocks don't work correctly
 */
stat_t cm_select_tool(const uint8_t tool_select)
{
    if (tool_select > TOOLS) {
        return (STAT_T_WORD_IS_INVALID);
    }
    float value[] = { (float)tool_select, 0,0,0,0,0 };
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_select_tool, value, flags);
    return (STAT_OK);
}

static void _exec_select_tool(float *value, bool *flag)
{
    cm.gm.tool_select = (uint8_t)value[0];
}

stat_t cm_change_tool(const uint8_t tool_change)
{
    float value[] = { (float)cm.gm.tool_select,0,0,0,0,0 };
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_change_tool, value, flags);
    return (STAT_OK);
}

static void _exec_change_tool(float *value, bool *flag)
{
    cm.gm.tool = (uint8_t)value[0];
}

/***********************************
 * Miscellaneous Functions (4.3.9) *
 ***********************************/
// see coolant.cpp/.h

/*
 * cm_message() - queue a RAM string as a message in the response (unconditionally)
 */

void cm_message(const char *message)
{
    nv_add_string((const char *)"msg", message);    // add message to the response object
}

/*
 * cm_reset_overrides() - reset manual feedrate and spindle overrides to initial conditions
 */

void cm_reset_overrides()
{
    cm.gmx.m48_enable = true;
    cm.gmx.mfo_enable = false;  // feed rate overrides
    cm.gmx.mfo_factor = 1.0;
    cm.gmx.mto_enable = false;  // traverse overrides
    cm.gmx.mto_factor = 1.0;
}

/*
static void _exec_feed_override(const bool m48_enable, const bool m50_enable, const float override_factor)
{
}
*/

/*
 * cm_m48_enable() - M48, M49
 *
 * M48 is the master enable for manual feedrate override and spindle override
 * If M48 is asserted M50 (mfo), M50.1 (mto) and M51 (sso) settings are in effect
 * If M49 is asserted M50 (mfo), M501. (mto) and M51 (sso) settings are in ignored
 *
 * See http://linuxcnc.org/docs/html/gcode/m-code.html#sec:M48,-M49-Speed-and-Feed-Override-Control
 */
/*  M48state    M48new      M50state    action (notes):
 *  disable     disable     disable     no action, no state change
 *  disable     disable     ENABLE      no action, no state change
 *
 *  disable     ENABLE      disable     no action, no state change
 *  disable     ENABLE      ENABLE      start ramp w/stored P value
 *
 *  ENABLE      disable     disable     no action, no state change
 *  ENABLE      disable     ENABLE      end ramp
 *
 *  ENABLE      ENABLE      disable     no action, no state change
 *  ENABLE      ENABLE      ENABLE      no action, no state change
 */
stat_t cm_m48_enable(uint8_t enable)        // M48, M49
{
    // handle changes to feed override given new state of m48/m49

    cm.gmx.m48_enable = enable;             // update state
    return (STAT_OK);
}

/*
 * cm_mfo_control() - M50 manual feed rate override comtrol
 *
 *  M50 enables manual feedrate override and the optional P override parameter.
 *  P is expressed as M% to N% of programmed feedrate, typically a value from 0.05 to 2.000.
 *  P may also be zero or missing. Behaviors:
 *
 *    P < minimum or P > maximum parameter, and not zero. Return error, no state change or action
 *    P omitted. Turn on feedrate override to current stored P value
 *    P = 0. Turn off feedrate override. (Do not change stored P value
 *    P = N. Turn on feedrate override to value of N, preserve new P value
 * See http://www.linuxcnc.org/docs/2.4/html/gcode_main.html#sec:M50:-Feed-Override
 *
 *  M48 is set ON on initialization and program end
 *  M50 is set OFF on initialization and program end
 *  P is set to 1.000 on initialization and program end (there is always a valid value)
 */
/* Implementation Notes:
 *
 * To do this correctly need to look not just at new values, but at current state
 * and transitions. See m48 for M48 transitions
 *
 *  M48state    M50enable   M50new   M50 endstate:  action (notes):
 *  disable     disable     M50 P0    disable       no action or state change
 *  disable     disable     M50       ENABLE        no action (m48 is disabled)
 *  disable     disable     M50 Pn    ENABLE        store new P value (no other action)
 *
 *  disable     ENABLE      M50 P0    disable       no action
 *  disable     ENABLE      M50       ENABLE        no action (m48 is disabled)
 *  disable     ENABLE      M50 Pn    ENABLE        store new P value (no other action)
 *
 *  ENABLE      disable     M50 P0    disable       no action or state change
 *  ENABLE      disable     M50       ENABLE        start ramp w/stored P value
 *  ENABLE      disable     M50 Pn    ENABLE        start ramp w/new P value; store P value
 *
 *  ENABLE      ENABLE      M50 P0    disable       end ramp
 *  ENABLE      ENABLE      M50       ENABLE        no action
 *  ENABLE      ENABLE      M50 Pn    ENABLE        start ramp w/new P value; store P value
 *                                                  (Note: new ramp will supercede any existing ramp)
 */

stat_t cm_mfo_control(const float P_word, const bool P_flag) // M50
{
    bool new_enable = true;
    bool new_override = false;
    if (P_flag) {                           // if parameter is present in Gcode block
        if (fp_ZERO(P_word)) {
            new_enable = false;             // P0 disables override
        } else {
            if (P_word < FEED_OVERRIDE_MIN) {
                return (STAT_INPUT_LESS_THAN_MIN_VALUE);
            }
            if (P_word > FEED_OVERRIDE_MAX) {
                return (STAT_INPUT_EXCEEDS_MAX_VALUE);
            }
            cm.gmx.mfo_factor = P_word;    // P word is valid, store it.
            new_override = true;
        }
    }
    if (cm.gmx.m48_enable) {               // if master enable is ON
        if (new_enable && (new_override || !cm.gmx.mfo_enable)) {   // 3 cases to start a ramp
            mp_start_feed_override(FEED_OVERRIDE_RAMP_TIME, cm.gmx.mfo_factor);
        } else if (cm.gmx.mfo_enable && !new_enable) {              // case to turn off the ramp
            mp_end_feed_override(FEED_OVERRIDE_RAMP_TIME);
        }
    }
    cm.gmx.mfo_enable = new_enable;        // always update the enable state
    return (STAT_OK);
}

stat_t cm_mto_control(const float P_word, const bool P_flag) // M50.1
{
    bool new_enable = true;
    bool new_override = false;
    if (P_flag) {                           // if parameter is present in Gcode block
        if (fp_ZERO(P_word)) {
            new_enable = false;             // P0 disables override
        } else {
            if (P_word < TRAVERSE_OVERRIDE_MIN) {
                return (STAT_INPUT_LESS_THAN_MIN_VALUE);
            }
            if (P_word > TRAVERSE_OVERRIDE_MAX) {
                return (STAT_INPUT_EXCEEDS_MAX_VALUE);
            }
            cm.gmx.mto_factor = P_word;    // P word is valid, store it.
            new_override = true;
        }
    }
    if (cm.gmx.m48_enable) {               // if master enable is ON
        if (new_enable && (new_override || !cm.gmx.mfo_enable)) {   // 3 cases to start a ramp
            mp_start_traverse_override(FEED_OVERRIDE_RAMP_TIME, cm.gmx.mto_factor);
        } else if (cm.gmx.mto_enable && !new_enable) {              // case to turn off the ramp
            mp_end_traverse_override(FEED_OVERRIDE_RAMP_TIME);
        }
    }
    cm.gmx.mto_enable = new_enable;        // always update the enable state
    return (STAT_OK);
}

/************************************************
 * Feedhold and Related Functions (no NIST ref) *
 ************************************************/
/*
 * Feedholds, queue flushes and end_holds are all related. The request functions set flags
 * or change state to "REQUESTED". The sequencing callback interprets the flags as so:
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
 *
 *  Below the request level, feedholds work like this:
 *    - The hold is initiated by calling cm_start_hold(). cm.hold_state is set to
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
 *    - Once deceleration is complete hold state transitions to FEEDHOLD_HOLD and the
 *      distance remaining in the bf last block is replanned up from zero velocity.
 *      The move in the bf block is NOT released (unlike normal operation), as it
 *      will be used again to restart from hold.
 *
 *    - When cm_end_hold() is called it releases the hold, restarts the move and restarts
 *      the spindle if the spindle is active.
 */
/* Queue Flush operation
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
/*
 * cm_request_feedhold()
 * cm_request_end_hold() - cycle restart
 * cm_request_queue_flush()
 */
void cm_request_feedhold(void) {
    // honor request if not already in a feedhold and you are moving
    if ((cm.hold_state == FEEDHOLD_OFF) && (cm.motion_state != MOTION_STOP)) {
        cm.hold_state = FEEDHOLD_REQUESTED;
    }
}

void cm_request_end_hold(void)
{
    if (cm.hold_state != FEEDHOLD_OFF) {
        cm.end_hold_requested = true;
    }
}

void cm_request_queue_flush()
{
    if ((cm.hold_state != FEEDHOLD_OFF) &&          // don't honor request unless you are in a feedhold
        (cm.queue_flush_state == FLUSH_OFF)) {      // ...and only once
        cm.queue_flush_state = FLUSH_REQUESTED;     // request planner flush once motion has stopped

        // NOTE: we used to flush the input buffers, but this is handled in xio *prior* to queue flush now
    }
}

/*
 * cm_feedhold_sequencing_callback() - sequence feedhold, queue_flush, and end_hold requests
 */
stat_t cm_feedhold_sequencing_callback()
{
    if (cm.hold_state == FEEDHOLD_REQUESTED) {
        cm_start_hold();                            // feed won't run unless the machine is moving
    }
    if (cm.queue_flush_state == FLUSH_REQUESTED) {
        cm_queue_flush();                           // queue flush won't run until runtime is idle
    }
    if (cm.end_hold_requested) {
        if (cm.queue_flush_state == FLUSH_OFF) {    // either no flush or wait until it's done flushing
            cm_end_hold();
        }
    }
    return (STAT_OK);
}

/*
 * cm_has_hold()   - return true if a hold condition exists (or a pending hold request)
 * cm_start_hold() - start a feedhhold by signalling the exec
 * cm_end_hold()   - end a feedhold by returning the system to normal operation
 * cm_queue_flush() - Flush planner queue and correct model positions
 */
bool cm_has_hold()
{
    return (cm.hold_state != FEEDHOLD_OFF);
}

void cm_start_hold()
{
    if (mp_has_runnable_buffer()) {                         // meaning there's something running
        cm_spindle_optional_pause(spindle.pause_on_hold);   // pause if this option is selected
        cm_coolant_optional_pause(coolant.pause_on_hold);   // pause if this option is selected
        cm_set_motion_state(MOTION_HOLD);
        cm.hold_state = FEEDHOLD_SYNC;                      // invokes hold from aline execution
    }
}

void cm_end_hold()
{
    if (cm.hold_state == FEEDHOLD_HOLD) {
        cm.end_hold_requested = false;
        mp_exit_hold_state();

        // State machine cases:
        if (cm.machine_state == MACHINE_ALARM) {
            cm_spindle_off_immediate();
            cm_coolant_off_immediate();

        } else if (cm.motion_state == MOTION_STOP) { // && (! MACHINE_ALARM)
            cm_spindle_off_immediate();
            cm_coolant_off_immediate();
            cm_cycle_end();

        } else {    // (MOTION_RUN || MOTION_PLANNING)  && (! MACHINE_ALARM)
            cm_cycle_start();
            cm_spindle_resume(spindle.dwell_seconds);
            cm_coolant_resume();
            st_request_exec_move();
        }
    }
}

void cm_queue_flush()
{
    if (mp_runtime_is_idle()) {                     // can't flush planner during movement
        mp_flush_planner();

        for (uint8_t axis = AXIS_X; axis < AXES; axis++) { // set all positions
            cm_set_position(axis, mp_get_runtime_absolute_position(axis));
        }
        if(cm.hold_state == FEEDHOLD_HOLD) {        // end feedhold if we're in one
            cm_end_hold();
        }
        cm.queue_flush_state = FLUSH_OFF;
        qr_request_queue_report(0);                 // request a queue report, since we've changed the number of buffers available
    }
}

/******************************
 * Program Functions (4.3.10) *
 ******************************/
/* This group implements stop, start, and end functions.
 * It is extended beyond the NIST spec to handle various situations.
 *
 * _exec_program_finalize()     - helper
 * cm_cycle_start()
 * cm_cycle_end()
 * cm_program_stop()            - M0
 * cm_optional_program_stop()   - M1
 * cm_program_end()             - M2, M30
 */
/*
 * Program and cycle state functions
 *
 * cm_program_stop and cm_optional_program_stop are synchronous Gcode commands
 * that are received through the interpreter. They cause all motion to stop
 * at the end of the current command, including spindle motion.
 *
 * Note that the stop occurs at the end of the immediately preceding command
 * (i.e. the stop is queued behind the last command).
 *
 * cm_program_end is a stop that also resets the machine to initial state
 *
 * cm_program_end() implements M2 and M30
 * The END behaviors are defined by NIST 3.6.1 are:
 *    1a.    Origin offsets are set to the default (like G54)
 *    1b. Axis offsets are set to zero (like G92.2)
 *    2.  Selected plane is set to CANON_PLANE_XY (like G17)
 *    3.  Distance mode is set to MODE_ABSOLUTE (like G90)
 *    4.  Feed rate mode is set to UNITS_PER_MINUTE (like G94)
 *    5.  Feed and speed overrides are set to ON (like M48)
 *    6.  Cutter compensation is turned off (like G40)
 *    7.  The spindle is stopped (like M5)
 *    8.  The current motion mode is set to G_1 (like G1)
 *    9.  Coolant is turned off (like M9)
 *
 * cm_program_end() implments things slightly differently (1a, 8):
 *    1a. Set default coordinate system (uses $gco, not G54)
 *    1b. Axis offsets are SUSPENDED (G92.2)
 *    2.  Selected plane is set to default plane ($gpl)
 *    3.  Distance mode is set to MODE_ABSOLUTE (like G90)
 *    4.  Feed rate mode is set to UNITS_PER_MINUTE (like G94)
 *    5.  Not implemented
 *    6.  Not implemented
 *    7.  The spindle is stopped (like M5)
 *    8.  Motion mode is CANCELED like G80 (not set to G1 as per NIST)
 *    9.  Coolant is turned off (like M9)
 */

static void _exec_program_finalize(float *value, bool *flag)
{
    cm_set_motion_state(MOTION_STOP);

    // Allow update in the alarm state, to accommodate queue flush (RAS)
    if ((cm.cycle_state == CYCLE_MACHINING || cm.cycle_state == CYCLE_OFF) &&
//      (cm.machine_state != MACHINE_ALARM) &&          // omitted by OMC (RAS)
        (cm.machine_state != MACHINE_SHUTDOWN)) {
        cm.machine_state = (cmMachineState)value[0];    // don't update macs/cycs if we're in the middle of a canned cycle,
        cm.cycle_state = CYCLE_OFF;                     // or if we're in machine alarm/shutdown mode
    }

    // reset the rest of the states
    cm.cycle_state = CYCLE_OFF;
    cm.hold_state = FEEDHOLD_OFF;
    mp_zero_segment_velocity();                         // for reporting purposes

    // perform the following resets if it's a program END
    if (((uint8_t)value[0]) == MACHINE_PROGRAM_END) {
        cm_suspend_origin_offsets();                    // G92.2 - as per NIST
//      cm_reset_origin_offsets();                      // G92.1 - alternative to above
        cm_set_coord_system(cm.default_coord_system);   // reset to default coordinate system
        cm_select_plane(cm.default_select_plane);       // reset to default arc plane
        cm_set_distance_mode(cm.default_distance_mode);
        cm_set_arc_distance_mode(INCREMENTAL_DISTANCE_MODE); // always the default
        cm_spindle_off_immediate();                     // M5
        cm_coolant_off_immediate();                     // M9
        cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);   // G94
        cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);// NIST specifies G1 (MOTION_MODE_STRAIGHT_FEED), but we cancel motion mode. Safer.
        cm_reset_overrides();                           // reset feedrate the spindle overrides
        temperature_reset();                            // turn off all heaters and fans
    }
    sr_request_status_report(SR_REQUEST_IMMEDIATE);     // request a final and full status report (not filtered)
}

void cm_cycle_start()
{
    if (cm.cycle_state == CYCLE_OFF) {                  // don't (re)start homing, probe or other canned cycles
        cm.machine_state = MACHINE_CYCLE;
        cm.cycle_state = CYCLE_MACHINING;
        qr_init_queue_report();                         // clear queue reporting buffer counts
    }
}

void cm_cycle_end()
{
    if(cm.cycle_state == CYCLE_MACHINING) {
        float value[] = { (float)MACHINE_PROGRAM_STOP, 0,0,0,0,0 };
        bool flags[]  = { 1,0,0,0,0,0 };
        _exec_program_finalize(value, flags);
    }
}

void cm_canned_cycle_end()
{
    cm.cycle_state = CYCLE_OFF;
    float value[] = { (float)MACHINE_PROGRAM_STOP, 0,0,0,0,0 };
    bool flags[]  = { 1,0,0,0,0,0 };
    _exec_program_finalize(value, flags);
}

void cm_program_stop()
{
    float value[] = { (float)MACHINE_PROGRAM_STOP, 0,0,0,0,0 };
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_program_finalize, value, flags);
}

void cm_optional_program_stop()
{
    float value[] = { (float)MACHINE_PROGRAM_STOP, 0,0,0,0,0 };
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_program_finalize, value, flags);
}

void cm_program_end()
{
    float value[] = { (float)MACHINE_PROGRAM_END, 0,0,0,0,0 };
    bool flags[]  = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_program_finalize, value, flags);
}


/*
 * cm_json_command() - M100
 */
stat_t cm_json_command(char *json_string)
{
    return mp_json_command(json_string);
}

/*
 * cm_json_command_immediate() - M100.1
 */
stat_t cm_json_command_immediate(char *json_string)
{
    return mp_json_command_immediate(json_string);
}

/*
 * cm_json_wait() - M102
 */
stat_t cm_json_wait(char *json_string)
{
    return mp_json_wait(json_string);
}


/**************************************
 * END OF CANONICAL MACHINE FUNCTIONS *
 **************************************/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

// Strings for writing settings as nvObj string values
// Ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=120881&start=0

#ifdef __TEXT_MODE

static const char msg_units0[] = " in";    // used by generic print functions
static const char msg_units1[] = " mm";
static const char msg_units2[] = " deg";
static const char *const msg_units[] = { msg_units0, msg_units1, msg_units2 };
#define DEGREE_INDEX 2

static const char msg_am00[] = "[disabled]";
static const char msg_am01[] = "[standard]";
static const char msg_am02[] = "[inhibited]";
static const char msg_am03[] = "[radius]";
static const char *const msg_am[] = { msg_am00, msg_am01, msg_am02, msg_am03};

static const char msg_g20[] = "G20 - inches mode";
static const char msg_g21[] = "G21 - millimeter mode";
static const char *const msg_unit[] = { msg_g20, msg_g21 };

static const char msg_stat0[] = "Initializing";    // combined state (stat) uses this array
static const char msg_stat1[] = "Ready";
static const char msg_stat2[] = "Alarm";
static const char msg_stat3[] = "Stop";
static const char msg_stat4[] = "End";
static const char msg_stat5[] = "Run";
static const char msg_stat6[] = "Hold";
static const char msg_stat7[] = "Probe";
static const char msg_stat8[] = "Cycle";
static const char msg_stat9[] = "Homing";
static const char msg_stat10[] = "Jog";
static const char msg_stat11[] = "Interlock";
static const char msg_stat12[] = "Shutdown";
static const char msg_stat13[] = "Panic";
static const char *const msg_stat[] = { msg_stat0, msg_stat1, msg_stat2, msg_stat3,
                                                msg_stat4, msg_stat5, msg_stat6, msg_stat7,
                                                msg_stat8, msg_stat9, msg_stat10, msg_stat11,
                                                msg_stat12, msg_stat13 };

static const char msg_macs0[] = "Initializing";
static const char msg_macs1[] = "Ready";
static const char msg_macs2[] = "Alarm";
static const char msg_macs3[] = "Stop";
static const char msg_macs4[] = "End";
static const char msg_macs5[] = "Cycle";
static const char msg_macs6[] = "Interlock";
static const char msg_macs7[] = "SHUTDOWN";
static const char msg_macs8[] = "PANIC";
static const char *const msg_macs[] = { msg_macs0, msg_macs1, msg_macs2, msg_macs3,
                                                msg_macs4, msg_macs5, msg_macs6, msg_macs7,
                                                msg_macs8 };

static const char msg_cycs0[] = "Off";
static const char msg_cycs1[] = "Machining";
static const char msg_cycs2[] = "Homing";
static const char msg_cycs3[] = "Probe";
static const char msg_cycs4[] = "Jog";
static const char *const msg_cycs[] = { msg_cycs0, msg_cycs1, msg_cycs2, msg_cycs3,  msg_cycs4 };

static const char msg_mots0[] = "Stop";
static const char msg_mots1[] = "Planning";
static const char msg_mots2[] = "Run";
static const char msg_mots3[] = "Hold";
static const char *const msg_mots[] = { msg_mots0, msg_mots1, msg_mots2, msg_mots3 };

static const char msg_hold0[] = "Off";
static const char msg_hold1[] = "Requested";
static const char msg_hold2[] = "Sync";
static const char msg_hold3[] = "Decel Continue";
static const char msg_hold4[] = "Decel to Zero";
static const char msg_hold5[] = "Decel Done";
static const char msg_hold6[] = "Pending";
static const char msg_hold7[] = "Hold";
static const char *const msg_hold[] = { msg_hold0, msg_hold1, msg_hold2, msg_hold3,
                                                msg_hold4, msg_hold5, msg_hold6, msg_hold7 };

static const char msg_home0[] = "Not Homed";
static const char msg_home1[] = "Homed";
static const char msg_home2[] = "Homing";
static const char *const msg_home[] = { msg_home0, msg_home1, msg_home2 };

static const char msg_g53[] = "G53 - machine coordinate system";
static const char msg_g54[] = "G54 - coordinate system 1";
static const char msg_g55[] = "G55 - coordinate system 2";
static const char msg_g56[] = "G56 - coordinate system 3";
static const char msg_g57[] = "G57 - coordinate system 4";
static const char msg_g58[] = "G58 - coordinate system 5";
static const char msg_g59[] = "G59 - coordinate system 6";
static const char *const msg_coor[] = { msg_g53, msg_g54, msg_g55, msg_g56, msg_g57, msg_g58, msg_g59 };

static const char msg_g00[] = "G0  - linear traverse";
static const char msg_g01[] = "G1  - linear feed";
static const char msg_g02[] = "G2  - clockwise arc feed";
static const char msg_g03[] = "G3  - counter clockwise arc feed";
static const char msg_g80[] = "G80 - cancel motion mode (none active)";
static const char *const msg_momo[] = { msg_g00, msg_g01, msg_g02, msg_g03, msg_g80 };

static const char msg_g17[] = "G17 - XY plane";
static const char msg_g18[] = "G18 - XZ plane";
static const char msg_g19[] = "G19 - YZ plane";
static const char *const msg_plan[] = { msg_g17, msg_g18, msg_g19 };

static const char msg_g61[] = "G61 - exact path mode";
static const char msg_g6a[] = "G61.1 - exact stop mode";
static const char msg_g64[] = "G64 - continuous mode";
static const char *const msg_path[] = { msg_g61, msg_g6a, msg_g64 };

static const char msg_g90[] = "G90 - absolute distance mode";
static const char msg_g91[] = "G91 - incremental distance mode";
static const char *const msg_dist[] = { msg_g90, msg_g91 };

static const char msg_g901[] = "G90.1 - absolute distance mode";
static const char msg_g911[] = "G91.1 - incremental distance mode (default mode)";
static const char *const msg_admo[] = { msg_g901, msg_g911 };

static const char msg_g93[] = "G93 - inverse time mode";
static const char msg_g94[] = "G94 - units-per-minute mode (i.e. feedrate mode)";
static const char msg_g95[] = "G95 - units-per-revolution mode";
static const char *const msg_frmo[] = { msg_g93, msg_g94, msg_g95 };

#else

#define msg_units NULL
#define msg_unit NULL
#define msg_stat NULL
#define msg_macs NULL
#define msg_cycs NULL
#define msg_mots NULL
#define msg_hold NULL
#define msg_home NULL
#define msg_coor NULL
#define msg_momo NULL
#define msg_plan NULL
#define msg_path NULL
#define msg_dist NULL
#define msg_admo NULL
#define msg_frmo NULL
#define msg_am NULL

#endif // __TEXT_MODE


/***** AXIS HELPERS *****************************************************************
 * _get_axis()        - return axis # or -1 if not an axis (works for mapped motors as well)
 * _coord()           - return coordinate system number or -1 if error
 * cm_get_axis_char() - return ASCII char for axis given the axis number
 * cm_get_axis_type() - return linear axis (0), rotary axis (1) or error (-1)
 */

/* _get_axis()
 *
 *  Cases that are handled by _get_axis():
 *    - sys/... value is a system parameter (global), there is no axis
 *    - xam     any axis parameter will return the axis number
 *    - 1ma     any motor parameter will return the mapped axis for that motor
 *    - 1su     an example of the above
 *    - mpox    readouts
 *    - g54x    offsets
 *    - tlx     tool length offset
 *    - tt1x    tool table
 *    - tt32x   tool table
 *    - _tex    diagnostic parameters
 */

static int8_t _get_axis(const index_t index)
{
    // test if this is a SYS parameter (global), in which case there will be no axis    
    if (strcmp("sys", cfgArray[index].group) == 0) {
        return (AXIS_TYPE_SYSTEM);
    }

    // if the leading character of the token is a number it's a motor
    char c = cfgArray[index].token[0];
    if (isdigit(cfgArray[index].token[0])) {
        return(st_cfg.mot[c-0x31].motor_map);
    }
        
    // otherwise it's an axis. Or undefined, which is usually a global.
    char *ptr;
    char axes[] = {"xyzabc"};
    if ((ptr = strchr(axes, c)) == NULL) {              // test the character in the 0 and 3 positions
        if ((ptr = strchr(axes, cfgArray[index].token[3])) == NULL) { // to accommodate 'xam' and 'g54x' styles
            return (AXIS_TYPE_UNDEFINED);
        }
    }
    return (ptr - axes);
}

/**** not used yet ****
static int8_t _coord(char *token) // extract coordinate system from 3rd character
{
    char *ptr;
    char coord_list[] = {"456789"};

    if ((ptr = strchr(coord_list, token[2])) == NULL) { // test the 3rd character against the string
        return (-1);
    }
    return (ptr - coord_list);
}
*/

char cm_get_axis_char(const int8_t axis)
{
    char axis_char[] = "XYZABC";
    if ((axis < 0) || (axis > AXES)) return (' ');
    return (axis_char[axis]);
}

cmAxisType cm_get_axis_type(const index_t index)
{
    int8_t axis = _get_axis(index);
    if (axis == AXIS_TYPE_UNDEFINED) { return (AXIS_TYPE_UNDEFINED); }
    if (axis == AXIS_TYPE_SYSTEM) { return (AXIS_TYPE_SYSTEM); }
    if (axis >= AXIS_A) { return (AXIS_TYPE_ROTARY); }
    return (AXIS_TYPE_LINEAR);
}

/**** Functions called directly from cfgArray table - mostly wrappers ****
 * _get_msg_helper() - helper to get string values
 *
 * cm_get_stat() - get combined machine state as value and string
 * cm_get_macs() - get raw machine state as value and string
 * cm_get_cycs() - get raw cycle state as value and string
 * cm_get_mots() - get raw motion state as value and string
 * cm_get_hold() - get raw hold state as value and string
 * cm_get_home() - get raw homing state as value and string
 *
 * cm_get_unit() - get units mode as integer and display string
 * cm_get_coor() - get goodinate system
 * cm_get_momo() - get runtime motion mode
 * cm_get_plan() - get model plane select
 * cm_get_path() - get model path control mode
 * cm_get_dist() - get model distance mode
 * cm_get_admo() - get model arc distance mode
 * cm_get_frmo() - get model feed rate mode
 * cm_get_tool() - get tool
 * cm_get_feed() - get feed rate
 * cm_get_mline()- get model line number for status reports
 * cm_get_line() - get active (model or runtime) line number for status reports
 * cm_get_vel()  - get runtime velocity
 * cm_get_ofs()  - get current work offset (runtime)
 * cm_get_pos()  - get current work position (runtime)
 * cm_get_mpos() - get current machine position (runtime)
 *
 * cm_print_pos()- print work position (with proper units)
 * cm_print_mpos()- print machine position (always mm units)
 * cm_print_coor()- print coordinate offsets with linear units
 * cm_print_corr()- print coordinate offsets with rotary units
 */

// Add the string for the enum to the nv, but leave it as a TYPE_INT
stat_t _get_msg_helper(nvObj_t *nv, const char *const msg_array[], uint8_t value)
{
    nv->value = (float)value;
    nv->valuetype = TYPE_INT;
    return(nv_copy_string(nv, (const char *)GET_TEXT_ITEM(msg_array, value)));
}

stat_t cm_get_stat(nvObj_t *nv) { return(_get_msg_helper(nv, msg_stat, cm_get_combined_state()));}
stat_t cm_get_macs(nvObj_t *nv) { return(_get_msg_helper(nv, msg_macs, cm_get_machine_state()));}
stat_t cm_get_cycs(nvObj_t *nv) { return(_get_msg_helper(nv, msg_cycs, cm_get_cycle_state()));}
stat_t cm_get_mots(nvObj_t *nv) { return(_get_msg_helper(nv, msg_mots, cm_get_motion_state()));}
stat_t cm_get_hold(nvObj_t *nv) { return(_get_msg_helper(nv, msg_hold, cm_get_hold_state()));}
stat_t cm_get_home(nvObj_t *nv) { return(_get_msg_helper(nv, msg_home, cm_get_homing_state()));}

stat_t cm_get_unit(nvObj_t *nv) { return(_get_msg_helper(nv, msg_unit, cm_get_units_mode(ACTIVE_MODEL)));}
stat_t cm_get_coor(nvObj_t *nv) { return(_get_msg_helper(nv, msg_coor, cm_get_coord_system(ACTIVE_MODEL)));}
stat_t cm_get_momo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_momo, cm_get_motion_mode(ACTIVE_MODEL)));}
stat_t cm_get_plan(nvObj_t *nv) { return(_get_msg_helper(nv, msg_plan, cm_get_select_plane(ACTIVE_MODEL)));}
stat_t cm_get_path(nvObj_t *nv) { return(_get_msg_helper(nv, msg_path, cm_get_path_control(ACTIVE_MODEL)));}
stat_t cm_get_dist(nvObj_t *nv) { return(_get_msg_helper(nv, msg_dist, cm_get_distance_mode(ACTIVE_MODEL)));}
stat_t cm_get_admo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_admo, cm_get_arc_distance_mode(ACTIVE_MODEL)));}
stat_t cm_get_frmo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_frmo, cm_get_feed_rate_mode(ACTIVE_MODEL)));}

stat_t cm_get_toolv(nvObj_t *nv)
{
    nv->value = (float)cm_get_tool(ACTIVE_MODEL);
    nv->valuetype = TYPE_INT;
    return (STAT_OK);
}

stat_t cm_get_mline(nvObj_t *nv)
{
    nv->value = (float)cm_get_linenum(MODEL);
    nv->valuetype = TYPE_INT;
    return (STAT_OK);
}

stat_t cm_get_line(nvObj_t *nv)
{
    nv->value = (float)cm_get_linenum(ACTIVE_MODEL);
    nv->valuetype = TYPE_INT;
    return (STAT_OK);
}

stat_t cm_get_vel(nvObj_t *nv)
{
    if (cm_get_motion_state() == MOTION_STOP) {
        nv->value = 0;
    } else {
        nv->value = mp_get_runtime_velocity();
        if (cm_get_units_mode(RUNTIME) == INCHES) {
            nv->value *= INCHES_PER_MM;
        }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

stat_t cm_get_feed(nvObj_t *nv)
{
    nv->value = cm_get_feed_rate(ACTIVE_MODEL);
    if (cm_get_units_mode(ACTIVE_MODEL) == INCHES) {
        nv->value *= INCHES_PER_MM;
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

stat_t cm_get_pos(nvObj_t *nv)
{
    nv->value = cm_get_work_position(ACTIVE_MODEL, _get_axis(nv->index));
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

stat_t cm_get_mpo(nvObj_t *nv)
{
    nv->value = cm_get_absolute_position(ACTIVE_MODEL, _get_axis(nv->index));
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

stat_t cm_get_ofs(nvObj_t *nv)
{
    nv->value = cm_get_work_offset(ACTIVE_MODEL, _get_axis(nv->index));
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

stat_t cm_get_tof(nvObj_t *nv)
{
    nv->value = cm.tl_offset[_get_axis(nv->index)];
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

/*
 * AXIS GET AND SET FUNCTIONS
 *
 * cm_get_am() - get axis mode w/enumeration string
 * cm_set_am() - set axis mode w/exception handling for axis type
 * cm_set_hi() - set homing input
 */

stat_t cm_get_am(nvObj_t *nv)
{
    get_ui8(nv);
    return(_get_msg_helper(nv, msg_am, nv->value));
}

stat_t cm_set_am(nvObj_t *nv)        // axis mode
{
    if (cm_get_axis_type(nv->index) == 0) {    // linear
        if (nv->value > AXIS_MODE_MAX_LINEAR) { 
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
    } else {
        if (nv->value > AXIS_MODE_MAX_ROTARY) { 
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
    }
    set_ui8(nv);
    return(STAT_OK);
}

stat_t cm_set_hi(nvObj_t *nv)
{
    if (nv->value < 0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value > D_IN_CHANNELS) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    set_ui8(nv);
    return (STAT_OK);
}

/**** Velocity and Jerk functions
 * cm_get_axis_jerk() - returns jerk for an axis
 * cm_set_axis_jerk() - sets the jerk for an axis, including recirpcal and cached values
 *
 * cm_set_vm() - set velocity max value - called from dispatch table
 * cm_set_fr() - set feedrate max value - called from dispatch table
 * cm_set_jm() - set jerk max value - called from dispatch table
 * cm_set_jh() - set jerk homing value - called from dispatch table
 *
 *  Jerk values can be rather large, often in the billions. This makes for some pretty big
 *  numbers for people to deal with. Jerk values are stored in the system in truncated format;
 *  values are divided by 1,000,000 then reconstituted before use.
 *
 *  The set_xjm() nad set_xjh() functions will accept either truncated or untruncated jerk
 *  numbers as input. If the number is > 1,000,000 it is divided by 1,000,000 before storing.
 *  Numbers are accepted in either millimeter or inch mode and converted to millimeter mode.
 *
 *  The axis_jerk() functions expect the jerk in divided-by 1,000,000 form
 */
float cm_get_axis_jerk(const uint8_t axis)
{
    return (cm.a[axis].jerk_max);
}

// Precompute sqrt(3)/10 for the max_junction_accel.
// See plan_line.cpp -> _calculate_junction_vmax() notes for details.
static const float _junction_accel_multiplier = sqrt(3.0)/10.0;

// Important note: Actual jerk is stored jerk * JERK_MULTIPLIER, and
// Time Quanta is junction_integration_time / 1000.
// We no longer incorporate jerk into this, since it can be channged per-move.
void _cm_recalc_max_junction_accel(const uint8_t axis) {
    float T = cm.junction_integration_time / 1000.0;
    float T2 = T*T;

    cm.a[axis].max_junction_accel = _junction_accel_multiplier * T2 * JERK_MULTIPLIER;
}

void cm_set_axis_jerk(const uint8_t axis, const float jerk)
{
    cm.a[axis].jerk_max = jerk;
}

stat_t cm_set_vm(nvObj_t *nv)
{
    uint8_t axis = _get_axis(nv->index);
    if ((axis == AXIS_A) || (axis == AXIS_B) || (axis == AXIS_C)) {
        ritorno(set_fltp(nv));
    } else {
        ritorno(set_flup(nv));
    }
    cm.a[axis].recip_velocity_max = 1/nv->value;
    return(STAT_OK);
}

stat_t cm_set_fr(nvObj_t *nv)
{
    uint8_t axis = _get_axis(nv->index);
    if ((axis == AXIS_A) || (axis == AXIS_B) || (axis == AXIS_C)) {
        ritorno(set_fltp(nv));
    } else {
        ritorno(set_flup(nv));
    }
    cm.a[axis].recip_feedrate_max = 1/nv->value;
    return(STAT_OK);
}

stat_t cm_set_jm(nvObj_t *nv)
{
    if (nv->value < JERK_INPUT_MIN) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value > JERK_INPUT_MAX) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    set_flu(nv);
    cm_set_axis_jerk(_get_axis(nv->index), nv->value);
    return(STAT_OK);
}

stat_t cm_set_jh(nvObj_t *nv)
{
    if (nv->value < JERK_INPUT_MIN) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value > JERK_INPUT_MAX) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    set_flu(nv);
    return(STAT_OK);
}

stat_t cm_set_jt(nvObj_t *nv)
{
    stat_t status = STAT_OK;

    if (nv->value < JUNCTION_INTEGRATION_MIN) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value > JUNCTION_INTEGRATION_MAX) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    set_flt(nv);

    // Must recalculate the max_junction_accel now that the time quanta has changed.
    for (uint8_t axis=0; axis<AXES; axis++) {
        _cm_recalc_max_junction_accel(axis);
    }
    return(status);
}

/*
 * cm_set_mfo() - set manual feedrate override factor
 * cm_set_mto() - set manual traverse override factor
 */

stat_t cm_set_mfo(nvObj_t *nv)
{
    if (nv->value < FEED_OVERRIDE_MIN) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value > FEED_OVERRIDE_MAX) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    set_flt(nv);
    return(STAT_OK);
}

stat_t cm_set_mto(nvObj_t *nv)
{
    if (nv->value < TRAVERSE_OVERRIDE_MIN) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value > TRAVERSE_OVERRIDE_MAX) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    set_flt(nv);
    return(STAT_OK);
}

/*
 * cm_get_so() - get spring factor offset
 *
 */

stat_t cm_get_so(nvObj_t *nv)
{
    if (cm_get_motion_state() == MOTION_STOP) {
        nv->value = 0;
    } else {
        nv->value = mp_get_runtime_spring_value(_get_axis(nv->index));
        if (cm_get_units_mode(RUNTIME) == INCHES) {
            nv->value *= INCHES_PER_MM;
        }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

/*
 * Commands
 *
 * cm_run_qf() - flush planner queue
 * cm_run_home() - run homing sequence
 */

stat_t cm_run_qf(nvObj_t *nv)
{
    cm_request_queue_flush();
    return (STAT_OK);
}

stat_t cm_run_home(nvObj_t *nv)
{
    if (fp_TRUE(nv->value)) {
        float axes[] = { 1,1,1,1,1,1 };
        bool flags[] = { 1,1,1,1,1,1 };
        cm_homing_cycle_start(axes, flags);
    }
    return (STAT_OK);
}

/*
 * Debugging Commands
 *
 * cm_dam() - dump active model
 */

stat_t cm_dam(nvObj_t *nv)
{
    xio_writeline("Active model:\n");
    cm_print_vel(nv);
    cm_print_feed(nv);
    cm_print_line(nv);
    cm_print_stat(nv);
    cm_print_macs(nv);
    cm_print_cycs(nv);
    cm_print_mots(nv);
    cm_print_hold(nv);
    cm_print_home(nv);
    cm_print_unit(nv);
    cm_print_coor(nv);
    cm_print_momo(nv);
    cm_print_plan(nv);
    cm_print_path(nv);
    cm_print_dist(nv);
    cm_print_frmo(nv);
    cm_print_tool(nv);

    return (STAT_OK);
}

/***********************************************************************************
 * AXIS JOGGING
 ***********************************************************************************/

float cm_get_jogging_dest(void)
{
    return cm.jogging_dest;
}

stat_t cm_run_jogx(nvObj_t *nv)
{
    set_flt(nv);
    cm_jogging_cycle_start(AXIS_X);
    return (STAT_OK);
}

stat_t cm_run_jogy(nvObj_t *nv)
{
    set_flt(nv);
    cm_jogging_cycle_start(AXIS_Y);
    return (STAT_OK);
}

stat_t cm_run_jogz(nvObj_t *nv)
{
    set_flt(nv);
    cm_jogging_cycle_start(AXIS_Z);
    return (STAT_OK);
}

stat_t cm_run_joga(nvObj_t *nv)
{
    set_flt(nv);
    cm_jogging_cycle_start(AXIS_A);
    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

/* model state print functions */

static const char fmt_vel[]  = "Velocity:%17.3f%s/min\n";
static const char fmt_feed[] = "Feed rate:%16.3f%s/min\n";
static const char fmt_line[] = "Line number:%10lu\n";
static const char fmt_stat[] = "Machine state:       %s\n"; // combined machine state
static const char fmt_macs[] = "Raw machine state:   %s\n"; // raw machine state
static const char fmt_cycs[] = "Cycle state:         %s\n";
static const char fmt_mots[] = "Motion state:        %s\n";
static const char fmt_hold[] = "Feedhold state:      %s\n";
static const char fmt_home[] = "Homing state:        %s\n";
static const char fmt_unit[] = "Units:               %s\n"; // units mode as ASCII string
static const char fmt_coor[] = "Coordinate system:   %s\n";
static const char fmt_momo[] = "Motion mode:         %s\n";
static const char fmt_plan[] = "Plane:               %s\n";
static const char fmt_path[] = "Path Mode:           %s\n";
static const char fmt_dist[] = "Distance mode:       %s\n";
static const char fmt_admo[] = "Arc Distance mode:   %s\n";
static const char fmt_frmo[] = "Feed rate mode:      %s\n";
static const char fmt_tool[] = "Tool number          %d\n";
static const char fmt_g92e[] = "G92 enabled          %d\n";

void cm_print_vel(nvObj_t *nv) { text_print_flt_units(nv, fmt_vel, GET_UNITS(ACTIVE_MODEL));}
void cm_print_feed(nvObj_t *nv) { text_print_flt_units(nv, fmt_feed, GET_UNITS(ACTIVE_MODEL));}
void cm_print_line(nvObj_t *nv) { text_print(nv, fmt_line);}     // TYPE_INT
void cm_print_tool(nvObj_t *nv) { text_print(nv, fmt_tool);}     // TYPE_INT
void cm_print_g92e(nvObj_t *nv) { text_print(nv, fmt_g92e);}     // TYPE_INT
void cm_print_stat(nvObj_t *nv) { text_print_str(nv, fmt_stat);} // print all these as TYPE_STRING
void cm_print_macs(nvObj_t *nv) { text_print_str(nv, fmt_macs);} // See _get_msg_helper() for details
void cm_print_cycs(nvObj_t *nv) { text_print_str(nv, fmt_cycs);}
void cm_print_mots(nvObj_t *nv) { text_print_str(nv, fmt_mots);}
void cm_print_hold(nvObj_t *nv) { text_print_str(nv, fmt_hold);}
void cm_print_home(nvObj_t *nv) { text_print_str(nv, fmt_home);}
void cm_print_unit(nvObj_t *nv) { text_print_str(nv, fmt_unit);}
void cm_print_coor(nvObj_t *nv) { text_print_str(nv, fmt_coor);}
void cm_print_momo(nvObj_t *nv) { text_print_str(nv, fmt_momo);}
void cm_print_plan(nvObj_t *nv) { text_print_str(nv, fmt_plan);}
void cm_print_path(nvObj_t *nv) { text_print_str(nv, fmt_path);}
void cm_print_dist(nvObj_t *nv) { text_print_str(nv, fmt_dist);}
void cm_print_admo(nvObj_t *nv) { text_print_str(nv, fmt_admo);}
void cm_print_frmo(nvObj_t *nv) { text_print_str(nv, fmt_frmo);}

static const char fmt_gpl[] = "[gpl] default gcode plane%10d [0=G17,1=G18,2=G19]\n";
static const char fmt_gun[] = "[gun] default gcode units mode%5d [0=G20,1=G21]\n";
static const char fmt_gco[] = "[gco] default gcode coord system%3d [1-6 (G54-G59)]\n";
static const char fmt_gpa[] = "[gpa] default gcode path control%3d [0=G61,1=G61.1,2=G64]\n";
static const char fmt_gdi[] = "[gdi] default gcode distance mode%2d [0=G90,1=G91]\n";

void cm_print_gpl(nvObj_t *nv) { text_print(nv, fmt_gpl);}  // TYPE_INT
void cm_print_gun(nvObj_t *nv) { text_print(nv, fmt_gun);}  // TYPE_INT
void cm_print_gco(nvObj_t *nv) { text_print(nv, fmt_gco);}  // TYPE_INT
void cm_print_gpa(nvObj_t *nv) { text_print(nv, fmt_gpa);}  // TYPE_INT
void cm_print_gdi(nvObj_t *nv) { text_print(nv, fmt_gdi);}  // TYPE_INT

/* system parameter print functions */

static const char fmt_jt[] = "[jt]  junction integrgation time%6.2f\n";
static const char fmt_ct[] = "[ct]  chordal tolerance%17.4f%s\n";
static const char fmt_sl[] = "[sl]  soft limit enable%12d [0=disable,1=enable]\n";
static const char fmt_lim[] ="[lim] limit switch enable%10d [0=disable,1=enable]\n";
static const char fmt_saf[] ="[saf] safety interlock enable%6d [0=disable,1=enable]\n";

void cm_print_jt(nvObj_t *nv) { text_print(nv, fmt_jt);}        // TYPE FLOAT
void cm_print_ct(nvObj_t *nv) { text_print_flt_units(nv, fmt_ct, GET_UNITS(ACTIVE_MODEL));}
void cm_print_sl(nvObj_t *nv) { text_print(nv, fmt_sl);}        // TYPE_INT
void cm_print_lim(nvObj_t *nv){ text_print(nv, fmt_lim);}       // TYPE_INT
void cm_print_saf(nvObj_t *nv){ text_print(nv, fmt_saf);}       // TYPE_INT

static const char fmt_m48e[] = "[m48e] overrides enabled%11d [0=disable,1=enable]\n";
static const char fmt_mfoe[] = "[mfoe] manual feed override enab%3d [0=disable,1=enable]\n";
static const char fmt_mfo[]  = "[mfo]  manual feedrate override%8.3f [0.05 < mfo < 2.00]\n";
static const char fmt_mtoe[] = "[mtoe] manual traverse over enab%3d [0=disable,1=enable]\n";
static const char fmt_mto[]  = "[mto]  manual traverse override%8.3f [0.05 < mto < 1.00]\n";
static const char fmt_tram[] = "[tram] is coordinate space rotated to be tram %s\n";
static const char fmt_nxln[] = "[nxln] the next line number expected is %10d\n";

void cm_print_m48e(nvObj_t *nv) { text_print(nv, fmt_m48e);}    // TYPE_INT
void cm_print_mfoe(nvObj_t *nv) { text_print(nv, fmt_mfoe);}    // TYPE INT
void cm_print_mfo(nvObj_t *nv)  { text_print(nv, fmt_mfo);}     // TYPE FLOAT
void cm_print_mtoe(nvObj_t *nv) { text_print(nv, fmt_mtoe);}    // TYPE INT
void cm_print_mto(nvObj_t *nv)  { text_print(nv, fmt_mto);}     // TYPE FLOAT
void cm_print_tram(nvObj_t *nv) { text_print(nv, fmt_tram);};   // TYPE BOOL
void cm_print_nxln(nvObj_t *nv) { text_print(nv, fmt_nxln);};   // TYPE INT

/*
 * axis print functions
 *
 *    _print_axis_ui8() - helper to print an integer value with no units
 *    _print_axis_flt() - helper to print a floating point linear value in prevailing units
 *    _print_pos_helper()
 *
 *    cm_print_am()
 *    cm_print_fr()
 *    cm_print_vm()
 *    cm_print_tm()
 *    cm_print_tn()
 *    cm_print_jm()
 *    cm_print_jh()
 *    cm_print_ra()
 *    cm_print_hi()
 *    cm_print_hd()
 *    cm_print_lv()
 *    cm_print_lb()
 *    cm_print_zb()
 *
 *    cm_print_pos() - print position with unit displays for MM or Inches
 *     cm_print_mpo() - print position with fixed unit display - always in Degrees or MM
 *     cm_print_tram() - print if the coordinate system is rotated
 */

static const char fmt_Xam[] = "[%s%s] %s axis mode%18d %s\n";
static const char fmt_Xfr[] = "[%s%s] %s feedrate maximum%11.0f%s/min\n";
static const char fmt_Xvm[] = "[%s%s] %s velocity maximum%11.0f%s/min\n";
static const char fmt_Xtm[] = "[%s%s] %s travel maximum%17.3f%s\n";
static const char fmt_Xtn[] = "[%s%s] %s travel minimum%17.3f%s\n";
static const char fmt_Xjm[] = "[%s%s] %s jerk maximum%15.0f%s/min^3 * 1 million\n";
static const char fmt_Xjh[] = "[%s%s] %s jerk homing%16.0f%s/min^3 * 1 million\n";
static const char fmt_Xra[] = "[%s%s] %s radius value%20.4f%s\n";
static const char fmt_Xsf[] = "[%s%s] %s spring offset factor%20.4f%s\n";
static const char fmt_Xsm[] = "[%s%s] %s spring offset max%20.4f%s\n";
static const char fmt_Xso[] = "[%s%s] %s spring offset%20.4f%s\n";
static const char fmt_Xhi[] = "[%s%s] %s homing input%15d [input 1-N or 0 to disable homing this axis]\n";
static const char fmt_Xhd[] = "[%s%s] %s homing direction%11d [0=search-to-negative, 1=search-to-positive]\n";
static const char fmt_Xsv[] = "[%s%s] %s search velocity%12.0f%s/min\n";
static const char fmt_Xlv[] = "[%s%s] %s latch velocity%13.2f%s/min\n";
static const char fmt_Xlb[] = "[%s%s] %s latch backoff%18.3f%s\n";
static const char fmt_Xzb[] = "[%s%s] %s zero backoff%19.3f%s\n";
static const char fmt_cofs[] = "[%s%s] %s %s offset%20.3f%s\n";
static const char fmt_cpos[] = "[%s%s] %s %s position%18.3f%s\n";

static const char fmt_pos[] = "%c position:%15.3f%s\n";
static const char fmt_mpo[] = "%c machine posn:%11.3f%s\n";
static const char fmt_ofs[] = "%c work offset:%12.3f%s\n";
static const char fmt_tof[] = "%c tool length offset:%12.3f%s\n";
static const char fmt_hom[] = "%c axis homing state:%2.0f\n";

static void _print_axis_ui8(nvObj_t *nv, const char *format)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, (uint8_t)nv->value);
    xio_writeline(cs.out_buf);
}

static void _print_axis_flt(nvObj_t *nv, const char *format)
{
    char *units;
    if (cm_get_axis_type(nv->index) == 0) {    // linear
        units = (char *)GET_UNITS(MODEL);
    } else {
        units = (char *)GET_TEXT_ITEM(msg_units, DEGREE_INDEX);
    }
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->value, units);
    xio_writeline(cs.out_buf);
}

static void _print_axis_coord_flt(nvObj_t *nv, const char *format)
{
    char *units;
    if (cm_get_axis_type(nv->index) == 0) {    // linear
        units = (char *)GET_UNITS(MODEL);
    } else {
        units = (char *)GET_TEXT_ITEM(msg_units, DEGREE_INDEX);
    }
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->token, nv->value, units);
    xio_writeline(cs.out_buf);
}

static void _print_pos(nvObj_t *nv, const char *format, uint8_t units)
{
    char axes[] = {"XYZABC"};
    uint8_t axis = _get_axis(nv->index);
    if (axis >= AXIS_A) { units = DEGREES;}
    sprintf(cs.out_buf, format, axes[axis], nv->value, GET_TEXT_ITEM(msg_units, units));
    xio_writeline(cs.out_buf);
}

static void _print_hom(nvObj_t *nv, const char *format)
{
    char axes[] = {"XYZABC"};
    uint8_t axis = _get_axis(nv->index);
    sprintf(cs.out_buf, format, axes[axis], nv->value);
    xio_writeline(cs.out_buf);
}

void cm_print_am(nvObj_t *nv)    // print axis mode with enumeration string
{
    sprintf(cs.out_buf, fmt_Xam, nv->group, nv->token, nv->group, (uint8_t)nv->value,
        GET_TEXT_ITEM(msg_am, (uint8_t)nv->value));
    xio_writeline(cs.out_buf);
}

void cm_print_fr(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xfr);}
void cm_print_vm(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xvm);}
void cm_print_tm(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xtm);}
void cm_print_tn(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xtn);}
void cm_print_jm(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xjm);}
void cm_print_jh(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xjh);}
void cm_print_ra(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xra);}

void cm_print_sf(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xsf);}
void cm_print_sm(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xsm);}
void cm_print_so(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xso);}

void cm_print_hi(nvObj_t *nv) { _print_axis_ui8(nv, fmt_Xhi);}
void cm_print_hd(nvObj_t *nv) { _print_axis_ui8(nv, fmt_Xhd);}
void cm_print_sv(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xsv);}
void cm_print_lv(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xlv);}
void cm_print_lb(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xlb);}
void cm_print_zb(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xzb);}

void cm_print_cofs(nvObj_t *nv) { _print_axis_coord_flt(nv, fmt_cofs);}
void cm_print_cpos(nvObj_t *nv) { _print_axis_coord_flt(nv, fmt_cpos);}

void cm_print_pos(nvObj_t *nv) { _print_pos(nv, fmt_pos, cm_get_units_mode(MODEL));}
void cm_print_mpo(nvObj_t *nv) { _print_pos(nv, fmt_mpo, MILLIMETERS);}
void cm_print_ofs(nvObj_t *nv) { _print_pos(nv, fmt_ofs, MILLIMETERS);}
void cm_print_tof(nvObj_t *nv) { _print_pos(nv, fmt_tof, MILLIMETERS);}
void cm_print_hom(nvObj_t *nv) { _print_hom(nv, fmt_hom);}

#endif // __TEXT_MODE
