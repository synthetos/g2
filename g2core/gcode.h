/*
 * gcode.h - rs274/ngc Gcode model and parser support
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef GCODE_H_ONCE
#define GCODE_H_ONCE

#include "hardware.h"

/**** Gcode-specific definitions ****/

typedef enum {                          // G Modal Group 1
    MOTION_MODE_STRAIGHT_TRAVERSE=0,    // G0 - straight traverse
    MOTION_MODE_STRAIGHT_FEED,          // G1 - straight feed
    MOTION_MODE_CW_ARC,                 // G2 - clockwise arc feed
    MOTION_MODE_CCW_ARC,                // G3 - counter-clockwise arc feed
    MOTION_MODE_CANCEL_MOTION_MODE,     // G80
    MOTION_MODE_STRAIGHT_PROBE,         // G38.2
    MOTION_MODE_CANNED_CYCLE_81,        // G81 - drilling
    MOTION_MODE_CANNED_CYCLE_82,        // G82 - drilling with dwell
    MOTION_MODE_CANNED_CYCLE_83,        // G83 - peck drilling
    MOTION_MODE_CANNED_CYCLE_84,        // G84 - right hand tapping
    MOTION_MODE_CANNED_CYCLE_85,        // G85 - boring, no dwell, feed out
    MOTION_MODE_CANNED_CYCLE_86,        // G86 - boring, spindle stop, rapid out
    MOTION_MODE_CANNED_CYCLE_87,        // G87 - back boring
    MOTION_MODE_CANNED_CYCLE_88,        // G88 - boring, spindle stop, manual out
    MOTION_MODE_CANNED_CYCLE_89         // G89 - boring, dwell, feed out
} cmMotionMode;

typedef enum {              // canonical plane - translates to:
                            //     axis_0  axis_1  axis_2
    CANON_PLANE_XY = 0,     // G17    X      Y      Z
    CANON_PLANE_XZ,         // G18    X      Z      Y
    CANON_PLANE_YZ          // G19    Y      Z      X
} cmCanonicalPlane;

typedef enum {
    INCHES = 0,             // G20
    MILLIMETERS,            // G21
    DEGREES                 // ABC axes (this value used for displays only)
} cmUnitsMode;

typedef enum {
    ABSOLUTE_COORDS = 0,    // machine coordinate system
    G54,                    // G54 coordinate system
    G55,                    // G55 coordinate system
    G56,                    // G56 coordinate system
    G57,                    // G57 coordinate system
    G58,                    // G58 coordinate system
    G59                     // G59 coordinate system
} cmCoordSystem;
#define COORD_SYSTEM_MAX G59 // set this manually to the last one

typedef enum {
    ABSOLUTE_OVERRIDE_OFF = 0,          // G53 disabled
    ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_OFFSETS,   // G53 enabled for movement, displays use current offsets
    ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_NO_OFFSETS // G53 enabled for movement, displays use no offset
} cmAbsoluteOverride;

typedef enum {              // G Modal Group 13
    PATH_EXACT_PATH = 0,    // G61 - hits corners but does not stop if it does not need to.
    PATH_EXACT_STOP,        // G61.1 - stops at all corners
    PATH_CONTINUOUS         // G64 and typically the default mode
} cmPathControl;

typedef enum {
    ABSOLUTE_DISTANCE_MODE = 0, // G90 / G90.1
    INCREMENTAL_DISTANCE_MODE   // G91 / G91.1
} cmDistanceMode;

typedef enum {
    INVERSE_TIME_MODE = 0,   // G93
    UNITS_PER_MINUTE_MODE,   // G94
    UNITS_PER_REVOLUTION_MODE// G95 (unimplemented)
} cmFeedRateMode;

typedef enum {
    ORIGIN_OFFSET_SET=0,    // G92 - set origin offsets
    ORIGIN_OFFSET_CANCEL,   // G92.1 - zero out origin offsets
    ORIGIN_OFFSET_SUSPEND,  // G92.2 - do not apply offsets, but preserve the values
    ORIGIN_OFFSET_RESUME    // G92.3 - resume application of the suspended offsets
} cmOriginOffset;

typedef enum {
    PROGRAM_STOP = 0,
    PROGRAM_END
} cmProgramFlow;

typedef enum {              // used for spindle and arc dir
    DIRECTION_CW = 0,
    DIRECTION_CCW
} cmDirection;

typedef enum {              // axis types. Enum must be in this order
    AXIS_TYPE_SYSTEM=-2,    // no axis, system parameter
    AXIS_TYPE_UNDEFINED=-1, // invalid type
    AXIS_TYPE_LINEAR,       // linear axis
    AXIS_TYPE_ROTARY        // rotary axis
} cmAxisType;

typedef enum {              // axis modes (ordered: see _cm_get_feed_time())
    AXIS_DISABLED = 0,      // kill axis
    AXIS_STANDARD,          // axis in coordinated motion w/standard behaviors
    AXIS_INHIBITED,         // axis is computed but not activated
    AXIS_RADIUS             // rotary axis calibrated to circumference
} cmAxisMode;
#define AXIS_MODE_LINEAR_MAX    AXIS_INHIBITED
#define AXIS_MODE_ROTARY_MAX    AXIS_RADIUS

/* Gcode state structures */

/****************************************************************************************
 * GCODE MODEL - The following GCodeModel/GCodeInput structs are used:
 *
 * - gm is the core Gcode model state. It keeps the internal gcode state model in 
 *   normalized canonical form. All values are unit converted (to mm) and in the 
 *   machine coordinate system (absolute coordinate system). Gm is owned by the 
 *   canonical machine layer and should be accessed only through cm_ routines.
 *
 *   The gm core struct is copied and passed as context to the runtime where it is used 
 *   for planning, move execution, feedholds, and reporting.
 *
 * - gmx is the extended gcode model variables that are only used by the canonical
 *   machine and do not need to be passed further down. It keeps "global" gcode state
 *   that does not change when you go down through the planner to the runtime. Other
 *   Gcode model state is kept in the singletons for various sub-systems, such as arcs
 *   spindle, coolant, and others (i.e. not ALL gcode global state is in gmx)
 *
 * - gn is used by the gcode interpreter and is re-initialized for each gcode block.
 *   It accepts data in the new gcode block in the formats present in the block 
 *   (pre-normalized forms). During initialization some state elements are necessarily 
 *    restored from gm.
 *
 * - gf is used by the gcode parser interpreter to hold flags for any data that has 
 *   changed in gn during the parse. gf.target[] values are also used by the canonical 
 *   machine during set_target().
 *
 * - cfg (config struct in config.h) is also used heavily and contains some values that
 *   might be considered to be Gcode model values. The distinction is that all values 
 *   in the config are persisted and restored, whereas the gm structs are transient. 
 *   So cfg has the G54 - G59 offsets, but gm has the G92 offsets. cfg has the power-on / 
 *   reset gcode default values, but gm has the operating state for the values 
 *   (which may have changed).
 */

typedef struct GCodeState {             // Gcode model state - used by model, planning and runtime
    int32_t linenum;                    // Gcode block line number
    cmMotionMode motion_mode;           // Group1: G0, G1, G2, G3, G38.2, G80, G81, G82
                                        //         G83, G84, G85, G86, G87, G88, G89

    float target[AXES];                 // XYZABC target where the move should go
    float target_comp[AXES];            // summation compensation (Kahan) overflow value
    float display_offset[AXES];         // work offsets from the machine coordinate system (for reporting only)

    float feed_rate;                    // F - normalized to millimeters/minute or in inverse time mode
    float P_word;                       // P - parameter used for dwell time in seconds, G10 coord select...

    cmFeedRateMode feed_rate_mode;      // See cmFeedRateMode for settings
    cmCanonicalPlane select_plane;      // G17,G18,G19 - values to set plane to
    cmUnitsMode units_mode;             // G20,G21 - 0=inches (G20), 1 = mm (G21)
    cmPathControl path_control;         // G61... EXACT_PATH, EXACT_STOP, CONTINUOUS
    cmDistanceMode distance_mode;       // G90=use absolute coords, G91=incremental movement
    cmDistanceMode arc_distance_mode;   // G90.1=use absolute IJK offsets, G91.1=incremental IJK offsets
    cmAbsoluteOverride absolute_override;// G53 TRUE = move using machine coordinates - this block only
    cmCoordSystem coord_system;         // G54-G59 - select coordinate system 1-9
    uint8_t tool;               // G    // M6 tool change - moves "tool_select" to "tool"
    uint8_t tool_select;        // G    // T value - T sets this value

    void reset() {
        linenum = 0;
        motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;

        for (uint8_t i = 0; i< AXES; i++) {
            target[i] = 0.0;
            display_offset[i] = 0.0;
        }

        feed_rate = 0.0;
        P_word = 0.0;

        feed_rate_mode = INVERSE_TIME_MODE;
        select_plane = CANON_PLANE_XY;
        units_mode = INCHES;
        path_control = PATH_EXACT_PATH;
        distance_mode = ABSOLUTE_DISTANCE_MODE;
        arc_distance_mode = ABSOLUTE_DISTANCE_MODE;
        absolute_override = ABSOLUTE_OVERRIDE_OFF;
        coord_system = ABSOLUTE_COORDS;
        tool = 0;
        tool_select = 0;

    };
} GCodeState_t;

typedef struct GCodeStateExtended {     // Gcode dynamic state extensions - used by model and arcs
    uint16_t magic_start;               // magic number to test memory integrity
    uint8_t next_action;                // handles G modal group 1 moves & non-modals
    uint8_t program_flow;               // used only by the gcode_parser
    int32_t last_line_number;           // used with line checksums

    float position[AXES];               // XYZABC model position (Note: not used in gn or gf)
    float g92_offset[AXES];             // XYZABC G92 offsets (aka origin offsets) (Note: not used in gn or gf)
    float g28_position[AXES];           // XYZABC stored machine position for G28
    float g30_position[AXES];           // XYZABC stored machine position for G30
    float p1_position[AXES];            // XYZABC stored machine position for return to p1 planner

    bool m48_enable;                    // master feedrate / spindle speed override enable
    bool mfo_enable;                    // feedrate override enable
    float mfo_factor;                   // 1.0000 x F feed rate. Go up or down from there
    bool mto_enable;                    // traverse override enable
    float mto_factor;                   // valid from 0.05 to 1.00

    bool g92_offset_enable;             // G92 offsets enabled/disabled.  0=disabled, 1=enabled
    bool block_delete_switch;           // set true to enable block deletes (true is default)

    uint16_t magic_end;
} GCodeStateX_t;

/*
 * Global Scope Functions
 */
void gcode_parser_init(void);
stat_t gcode_parser(char* block);
stat_t gc_get_gc(nvObj_t* nv);
stat_t gc_run_gc(nvObj_t* nv);

#endif  // End of include guard: GCODE_H_ONCE
