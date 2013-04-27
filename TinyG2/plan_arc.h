/*
 * plan_arc.h - arc planning and motion execution
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
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

#ifndef PLAN_ARC_H_ONCE
#define PLAN_ARC_H_ONCE 

#ifdef __cplusplus
extern "C"{
#endif

// See planner.h for MM_PER_ARC_SEGMENT setting

typedef struct arArcSingleton {			// persistent planner and runtime variables
	double magic_start;
	uint8_t run_state;			// runtime state machine sequence
	uint32_t linenum;			// line number of the arc feed move (Nxxxxx)
	uint32_t lineindex;			// line index of the arc feed move (autoincrement)
	
	double endpoint[AXES];		// endpoint position
	double position[AXES];		// accumulating runtime position
	double target[AXES];		// runtime target position
	double work_offset[AXES];	// offset from machine coord system for reporting

	double length;				// length of line or helix in mm
	double time;				// total running time (derived)
	double min_time;			// not sure this is needed
	double theta;				// total angle specified by arc
	double radius;				// computed via offsets
	double angular_travel;		// travel along the arc
	double linear_travel;		// travel along linear axis of arc
	uint8_t axis_1;				// arc plane axis
	uint8_t axis_2;				// arc plane axis
	uint8_t axis_linear;		// transverse axis (helical)

	double segments;			// number of segments in arc or blend
	int32_t segment_count;		// count of running segments
	double segment_time;		// constant time per aline segment
	double segment_theta;		// angular motion per segment
	double segment_linear_travel;// linear motion per segment
	double center_1;			// center of circle at axis 1 (typ X)
	double center_2;			// center of circle at axis 2 (typ Y)
	double magic_end;
} arc_t;
arc_t ar;

// function prototypes
uint8_t ar_arc(	const double target[],
				const double i, const double j, const double k, 
				const double theta, 
				const double radius, 
		   		const double angular_travel, 
				const double linear_travel, 
		   		const uint8_t axis_1, 
				const uint8_t axis_2, 
				const uint8_t axis_linear,
				const double minutes,
				const double work_offset[],
				const double min_time);

uint8_t ar_arc_callback(void);
void ar_abort_arc(void);

#endif	// End of include guard: PLAN_ARC_H_ONCE
