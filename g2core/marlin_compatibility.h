/*
 * marlin_compatibility.cpp - support for marlin protocol and gcode
 * This file is part of the g2core project
 *
 * Copyright (c) 2017 Alden S. Hart, Jr.
 * Copyright (c) 2017 Rob Giseburt
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

#ifndef MARLIN_COMPAT_H_ONCE
#define MARLIN_COMPAT_H_ONCE

#include "g2core.h"  // #1
#include "config.h"  // #2

/*
 * Global Scope Functions
 */

// gcode parsing and fack stk500v2
stat_t marlin_verify_checksum(char *str);
bool marlin_handle_fake_stk500(char *str);

// gcode handling
stat_t marlin_start_tramming_bed(); //G29
stat_t marlin_request_temperature_report(); // M105
stat_t marlin_request_position_report();    // M114
stat_t marlin_set_temperature(uint8_t tool, float temperature, bool wait); // M104, M109, M140, M190

// response handler (primarily just prints "ok")
void marlin_response(const stat_t status, char *buf);

// controller loop callback
stat_t marlin_callback();

stat_t cm_marlin_set_extruder_mode(const uint8_t mode); // M82, M82
stat_t marlin_set_fan_speed(const uint8_t fan, float speed); // M106, M107

stat_t marlin_disable_motors(); // M84

stat_t marlin_report_version(); // M115
#endif  // End of include guard: MARLIN_COMPAT_H_ONCE
