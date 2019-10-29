/*
 * settings_othermill_pro.h - Other Machine Company Othermill Pro
 * This file is part of the TinyG project
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
/* Note: The values in this file are the default settings that are loaded
 *      into a virgin EEPROM, and can be changed using the config commands.
 *    After initial load the EEPROM values (or changed values) are used.
 *
 *    System and hardware settings that you shouldn't need to change
 *    are in hardware.h  Application settings that also shouldn't need
 *    to be changed are in tinyg.h
 */

#ifndef SETTINGS_OTHERMILL_PRO_H_ONCE
#define SETTINGS_OTHERMILL_PRO_H_ONCE

/***********************************************************************/
/**** OtherMill Pro profile ***************************************/
/***********************************************************************/

// default to same settings as standard Othermill, unless they are overridden in this file
#include "settings_othermill.h"

// interim max values for testing
#undef X_VELOCITY_MAX
#undef X_FEEDRATE_MAX
#undef Y_VELOCITY_MAX
#undef Y_FEEDRATE_MAX
#define X_VELOCITY_MAX 2600
#define X_FEEDRATE_MAX X_VELOCITY_MAX
#define Y_VELOCITY_MAX 2600
#define Y_FEEDRATE_MAX Y_VELOCITY_MAX

#undef P1_CW_SPEED_HI
#undef P1_CW_SPEED_LO
#undef P1_CW_PHASE_HI
#undef P1_CW_PHASE_LO
#define P1_CW_SPEED_HI 26000
#define P1_CW_SPEED_LO 8000
#define P1_CW_PHASE_HI 0.19
#define P1_CW_PHASE_LO 0.135

#undef PAUSE_DWELL_TIME
#define PAUSE_DWELL_TIME 4.0

#undef MOTOR_POWER_LEVEL_XY
#undef MOTOR_POWER_LEVEL_Z
#define MOTOR_POWER_LEVEL_XY 0.275
#define MOTOR_POWER_LEVEL_Z 0.375

#endif  // SETTINGS_OTHERMILL_PRO_H_ONCE
