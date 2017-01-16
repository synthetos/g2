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
#include "g2core.h"  // #1
#include "config.h"  // #2
#include "settings.h"

#if MARLIN_COMPAT_ENABLED == true

#include "controller.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "util.h"
#include "xio.h"            // for char definitions

// Structures used

// local helper functions and macros

/*
 * marlin_verify_checksum() - check to see if we have a line number (cheaply) and a valid checksum
 */

stat_t marlin_verify_checksum(char *str)
{
    if (*str != 'N') { return STAT_OK; } // we only check if we have a line number

    char checksum = 0;
    char c = *str++;
    while (c && (c != '*')) {
        checksum ^= c;
        c = *str++;
    }

    // c might be 0 here, in which case we didn't get a checksum and we return STAT_OK

    if ((c == '*') && strtol(str, NULL, 10) != checksum) {
        return STAT_CHECKSUM_MATCH_FAILED;
    }
    return STAT_OK;
}



/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

//stat_t gc_get_gc(nvObj_t *nv)
//{
//    ritorno(nv_copy_string(nv, cs.saved_buf));
//    nv->valuetype = TYPE_STRING;
//    return (STAT_OK);
//}
//
//stat_t gc_run_gc(nvObj_t *nv)
//{
//    return(gcode_parser(*nv->stringp));
//}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

// no text mode functions here. Move along

#endif // __TEXT_MODE

#endif // MARLIN_COMPAT_ENABLED == true
