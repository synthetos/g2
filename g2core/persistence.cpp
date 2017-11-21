/*
 * persistence.cpp - persistence functions
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2017 Alden S. Hart Jr.
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
#include "g2core.h"
#include "persistence.h"
#include "canonical_machine.h"
#include "report.h"
//#include "util.h"

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

nvmSingleton_t nvm;

/***********************************************************************************
 **** GENERIC STATIC FUNCTIONS AND VARIABLES ***************************************
 ***********************************************************************************/


/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/

void persistence_init()
{
	return;
}

/*
 * read_persistent_value()	- return value (as float) by index
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 */

stat_t read_persistent_value(nvObj_t *nv)
{
	nv->value = 0;
	return (STAT_OK);
}

/*
 * write_persistent_value() - write to NVM by index, but only if the value has changed
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 *	Note: Removed NAN and INF checks on floats - not needed
 */

stat_t write_persistent_value(nvObj_t *nv)
{
//    if (cm.cycle_state != CYCLE_OFF) { // can't write when machine is moving
//        return(rpt_exception(STAT_FILE_NOT_OPEN, "write_persistent_value() can't write when machine is in cycle"));
//    }
	return (STAT_OK);
}
