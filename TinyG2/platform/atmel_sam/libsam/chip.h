/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _LIB_SAM_
#define _LIB_SAM_

/*
 * Core and peripherals registers definitions
 */
#include "sam.h"

// ASH: ++++ The following modifies the SAM3XA_SERIES #define from the sam.h file to fix compilation problems in adc.h
// Ref: http://asf.atmel.no/docs/latest/common.services.calendar.example2.stk600-rcuc3d/html/group__sam__part__macros__group.html
//#define SAM3XA_SERIES (SAM3A4 || SAM3A8)	// original define in sam.h file
#undef SAM3XA_SERIES
#define SAM3XA_SERIES (SAM3X4 || SAM3X8 || SAM3A4 || SAM3A8)

// ASH: ++++ Added as this is note defined anywhere else
#define USB_PID USB_PID_DUE		

/**** TO HERE ****/

/* Define attribute */
#if defined (  __GNUC__  ) /* GCC CS3 */
    #define WEAK __attribute__ ((weak))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
    #define WEAK __weak
#endif

/* Define NO_INIT attribute */
#if defined (  __GNUC__  )
    #define NO_INIT
#elif defined ( __ICCARM__ )
    #define NO_INIT __no_init
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/*
 * Peripherals
 */
#include "include/adc.h"
#if (SAM3XA_SERIES) || (SAM3N_SERIES) || (SAM3S_SERIES)
#include "include/dacc.h"
#endif // (SAM3XA_SERIES) || (SAM3N_SERIES) || (SAM3S_SERIES)

#include "include/interrupt_sam_nvic.h"
#include "include/efc.h"
#include "include/gpbr.h"
#include "include/pio.h"
#include "include/pmc.h"
#include "include/pwmc.h"
#include "include/rstc.h"
#include "include/rtc.h"
#include "include/rtt.h"
#include "include/spi.h"
#include "include/ssc.h"
#include "include/tc.h"
#include "include/twi.h"
#include "include/usart.h"
#include "include/wdt.h"

#include "include/timetick.h"
#include "include/USB_device.h"
#include "include/USB_host.h"

#if (SAM3XA_SERIES)
#include "include/can.h"
#include "include/emac.h"
#include "include/trng.h"
#include "include/uotghs_device.h"
#include "include/uotghs_host.h"
#endif /* (SAM3XA_SERIES) */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* _LIB_SAM_ */
