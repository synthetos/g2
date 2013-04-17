/*
  MotateTimers.hpp - Library for the Arduino-compatible Motate system
  http://tinkerin.gs/

  Copyright (c) 2012 Robert Giseburt

	This file is part of the Motate Library.

	The Motate Library is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The Motate Library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with the Motate Library.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MOTATETIMERS_H_ONCE
#define MOTATETIMERS_H_ONCE

#include <inttypes.h>

/************************************************
 *
 * Timers are not quite as time-critical as pins, so we can relax (a *little*)
 * on optimizing the hell out of them.
 * 
 * The exception being the interrupts, which MUST be bare-metal speed.
 * 
 ************************************************/

/* Timers have a simple pattern: Timer, which contains one or more Channels.
 * Timers control the Mode (Up, Up-Down, Capture) and Frequency, and the Channels control
 * the ouput (pin change, interrupt) as well as the duty cycle.
 * Some processors share a master Timer among many Channels (AVR, XMega), where others
 * have completely independent timers (Atmel Sam ARM). In the latter case, we still honor the
 * Timer -> Channel relationship int he API, except it's simply a one-to-one relationship.
 */


namespace Motate {	
} // namespace Motate

/****************************************
	These defines allow masking of *some* (non-neccessary) functionality that is
	not available on all architectures:
		MOTATE_AVR_COMPATIBILITY -- only present functionality that is also on the AVR architecture
		MOTATE_AVRX_COMPATIBILITY -- only present functionality that is also on the AVR XMEGA architecture
		MOTATE_SAM_COMPATIBILITY -- only present functionality that is also on the SAM architecture
****************************************/

#ifdef __AVR_XMEGA__

#include <utility/AvrXTimers.h>

#else

#ifdef __AVR__
#include <utility/AvrTimers.h>
#endif

#endif

#ifdef __SAM3X8E__
#include <utility/SamTimers.h>
#endif


#endif /* end of include guard: MOTATETIMERS_H_ONCE */