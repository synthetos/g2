/*
  MotatePins.hpp - Library for the Arduino-compatible Motate system
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

#ifndef MOTATEPINS_H_ONCE
#define MOTATEPINS_H_ONCE

#include <inttypes.h>

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

#include <utility/AvrXPins.h>

#else

#ifdef __AVR__
#include <utility/AvrPins.h>
#endif

#endif

#ifdef __SAM3X8E__
#include <utility/SamPins.h>
#endif

#endif /* end of include guard: MOTATEPINS_H_ONCE */