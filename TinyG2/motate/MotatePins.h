/*
  MotatePins.hpp - Library for the Arduino-compatible Motate system
  http://tinkerin.gs/

  Copyright (c) 2013 Robert Giseburt

	This file is part of the Motate Library.

	This file ("the software") is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License, version 2 as published by the
	Free Software Foundation. You should have received a copy of the GNU General Public
	License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.

	As a special exception, you may use this file as part of a software library without
	restriction. Specifically, if other files instantiate templates or use macros or
	inline functions from this file, or you compile this file and link it with  other
	files to produce an executable, this file does not by itself cause the resulting
	executable to be covered by the GNU General Public License. This exception does not
	however invalidate any other reasons why the executable file might be covered by the
	GNU General Public License.

	THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
	WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
	SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
	OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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

#if defined(__SAM3X8E__) || defined(__SAM3X8C__)
#include <utility/SamPins.h>
#endif

#endif /* end of include guard: MOTATEPINS_H_ONCE */