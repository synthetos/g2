/*
  utility/AvrPins.hpp - Library for the Arduino-compatible Motate system
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

#ifndef AVRTIMERS_H_ONCE
#define AVRTIMERS_H_ONCE

#include <avr/io.h>
#include <util/atomic.h>

namespace Motate {
	enum TimerMode {
		kTimerUp           = 0,
		kTimerUpToMatch    = 1,
		kTimerUpDown       = 2,
		kTimerInputCapture = 3,
	};
	
	enum TimerChannelOutputOptions {
		kOutputDisconnected = 0,
		kToggleOnMatch      = 1
		kClearOnMatch       = 2,
		kSetOnMatch         = 3,
	};

	enum TimerChannelInterruptOptions {
		kInterruptsOff       = 0,
		kInterruptOnMatchA   = 1<<0,
		kInterruptOnMatchB   = 1<<1;
		kInterruptOnOverflow = 1<<2,
		kSetOnMatch          = 1<<3,
	};



}
#endif /* end of include guard: AVRTIMERS_H_ONCE */