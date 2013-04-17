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

#include "sam.h"

/* Sam hardware timers have three channels each. Each channel is actually an
 * independent timer, so we have a little nomenclature clash.
 *
 * Sam Timer != Motate::Timer!!!
 *
 * A Sam Timer CHANNEL is actually the portion that a Motate::Timer controls
 * direcly. Each SAM CHANNEL has two Motate:Timers (A and B).
 * (Actually, the Quadrature Decoder and Block Control can mix them up some,
 * but we ignore that.)
 * So, for the Sam, we have to maintain the same interface, and treat each
 * channel as an independent timer.
 */

namespace Motate {
	enum TimerMode {
		/* InputCapture mode (WAVE = 0) */
		kTimerInputCapture         = 0,
		/* InputCapture mode (WAVE = 0), counts up to RC */
		kTimerInputCaptureToMatch  = 0 | TC_CMR_CPCTRG,

		/* Waveform select, Up to 0xFFFFFFFF */
		kTimerUp            = TC_CMR_WAVE | TC_CMR_WAVSEL_UP,
		/* Waveform select, Up to RC */
		kTimerUpToMatch     = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC,
		/* Waveform select, Up to 0xFFFFFFFF, then Down */
		kTimerUpDown        = TC_CMR_WAVE | TC_CMR_WAVSEL_UPDOWN,
		/* Waveform select, Up to RC, then Down */
		kTimerUpDownToMatch = TC_CMR_WAVE | TC_CMR_WAVSEL_UPDOWN_RC,
	};
	
	enum TimerChannelOutputOptions {
		kOutputDisconnected = 0,
		kToggleOnMatch      = 1
		kClearOnMatch       = 2,
		kSetOnMatch         = 3,
	};

	enum TimerChannelInterruptOptions {
		kInterruptsOff       = 0,
		kInterruptOnMatch    = 1<<1;
		kInterruptOnOverflow = 1<<2,
		kSetOnMatch          = 1<<3, // ???
	};

	template <uint8_t timerNum>
	struct Timer {
		static Tc *tc;
		static TcChannel *tcChan;
		
		/* ################################################################### *
		/* #                          WARNING                                # *
		/* # WARNING: Sam channels (tcChan) DO NOT map to Motate Channels!?! # *
		/* #                          WARNING           (u been warned)      # *
		/* ################################################################### */
		
		Timer() : init() {};
				
		void init() {
			/* Unlock this thing */
			unlock();
		}
		
		void unlock() {
			tc->TC_WPMR = TC_WPMR_WPEN | TC_WPMR_WPKEY(0x54494D);
		}

		/* WHOA!! Only do this if you know what you're doing!! */
		void lock() {
			tc->TC_WPMR = TC_WPMR_WPEN | TC_WPMR_WPKEY(0x54494D);
		}
		
		bool setModeAndFrequency(const TimerMode mode, const uint32_t freq) {
			/* Prepare to be able to make changes: *?
			/*   Disable TC clock */
			tcChan->TC_CCR = TC_CCR_CLKDIS ;
			/*   Disable interrupts */
			tcChan->TC_IDR = 0xFFFFFFFF ;
			/*   Clear status register */
			tcChan->TC_SR ;

			/* Setup clock "prescaler" */
			/* Divisors: TC1: 2, TC2: 8, TC3: 32, TC4: 128, TC5: ???! */
			/* For now, we don't support TC5. */
			
			// Grab the SystemCoreClock value, in case it's volatile.
			uint32_t masterClock = SystemCoreClock;
			
			if (freq < ((masterClock / 128) / 0x10000) && freq > (masterClock / 128)) {

			}
			

			/*  Set mode */
			tcChan->TC_CMR = mode;
		};
		uint8_t getOutputValues(const uint8_t mask = 0xff) {
			// stub
			return 0;
		};
	};


}
#endif /* end of include guard: AVRTIMERS_H_ONCE */