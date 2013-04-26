/*
utility/SamTimers.hpp - Library for the Arduino-compatible Motate system
http://tinkerin.gs/

Copyright (c) 2012 Robert Giseburt

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

#ifndef SAMTIMERS_H_ONCE
#define SAMTIMERS_H_ONCE

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
		/* Waveform select, Up to TOP (RC) */
		kTimerUpToMatch     = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC,
		/* Waveform select, Up to 0xFFFFFFFF, then Down */
		kTimerUpDown        = TC_CMR_WAVE | TC_CMR_WAVSEL_UPDOWN,
		/* Waveform select, Up to TOP (RC), then Down */
		kTimerUpDownToMatch = TC_CMR_WAVE | TC_CMR_WAVSEL_UPDOWN_RC,
	};

	enum TimerChannelOutputOptions {
		kOutputDisconnected = 0,
		kToggleOnMatch      = 1,
		kClearOnMatch       = 2,
		kSetOnMatch         = 3,
	};

	enum TimerChannelInterruptOptions {
		kInterruptsOff       = 0,
		kInterruptOnMatchA   = 1<<1,
		kInterruptOnMatchB   = 1<<2,
		/* Interrupt on overflow could be a match C as well. */
		kInterruptOnOverflow = 1<<3,
	};

	enum TimerErrorCodes {
		kFrequencyUnattainable = -1,
	};

	template <uint8_t timerNum>
	struct Timer {

		// NOTE: Notice! The *pointers* are const, not the *values*.
		static Tc * const tc();
		static TcChannel * const tcChan();
		static const uint32_t peripheralId(); // ID_TC0 .. ID_TC8
		static const IRQn_Type tcIRQ();

		/********************************************************************
		**                          WARNING                                **
		** WARNING: Sam channels (tcChan) DO NOT map to Motate Channels!?! **
		**                          WARNING           (u been warned)      **
		*********************************************************************/

		Timer() { init(); };

		void init() {
			/* Unlock this thing */
			unlock();
		}

		void unlock() {
			tc()->TC_WPMR = TC_WPMR_WPKEY(0x54494D);
		}

		/* WHOA!! Only do this if you know what you're doing!! */
		void lock() {
			tc()->TC_WPMR = TC_WPMR_WPEN | TC_WPMR_WPKEY(0x54494D);
		}

		void enablePeripheralClock() {
			if (peripheralId() < 32) {
				uint32_t id_mask = 1u << ( peripheralId() );
				if ((PMC->PMC_PCSR0 & id_mask) != id_mask) {
					PMC->PMC_PCER0 = id_mask;
				}
			} else {
				uint32_t id_mask = 1u << ( peripheralId() - 32 );
				if ((PMC->PMC_PCSR1 & id_mask) != id_mask) {
					PMC->PMC_PCER1 = id_mask;
				}
			}
		};

		void disablePeripheralClock() {
			if (peripheralId() < 32) {
				uint32_t id_mask = 1u << ( peripheralId() );
				if ((PMC->PMC_PCSR0 & id_mask) == id_mask) {
					PMC->PMC_PCDR0 = id_mask;
				}
			} else {
				uint32_t id_mask = 1u << ( peripheralId() - 32 );
				if ((PMC->PMC_PCSR1 & id_mask) == id_mask) {
					PMC->PMC_PCDR1 = id_mask;
				}
			}
		};

		// Set the mode and frequency.
		// Returns: The actual frequency that was used, or kFrequencyUnattainable
		int32_t setModeAndFrequency(const TimerMode mode, const uint32_t freq) {
			/* Prepare to be able to make changes: */
			/*   Disable TC clock */
			tcChan()->TC_CCR = TC_CCR_CLKDIS ;
			/*   Disable interrupts */
			tcChan()->TC_IDR = 0xFFFFFFFF ;
			/*   Clear status register */
			tcChan()->TC_SR ;

			enablePeripheralClock();

			/* Setup clock "prescaler" */
			/* Divisors: TC1: 2, TC2: 8, TC3: 32, TC4: 128, TC5: ???! */
			/* For now, we don't support TC5. */

			// Grab the SystemCoreClock value, in case it's volatile.
			uint32_t masterClock = SystemCoreClock;

			// Store the divisor temporarily, to avoid looking it up again...
			uint32_t divisor = 2; // sane default of 2

			// TC1 = MCK/2
			if (freq > ((masterClock / 2) / 0x10000) && freq < (masterClock / 2)) {
				/*  Set mode */
				tcChan()->TC_CMR = mode | TC_CMR_TCCLKS_TIMER_CLOCK1;
				divisor = 2;

			// TC2 = MCK/8
			} else if (freq > ((masterClock / 8) / 0x10000) && freq < (masterClock / 8)) {
						/*  Set mode */
				tcChan()->TC_CMR = mode | TC_CMR_TCCLKS_TIMER_CLOCK2;
				divisor = 8;

			// TC3 = MCK/32
			} else if (freq > ((masterClock / 32) / 0x10000) && freq < (masterClock / 32)) {
						/*  Set mode */
				tcChan()->TC_CMR = mode | TC_CMR_TCCLKS_TIMER_CLOCK3;
				divisor = 32;

			// TC4 = MCK/128
			} else if (freq > ((masterClock / 128) / 0x10000) && freq < (masterClock / 128)) {
						/*  Set mode */
				tcChan()->TC_CMR = mode | TC_CMR_TCCLKS_TIMER_CLOCK4;
				divisor = 128;

			// Nothing fit! Hmm...
			} else {
				// PUNT! For now, just guess TC1.
				/*  Set mode */
				tcChan()->TC_CMR = mode | TC_CMR_TCCLKS_TIMER_CLOCK1;

				return kFrequencyUnattainable;
			}

			//TODO: Add ability to select external clocks... -RG

			// Extra mile, set the actual frequency, but only if we're going to RC.
			if (mode == kTimerInputCaptureToMatch
				|| mode == kTimerUpToMatch
			|| mode == kTimerUpDownToMatch) {

				int32_t newTop = masterClock/(divisor*freq);
				setTop(newTop);

				// Determine and return the new frequency.
				return masterClock/(divisor*newTop);
			}

			// Optimization -- we can't use RC for much when we're not using it,
			//  so, instead of looking up if we're using it or not, just set it to
			//  0xFFFF when we're not using it.
			setTop(0xFFFF);

			// Determine and return the new frequency.
			return masterClock/(divisor*0xFFFF);
		};

		// Set the TOP value for modes that use it.
		// WARNING: No sanity checking is done to verify that you are, indeed, in a mode that uses it.
		void setTop(const uint32_t topValue) {
			tcChan()->TC_RC = topValue;
		};

		// Here we want to get what the TOP value is. Is the mode is one that resets on RC, then RC is the TOP.
		// Otherwise, TOP is 0xFFFF. In order to see if TOP is RC, we need to look at the CPCTRG (RC Compare
		// Trigger Enable) bit of the CMR (Channel Mode Register). Note that this bit position is the same for
		// waveform or Capture mode, even though the Datasheet seems to obfuscate this fact.
		uint32_t getTopValue() {
			return tcChan()->TC_CMR & TC_CMR_CPCTRG ? tcChan()->TC_RC : 0xFFFF;
		};

		// Return the current value of the counter. This is a fleeting thing...
		uint32_t getValue() {
			return tcChan()->TC_CV;
		}

		void start() {
			tcChan()->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
		};

		void stop() {
			tcChan()->TC_CCR = TC_CCR_CLKDIS;
		};

		// Channel-specific functions. These are Motate channels, but they happen to line-up.
		// Motate channel A = Sam channel A.
		// Motate channel B = Sam channel B.

		// Specify the duty cycle as a value from 0.0 .. 1.0;
		void setDutyCycleA(const float ratio) {
			tcChan()->TC_RA = getTopValue() * ratio;
		};

		void setDutyCycleB(const float ratio) {
				tcChan()->TC_RB = getTopValue() * ratio;
		};

		// Specify channel A/B duty cycle as a integer value from 0 .. TOP.
		// TOP in this case is either RC_RC or 0xFFFF.
		void setDutyCycleA(const uint32_t absolute) {
			tcChan()->TC_RA = absolute;
		};

		void setDutyCycleB(const uint32_t absolute) {
			tcChan()->TC_RB = absolute;
		};

		void setInterrupts(const uint32_t interrupts) {
			if (interrupts != kInterruptsOff) {
				tcChan()->TC_IDR = 0xFFFFFFFF;
				NVIC_EnableIRQ(tcIRQ());

				if (interrupts | kInterruptOnOverflow) {
					// Check to see if we're overflowing on C. See getTopValue() description.
					if (tcChan()->TC_CMR & TC_CMR_CPCTRG) {
						tcChan()->TC_IER = TC_IER_CPCS; // RC Compare
					} else {
						tcChan()->TC_IER = TC_IER_COVFS; // Counter Overflow
					}
				}
				if (interrupts | kInterruptOnMatchA) {
					tcChan()->TC_IER = TC_IER_CPAS; // RA Compare
				}
				if (interrupts | kInterruptOnMatchB) {
					tcChan()->TC_IER = TC_IER_CPBS; // RB Compare
				}

			} else {
				tcChan()->TC_IDR = 0xFFFFFFFF;
				NVIC_DisableIRQ(tcIRQ());
			}
		}

		// Placeholder for user code.
		static void interrupt();
	};

	template<> Tc * const        Timer<0>::tc()           { return TC0; };
	template<> TcChannel * const Timer<0>::tcChan()       { return TC0->TC_CHANNEL + 0; };
	template<> const uint32_t    Timer<0>::peripheralId() { return ID_TC0; };
	template<> const IRQn_Type   Timer<0>::tcIRQ()        { return TC0_IRQn; };

	template<> Tc * const        Timer<1>::tc()           { return TC0; };
	template<> TcChannel * const Timer<1>::tcChan()       { return TC0->TC_CHANNEL + 1; };
	template<> const uint32_t    Timer<1>::peripheralId() { return ID_TC1; };
	template<> const IRQn_Type   Timer<1>::tcIRQ()        { return TC1_IRQn; };

	template<> Tc * const        Timer<2>::tc()           { return TC0; };
	template<> TcChannel * const Timer<2>::tcChan()       { return TC0->TC_CHANNEL + 2; };
	template<> const uint32_t    Timer<2>::peripheralId() { return ID_TC2; };
	template<> const IRQn_Type   Timer<2>::tcIRQ()        { return TC2_IRQn; };

	template<> Tc * const        Timer<3>::tc()           { return TC1; };
	template<> TcChannel * const Timer<3>::tcChan()       { return TC1->TC_CHANNEL + 0; };
	template<> const uint32_t    Timer<3>::peripheralId() { return ID_TC3; };
	template<> const IRQn_Type   Timer<3>::tcIRQ()        { return TC3_IRQn; };

	template<> Tc * const        Timer<4>::tc()           { return TC1; };
	template<> TcChannel * const Timer<4>::tcChan()       { return TC1->TC_CHANNEL + 1; };
	template<> const uint32_t    Timer<4>::peripheralId() { return ID_TC4; };
	template<> const IRQn_Type   Timer<4>::tcIRQ()        { return TC4_IRQn; };

	template<> Tc * const        Timer<5>::tc()           { return TC1; };
	template<> TcChannel * const Timer<5>::tcChan()       { return TC1->TC_CHANNEL + 2; };
	template<> const uint32_t    Timer<5>::peripheralId() { return ID_TC5; };
	template<> const IRQn_Type   Timer<5>::tcIRQ()        { return TC5_IRQn; };

	template<> Tc * const        Timer<6>::tc()           { return TC2; };
	template<> TcChannel * const Timer<6>::tcChan()       { return TC2->TC_CHANNEL + 0; };
	template<> const uint32_t    Timer<6>::peripheralId() { return ID_TC6; };
	template<> const IRQn_Type   Timer<6>::tcIRQ()        { return TC6_IRQn; };

	template<> Tc * const        Timer<7>::tc()           { return TC2; };
	template<> TcChannel * const Timer<7>::tcChan()       { return TC2->TC_CHANNEL + 1; };
	template<> const uint32_t    Timer<7>::peripheralId() { return ID_TC7; };
	template<> const IRQn_Type   Timer<7>::tcIRQ()        { return TC7_IRQn; };

	template<> Tc * const        Timer<8>::tc()           { return TC2; };
	template<> TcChannel * const Timer<8>::tcChan()       { return TC2->TC_CHANNEL + 2; };
	template<> const uint32_t    Timer<8>::peripheralId() { return ID_TC8; };
	template<> const IRQn_Type   Timer<8>::tcIRQ()        { return TC8_IRQn; };

} // namespace Motate

#define MOTATE_TIMER_INTERRUPT(number) extern "C" void TC ## number ## _Handler(void)

#endif /* end of include guard: SAMTIMERS_H_ONCE */