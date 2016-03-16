/*
  utility/SamTimers.h - Library for the Arduino-compatible Motate system
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

#ifndef SAMTIMERS_H_ONCE
#define SAMTIMERS_H_ONCE

#include "sam.h"
#include "SamCommon.h"

/* Sam hardware has two types of timer: "Timers" and "PWMTimers"
 *
 * Timers:
 *
 * Sam hardware timers have three channels each. Each channel is actually an
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
 *
 *
 * PWMTimers:
 * 
 * For compatibility and transparency with Timers, we use the same TimerModes,
 * even through they actually use bitmaps for Timer registers.
 *
 * Timers have more modes than PWM Timers, and more interrupts.
 * We return kInvalidMode for the ones that don't map, except that we treat "Up"
 * and "UpToMatch" both as "LeftAligned," and "UpDown" and "UpDownToMatch"
 * as "CenterAligned."
 *
 * Consequently, you can use kPWMLeftAligned and kPWMCenterAligned as valid modes
 * on a Timer.
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
		/* For PWM, we'll alias kTimerUpToMatch as: */
		kPWMLeftAligned     = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC,
		/* Waveform select, Up to 0xFFFFFFFF, then Down */
		kTimerUpDown        = TC_CMR_WAVE | TC_CMR_WAVSEL_UPDOWN,
		/* Waveform select, Up to TOP (RC), then Down */
		kTimerUpDownToMatch = TC_CMR_WAVE | TC_CMR_WAVSEL_UPDOWN_RC,
		/* For PWM, we'll alias kTimerUpDownToMatch as: */
		kPWMCenterAligned     = kTimerUpDownToMatch,
	};

	/* We're trading acronyms for verbose CamelCase. Dubious. */
	enum TimerChannelOutputOptions {
		kOutputDisconnected = 0,

		/* ACPA (TC_CMR) RA Compare Effect on TIOA */
		kToggleAOnCompareA  = TC_CMR_ACPA_TOGGLE,
		kClearAOnCompareA   = TC_CMR_ACPA_CLEAR,
		kSetAOnCompareA     = TC_CMR_ACPA_SET,

		/* BCPB (TC_CMR) RB Compare Effect on TIOB */
		/* See note below about TC_CMR_EEVT_XC0 */
		kToggleBOnCompareB  = TC_CMR_BCPB_TOGGLE | TC_CMR_EEVT_XC0,
		kClearBOnCompareB   = TC_CMR_BCPB_CLEAR | TC_CMR_EEVT_XC0,
		kSetBOnCompareB     = TC_CMR_BCPB_SET | TC_CMR_EEVT_XC0,

		/* We use "match" the same as the mode -- RC Compare */
		/* ACPC (TC_CMR) RC Compare Effect on TIOA */
		kToggleAOnMatch     = TC_CMR_ACPC_TOGGLE,
		kClearAOnMatch      = TC_CMR_ACPC_CLEAR,
		kSetAOnMatch        = TC_CMR_ACPC_SET,

		/* BCPC (TC_CMR) RC Compare Effect on TIOB */
		kToggleBOnMatch     = TC_CMR_BCPC_TOGGLE | TC_CMR_EEVT_XC0,
		kClearBOnMatch      = TC_CMR_BCPC_CLEAR | TC_CMR_EEVT_XC0,
		kSetBOnMatch        = TC_CMR_BCPC_SET | TC_CMR_EEVT_XC0,


		/* Aliases for use with PWM */
		kPWMOnA             = kClearAOnCompareA | kSetAOnMatch,
		kPWMOnAInverted     = kSetAOnCompareA | kClearAOnMatch,

		kPWMOnB             = kClearBOnCompareB | kSetBOnMatch,
		kPWMOnBInverted     = kSetBOnCompareB | kClearBOnMatch
};

	/* We use TC_CMR_EEVT_XC0 in the above to allow TIOB to be an output.
	 * The defualt is for it to be the input for ExternalEvent.
	 * By setting it to XC0, we allow it to be an output.
	 */

	enum TimerChannelInterruptOptions {
		kInterruptsOff              = 0,
        /* Alias for "off" to make more sense
            when returned from setInterruptPending(). */
		kInterruptUnknown           = 0,
		
        kInterruptOnMatchA          = 1<<1,
		kInterruptOnMatchB          = 1<<2,
		/* Note: Interrupt on overflow could be a match C as well. */
		kInterruptOnOverflow        = 1<<3,

		/* This turns the IRQ on, but doesn't set the timer to ever trigger it. */
		kInterruptOnSoftwareTrigger = 1<<4,

		/* Set priority levels here as well: */
		kInterruptPriorityHighest   = 1<<5,
		kInterruptPriorityHigh      = 1<<6,
		kInterruptPriorityMedium    = 1<<7,
		kInterruptPriorityLow       = 1<<8,
		kInterruptPriorityLowest    = 1<<9,
	};

	enum TimerErrorCodes {
		kFrequencyUnattainable = -1,
		kInvalidMode = -2,
	};

	enum PWMTimerClockOptions {
		kPWMClockPrescalerOnly = 0,
		kPWMClockPrescaleAndDivA = 1,
		kPWMClockPrescaleAndDivB = 2,
	};

	typedef const uint8_t timer_number;
    
	template <uint8_t timerNum>
	struct Timer : SamCommon< Timer<timerNum> > {

		// NOTE: Notice! The *pointers* are const, not the *values*.
		static Tc * const tc();
		static TcChannel * const tcChan();
		static const uint32_t peripheralId(); // ID_TC0 .. ID_TC8
		static const IRQn_Type tcIRQ();
        
        typedef SamCommon< Timer<timerNum> > common;
        
		/********************************************************************
		**                          WARNING                                **
		** WARNING: Sam channels (tcChan) DO NOT map to Motate Channels!?! **
		**                          WARNING           (u been warned)      **
		*********************************************************************/

		Timer() { init(); };
		Timer(const TimerMode mode, const uint32_t freq) {
			init();
			setModeAndFrequency(mode, freq);
		};

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

		// Set the mode and frequency.
		// Returns: The actual frequency that was used, or kFrequencyUnattainable
		// freq is not const since we may "change" it
		int32_t setModeAndFrequency(const TimerMode mode, uint32_t freq) {
			/* Prepare to be able to make changes: */
			/*   Disable TC clock */
			tcChan()->TC_CCR = TC_CCR_CLKDIS ;
			/*   Disable interrupts */
			tcChan()->TC_IDR = 0xFFFFFFFF ;
			/*   Clear status register */
			tcChan()->TC_SR;

            common::enablePeripheralClock();

			if (mode == kTimerUpDownToMatch || mode == kTimerUpDown)
				freq /= 2;

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
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_WAVSEL_Msk | TC_CMR_TCCLKS_Msk)) |
					mode | TC_CMR_TCCLKS_TIMER_CLOCK1;
				divisor = 2;

			// TC2 = MCK/8
			} else if (freq > ((masterClock / 8) / 0x10000) && freq < (masterClock / 8)) {
						/*  Set mode */
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_WAVSEL_Msk | TC_CMR_TCCLKS_Msk)) |
					mode | TC_CMR_TCCLKS_TIMER_CLOCK2;
				divisor = 8;

			// TC3 = MCK/32
			} else if (freq > ((masterClock / 32) / 0x10000) && freq < (masterClock / 32)) {
						/*  Set mode */
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_WAVSEL_Msk | TC_CMR_TCCLKS_Msk)) |
					mode | TC_CMR_TCCLKS_TIMER_CLOCK3;
				divisor = 32;

			// TC4 = MCK/128
			} else if (freq > ((masterClock / 128) / 0x10000) && freq < (masterClock / 128)) {
						/*  Set mode */
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_WAVSEL_Msk | TC_CMR_TCCLKS_Msk)) |
					mode | TC_CMR_TCCLKS_TIMER_CLOCK4;
				divisor = 128;

			// Nothing fit! Hmm...
			} else {
				// PUNT! For now, just guess TC1.
				/*  Set mode */
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_WAVSEL_Msk | TC_CMR_TCCLKS_Msk)) |
					mode | TC_CMR_TCCLKS_TIMER_CLOCK1;

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

		void stopOnMatch() {
			tcChan()->TC_CMR = TC_CMR_CPCSTOP;
		};

		// Channel-specific functions. These are Motate channels, but they happen to line-up.
		// Motate channel A = Sam channel A.
		// Motate channel B = Sam channel B.

		// Specify the duty cycle as a value from 0.0 .. 1.0;
		void setDutyCycleA(const float ratio) {
			setExactDutyCycleA(getTopValue() * ratio);
		};

		void setDutyCycleB(const float ratio) {
			setExactDutyCycleB(getTopValue() * ratio);
		};

		// Specify channel A/B duty cycle as a integer value from 0 .. TOP.
		// TOP in this case is either RC_RC or 0xFFFF.
		void setExactDutyCycleA(const uint32_t absolute) {
			tcChan()->TC_RA = absolute;
		};

		void setExactDutyCycleB(const uint32_t absolute) {
			tcChan()->TC_RB = absolute;
		};

		void setOutputOptions(const uint32_t options) {

			// Note that we carefully crafted the TimerChannelOutputOptions
			// to match the bits in CMR, so we just mask and set!
			tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(
													 TC_CMR_ACPA_Msk |
													 TC_CMR_BCPB_Msk |
													 TC_CMR_ACPC_Msk |
													 TC_CMR_BCPC_Msk |
													 TC_CMR_EEVT_XC0
													)
								) | options;
		};

		// Use this to leave the OutputB options alone.
		void setOutputAOptions(const uint32_t options) {

			// Note that we carefully crafted the TimerChannelOutputOptions
			// to match the bits in CMR, so we just mask and set!
			tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(
													 TC_CMR_ACPA_Msk |
													 TC_CMR_ACPC_Msk
													 )
								) | options;
		};
		// Use this to leave the OutputA options alone.
		void setOutputBOptions(const uint32_t options) {
			// Note that we carefully crafted the TimerChannelOutputOptions
			// to match the bits in CMR, so we just mask and set!
			tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(
													 TC_CMR_BCPB_Msk |
													 TC_CMR_BCPC_Msk |
													 TC_CMR_EEVT_XC0
													 )
								) | options;
		};

		// These are special function for stopping output waveforms.
		// This is defferent from stopping the timer, which kill both channels and
		// all interrupts. This simply stops the pin output from changing, and is used
		// to set the duty cycle to 0.

		// ASSUMPTION: The pin is not in Toggle mode.
		void stopPWMOutputA() {
			if ((tcChan()->TC_CMR & TC_CMR_ACPA_Msk) == kSetAOnCompareA)
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_ACPC_Msk)) | kSetAOnMatch;
			else
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_ACPC_Msk)) | kClearAOnMatch;
		};

		void stopPWMOutputB() {
			if ((tcChan()->TC_CMR & TC_CMR_BCPB_Msk) == kSetBOnCompareB)
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_BCPC_Msk)) | kSetBOnMatch;
			else
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_BCPC_Msk)) | kClearBOnMatch;
		};

		// These two start the waveform. We try to be as fast as we can.
		// ASSUMPTION: We stopped it with the corresponding function.
		// ASSUMPTION: The pin is not and was not in Toggle mode.
		void startPWMOutputA() {
			if ((tcChan()->TC_CMR & TC_CMR_ACPA_Msk) == kSetAOnCompareA)
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_ACPC_Msk)) | kClearAOnMatch;
			else
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_ACPC_Msk)) | kSetAOnMatch;
		};

		void startPWMOutputB() {
			if ((tcChan()->TC_CMR & TC_CMR_BCPB_Msk) == kSetBOnCompareB)
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_BCPC_Msk)) | kClearBOnMatch;
			else
				tcChan()->TC_CMR = (tcChan()->TC_CMR & ~(TC_CMR_BCPC_Msk)) | kSetBOnMatch;
		};


		void setInterrupts(const uint32_t interrupts) {
			if (interrupts != kInterruptsOff) {
				tcChan()->TC_IDR = 0xFFFFFFFF;

				if (interrupts & kInterruptOnOverflow) {
					// Check to see if we're overflowing on C. See getTopValue() description.
					if (tcChan()->TC_CMR & TC_CMR_CPCTRG) {
						tcChan()->TC_IER = TC_IER_CPCS; // RC Compare
					} else {
						tcChan()->TC_IER = TC_IER_COVFS; // Counter Overflow
					}
				}
				if (interrupts & kInterruptOnMatchA) {
					tcChan()->TC_IER = TC_IER_CPAS; // RA Compare
				}
				if (interrupts & kInterruptOnMatchB) {
					tcChan()->TC_IER = TC_IER_CPBS; // RB Compare
				}

				/* Set interrupt priority */
				if (interrupts & kInterruptPriorityHighest) {
					NVIC_SetPriority(tcIRQ(), 0);
				}
				else if (interrupts & kInterruptPriorityHigh) {
					NVIC_SetPriority(tcIRQ(), 3);
				}
				else if (interrupts & kInterruptPriorityMedium) {
					NVIC_SetPriority(tcIRQ(), 7);
				}
				else if (interrupts & kInterruptPriorityLow) {
					NVIC_SetPriority(tcIRQ(), 11);
				}
				else if (interrupts & kInterruptPriorityLowest) {
					NVIC_SetPriority(tcIRQ(), 15);
				}

				NVIC_EnableIRQ(tcIRQ());
			} else {
				tcChan()->TC_IDR = 0xFFFFFFFF;
				NVIC_DisableIRQ(tcIRQ());
			}
		}
        
		void setInterruptPending() {
			NVIC_SetPendingIRQ(tcIRQ());
		}

/* Here for reference (most we don't use):
 TC_SR_COVFS   (TC_SR) Counter Overflow Status
 TC_SR_LOVRS   (TC_SR) Load Overrun Status
 TC_SR_CPAS    (TC_SR) RA Compare Status
 TC_SR_CPBS    (TC_SR) RB Compare Status
 TC_SR_CPCS    (TC_SR) RC Compare Status
 TC_SR_LDRAS   (TC_SR) RA Loading Status
 TC_SR_LDRBS   (TC_SR) RB Loading Status
 TC_SR_ETRGS   (TC_SR) External Trigger Status
 TC_SR_CLKSTA  (TC_SR) Clock Enabling Status
 TC_SR_MTIOA   (TC_SR) TIOA Mirror
 TC_SR_MTIOB   (TC_SR) TIOB Mirror
*/

		TimerChannelInterruptOptions getInterruptCause() {
			uint32_t sr = tcChan()->TC_SR;
					// if it is either an overflow or a RC compare
			if (sr & (TC_SR_COVFS | TC_SR_CPCS)) {
				return kInterruptOnOverflow;
			}
			else if (sr & (TC_SR_CPAS)) {
				return kInterruptOnMatchA;
			}
			else if (sr & (TC_SR_CPBS)) {
				return kInterruptOnMatchB;
			}
			else if (sr & (TC_SR_ETRGS)) {
				return kInterruptOnMatchA;
			}
			return kInterruptUnknown;
		}

		// Placeholder for user code.
		static void interrupt() __attribute__ ((weak));
	};


	template <uint8_t timerNum>
	struct PWMTimer {

		// NOTE: Notice! The *pointers* are const, not the *values*.
		static Pwm * const pwm();
		static PwmCh_num * const pwmChan();
		static const uint32_t peripheralId(); // ID_TC0 .. ID_TC8
		static const IRQn_Type pwmIRQ();

		/********************************************************************
		 **                          WARNING                                **
		 ** WARNING: Sam channels (tcChan) DO NOT map to Motate Channels!?! **
		 **                          WARNING           (u been warned)      **
		 *********************************************************************/

		PWMTimer() { init(); };
		PWMTimer(const TimerMode mode, const uint32_t freq) {
			init();
			setModeAndFrequency(mode, freq);
		};

		void init() {
			/* Unlock this thing */
			unlock();
		}

#ifndef YOU_REALLY_WANT_PWM_LOCK_AND_UNLOCK
#define YOU_REALLY_WANT_PWM_LOCK_AND_UNLOCK 0
#endif

#if YOU_REALLY_WANT_PWM_LOCK_AND_UNLOCK
		// You probably don't....
		void unlock() {
			pwm()->PWM_WPCR = PWM_WPCR_WPKEY(0x50574D /* "PWM" */);
		}

		/* WHOA!! Only do this if you know what you're doing!! */
		void lock() {
			// This locks EVERYTHING!!!
			pwm()->PWM_WPCR = PWM_WPCR_WPCMD(1) | PWM_WPCR_WPRG0 | PWM_WPCR_WPRG1 | PWM_WPCR_WPRG2 | PWM_WPCR_WPRG3 | PWM_WPCR_WPRG4 | PWM_WPCR_WPRG5 | TC_WPMR_WPKEY(0x54494D);
		}
#else
		// Non-ops to keep the compiler happy.
		void unlock() {};
		void lock() {};
#endif

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

		/* Set the mode and frequency.
		 * Returns: The actual frequency that was used, or kFrequencyUnattainable
		 * freq is not const since we may "change" it.
		 * PWM module can optionally use one of two additional prescale multipliers:
		 *     A (kPWMClockPrescaleAndDivA) or B (kPWMClockPrescaleAndDivB).
		 * However, these are shared clocks used by all PWM channels.
		 * Only use these on timers that will have drastically different periods.
		 * There is currently no way to set multiple times to the same Clock A or B.
		 */
		int32_t setModeAndFrequency(const TimerMode mode, uint32_t frequency, const uint8_t clock = kPWMClockPrescalerOnly) {
			/* Prepare to be able to make changes: */
			/*   Disable TC clock */
			pwm()->PWM_DIS = 1 << timerNum ;
			/*   Disable interrupts */
			pwm()->PWM_IDR1 = 0xFFFFFFFF ;
			pwm()->PWM_IDR2	= 0xFFFFFFFF ;

			enablePeripheralClock();

			if (mode == kTimerInputCapture || mode == kTimerInputCaptureToMatch)
				return kFrequencyUnattainable;

			// Remember: kTimerUpDownToMatch and kPWMCenterAligned are identical.
			if (mode == kPWMCenterAligned)
				frequency /= 2;

			/* Setup clock "prescaler" */
			/* Divisors: 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 */

			// Grab the SystemCoreClock value, in case it's volatile.
			uint32_t masterClock = SystemCoreClock;

			// Store the divisor temporarily, to avoid looking it up again...
			uint32_t divisors[11] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
			uint8_t divisor_index = 0;
			uint32_t prescaler;

			if (clock == kPWMClockPrescaleAndDivA || clock == kPWMClockPrescaleAndDivB) {
				// ** THIS CLOCK A/CLOCK B CODE ALL NEEDS CHECKED ** //


				// Find prescaler value
				prescaler = (masterClock / divisors[divisor_index]) / frequency;
				while ((prescaler > 255) && (divisor_index < 11)) {
					divisor_index++;
					prescaler = (masterClock / divisors[divisor_index]) / frequency;
				}

				
				// Set the actual frequency, if we found a match.
				if (clock == kPWMClockPrescaleAndDivA) {
					// Setup divisor A

					PWM->PWM_CLK = (PWM->PWM_CLK & ~PWM_CLK_DIVA_Msk) | prescaler | (divisors[divisor_index] << 8);

					int32_t newTop = masterClock/(divisors[divisor_index] * prescaler * frequency);
					setTop(newTop, /*setOnNext=*/false);

					// Determine and return the new frequency.
					return masterClock/(divisors[divisor_index]*newTop);
				}
				else { // if clock == kPWMClockPrescaleAndDivB
					   // SAME THING, BUT B
				}
			}

			// if clock == kPWMClockPrescalerOnly

			// Find prescaler value
			uint32_t test_value = masterClock / divisors[divisor_index];

			// We assume if (divisor_index == 10) then 10 will be the value we use...
			// We want OUT of the while loop when we have the right divisor.
			// AGAIN: FAILING this test means we have the RIGHT divisor.
			while ((divisor_index < 10) && ((frequency > test_value) || (frequency < (test_value / 0x10000)))) {
				divisor_index++;
				test_value = masterClock / divisors[divisor_index];
			}

			pwmChan()->PWM_CMR = (divisor_index & 0xff) | (mode == kPWMCenterAligned ? PWM_CMR_CALG : 0) |
              /* Preserve inversion: */(pwmChan()->PWM_CMR & PWM_CMR_CPOL);

			// ToDo: Polarity setttings, Dead-Time control, Counter events

			int32_t newTop = test_value / frequency;
			setTop(newTop, /*setOnNext=*/ false);

			// Determine and return the new frequency.
			return test_value * newTop;
		};

		// Set the TOP value for modes that use it.
		// WARNING: No sanity checking is done to verify that you are, indeed, in a mode that uses it.
		void setTop(const uint32_t topValue, bool setOnNext = true) {
			if (setOnNext)
				pwmChan()->PWM_CPRDUPD = topValue;
			else
				pwmChan()->PWM_CPRD = topValue;
		};

		// Here we want to get what the TOP value is.
		uint32_t getTopValue() {
			return pwmChan()->PWM_CPRD;
		};

		// Return the current value of the counter. This is a fleeting thing...
		uint32_t getValue() {
			return pwmChan()->PWM_CCNT;
		}

		void start() {
			pwm()->PWM_ENA = 1 << timerNum;
		};

		void stop() {
			pwm()->PWM_DIS = 1 << timerNum;
		};

		// Channel-specific functions. These are Motate channels, but they happen to line-up.

		// Specify the duty cycle as a value from 0.0 .. 1.0;
		void setDutyCycleA(const float ratio, bool setOnNext = true) {
			if (setOnNext)
				pwmChan()->PWM_CDTYUPD = getTopValue() * ratio;
			else
				pwmChan()->PWM_CDTY = getTopValue() * ratio;

		};

		// Specify channel A/B duty cycle as a integer value from 0 .. TOP.
		// TOP in this case is either RC_RC or 0xFFFF.
		void setExactDutyCycleA(const uint32_t absolute, bool setOnNext = true) {
			if (setOnNext)
				pwmChan()->PWM_CDTYUPD = absolute;
			else
				pwmChan()->PWM_CDTY = absolute;

		};

		void setOutputOptions(const uint32_t options) {
			setOutputAOptions(options);
		};

		void setOutputAOptions(const uint32_t options) {
			if (options == kPWMOnAInverted) {
				pwmChan()->PWM_CMR |= PWM_CMR_CPOL;
			}
			else if (options == kPWMOnA) {
				pwmChan()->PWM_CMR &= ~PWM_CMR_CPOL;
			}
		};


		// ASSUMPTION: The pin is not in Toggle mode.
		void stopPWMOutputA() {
//			stop();
		};

		// These two start the waveform. We try to be as fast as we can.
		// ASSUMPTION: We stopped it with the corresponding function.
		// ASSUMPTION: The pin is not and was not in Toggle mode.
		void startPWMOutputA() {
//			start();
		};

		void setInterrupts(const uint32_t interrupts) {
#if 0
			// TODO
			if (interrupts != kInterruptsOff) {
				pwmChan()->TC_IDR = 0xFFFFFFFF;
				NVIC_EnableIRQ(pwmIRQ());

				if (interrupts & kInterruptOnOverflow) {
					// Check to see if we're overflowing on C. See getTopValue() description.
					if (pwmChan()->TC_CMR & TC_CMR_CPCTRG) {
						pwmChan()->TC_IER = TC_IER_CPCS; // RC Compare
					} else {
						pwmChan()->TC_IER = TC_IER_COVFS; // Counter Overflow
					}
				}
				if (interrupts & kInterruptOnMatchA) {
					pwmChan()->TC_IER = TC_IER_CPAS; // RA Compare
				}
				if (interrupts & kInterruptOnMatchB) {
					pwmChan()->TC_IER = TC_IER_CPBS; // RB Compare
				}

				/* Set interrupt priority */
				if (interrupts & kInterruptPriorityHighest) {
					NVIC_SetPriority(pwmIRQ(), 0);
				}
				else if (interrupts & kInterruptPriorityHigh) {
					NVIC_SetPriority(pwmIRQ(), 3);
				}
				else if (interrupts & kInterruptPriorityMedium) {
					NVIC_SetPriority(pwmIRQ(), 7);
				}
				else if (interrupts & kInterruptPriorityLow) {
					NVIC_SetPriority(pwmIRQ(), 11);
				}
				else if (interrupts & kInterruptPriorityLowest) {
					NVIC_SetPriority(pwmIRQ(), 15);
				}

			} else {
				pwmChan()->TC_IDR = 0xFFFFFFFF;
				NVIC_DisableIRQ(pwmIRQ());
			}
#endif
		}

		void setInterruptPending() {
			NVIC_SetPendingIRQ(pwmIRQ());
		}

		TimerChannelInterruptOptions getInterruptCause() {
#if 0
			uint32_t sr = pwmChan()->TC_SR;
			// if it is either an overflow or a RC compare
			if (sr & (TC_SR_COVFS | TC_SR_CPCS)) {
				return kInterruptOnOverflow;
			}
			else if (sr & (TC_SR_CPAS)) {
				return kInterruptOnMatchA;
			}
			else if (sr & (TC_SR_CPBS)) {
				return kInterruptOnMatchB;
			}
			else if (sr & (TC_SR_ETRGS)) {
				return kInterruptOnMatchA;
			}
#endif
			return kInterruptUnknown;
		}
		
		// Placeholder for user code.
		static void interrupt() __attribute__ ((weak));
	};

	static const timer_number SysTickTimerNum = 0xFF;
	template <>
	struct Timer<SysTickTimerNum> {
		static volatile uint32_t _motateTickCount;
		
		/********************************************************************
		 **                          WARNING                                **
		 ** WARNING: Sam channels (tcChan) DO NOT map to Motate Channels!?! **
		 **                          WARNING           (u been warned)      **
		 *********************************************************************/

		Timer() { init(); };
		Timer(const TimerMode mode, const uint32_t freq) {
			init();
//			setModeAndFrequency(mode, freq);
		};

		void init() {
			_motateTickCount = 0;

			// Set Systick to 1ms interval, common to all SAM3 variants
			if (SysTick_Config(SystemCoreClock / 1000))
			{
				// Capture error
				while (true);
			}
		};

		// Set the mode and frequency.
		// Should we offer this one? -RG
//		int32_t setModeAndFrequency(const TimerMode mode, uint32_t freq) {};

		// Return the current value of the counter. This is a fleeting thing...
		uint32_t getValue() {
			return _motateTickCount;
		};

		void _increment() {
			_motateTickCount++;
		};

		// Placeholder for user code.
		static void interrupt() __attribute__ ((weak));
	};
	extern Timer<SysTickTimerNum> SysTickTimer;

	// Provide a Arduino-compatible blocking-delay function
	inline void delay( uint32_t microseconds )
	{
		uint32_t doneTime = SysTickTimer.getValue() + microseconds;

		do
		{} while ( SysTickTimer.getValue() < doneTime );
	}

    struct Timeout {
        uint32_t start_, delay_;
        Timeout() : start_ {0}, delay_ {0} {};

        bool isSet() {
            return (start_ > 0);
        }

        bool isPast() {
            if (!isSet()) {
                return false;
            }
            return ((SysTickTimer.getValue() - start_) > delay_);
        };

        void set(uint32_t delay) {
            start_ = SysTickTimer.getValue();
            delay_ = delay;
        };

        void clear() {
            start_ = 0;
            delay_ = 0;
        }
    };

} // namespace Motate

#define MOTATE_TIMER_INTERRUPT(number) template<> void Timer<number>::interrupt()

/** THIS IS OLD INFO, AND NO LONGER RELEVANT TO THIS PROJECT, BUT IT WAS HARD TO COME BY: **/

/*****
 Ok, here we get ugly: We need the *mangled* names for the specialized interrupt functions,
 so that we can use weak references from C functions TCn_Handler to the C++ Timer<n>::interrupt(),
 so that we get clean linkage to user-provided functions, and no errors if those functions don't exist.
 
 So, to get the mangled names (which will only for for GCC, btw), I do this in a bash shell (ignore any errors after the g++ line):
 
 cat <<END >> temp.cpp
 #include <inttypes.h>
 namespace Motate {
 template <uint8_t timerNum>
 struct Timer {
 static void interrupt();
 };
 template<> void Timer<0>::interrupt() {};
 template<> void Timer<1>::interrupt() {};
 template<> void Timer<2>::interrupt() {};
 template<> void Timer<3>::interrupt() {};
 template<> void Timer<4>::interrupt() {};
 template<> void Timer<5>::interrupt() {};
 template<> void Timer<6>::interrupt() {};
 template<> void Timer<7>::interrupt() {};
 template<> void Timer<8>::interrupt() {};
 }
 END
 arm-none-eabi-g++ temp.cpp -o temp.o -mthumb -nostartfiles -mcpu=cortex-m3
 arm-none-eabi-nm temp.o | grep Motate
 rm temp.o temp.cpp
 
 
 You should get output like this:
 
 00008000 T _ZN6Motate5TimerILh0EE9interruptEv
 0000800c T _ZN6Motate5TimerILh1EE9interruptEv
 00008018 T _ZN6Motate5TimerILh2EE9interruptEv
 00008024 T _ZN6Motate5TimerILh3EE9interruptEv
 00008030 T _ZN6Motate5TimerILh4EE9interruptEv
 0000803c T _ZN6Motate5TimerILh5EE9interruptEv
 00008048 T _ZN6Motate5TimerILh6EE9interruptEv
 00008054 T _ZN6Motate5TimerILh7EE9interruptEv
 00008060 T _ZN6Motate5TimerILh8EE9interruptEv
 
 Ignore the hex number and T at the beginning, and the rest is the mangled names you need for below.
 I broke the string into three parts to clearly show the part that is changing.
 */


#endif /* end of include guard: SAMTIMERS_H_ONCE */