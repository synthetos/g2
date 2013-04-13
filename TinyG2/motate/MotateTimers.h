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

#ifndef MOTATETIMERS_H_ONCE
#define MOTATETIMERS_H_ONCE

#include <avr/io.h>
#include <util/atomic.h>
#include <inttypes.h>

namespace Motate {
	template <uint8_t timerNum>
	struct Timer {
		// This space intentionally left blank.
	};

	template<>
	struct Timer<0> {
		static const int8_t number = 0;
		
		enum TimeMode {
			Normal      = 0,
			PCPWM_FF    = 1,
			CTC         = 2,
			FastPWM_255 = 3,
			Reserved0   = 4,
			PCPWM_A     = 5,
			Reserved1   = 6,
			FastPWM_A   = 7,
		};
		
		enum PrescaleValues {
			TimerOff          = 0,
			NoPrescale        = 1,
			ClockBy_8         = 2,
			ClockBy_64        = 3,
			ClockBy_256       = 4,
			ClockBy_1024      = 5,
			ExternalT0Falling = 6,
			ExternalT0Rising  = 7,
		};
		
		enum InterruptValues {
			disabled = 0,
			overflow = 1,
			compareMatchA = 2,
			compareMatchB = 4,
		};
		
		struct modeType {
			Timer *parent;

			modeType(Timer *parentIn) : parent(parentIn) {};
			void set(Timer<0>::TimeMode x) {
				switch (x) {
					case Normal:
						TCCR0A &= ~(_BV(WGM00) | _BV(WGM01));
						TCCR0B &= ~_BV(WGM02);
						break;
					case PCPWM_FF:
						TCCR0A |=  _BV(WGM00);
						TCCR0A &= ~_BV(WGM01);
						TCCR0B &= ~_BV(WGM02);
						break;
					case CTC:
						TCCR0A &= ~_BV(WGM00);
						TCCR0A |=  _BV(WGM01);
						TCCR0B &= ~_BV(WGM02);
						break;
					case FastPWM_255:
						TCCR0A |=  _BV(WGM00) | _BV(WGM01);
						TCCR0B &= ~_BV(WGM02);
						break;
					case Reserved0:
						break;
					case PCPWM_A:
						TCCR0A |=  _BV(WGM00);
						TCCR0A &= ~_BV(WGM01);
						TCCR0B |=  _BV(WGM02);
						break;
					case Reserved1:
						break;
					case FastPWM_A:
						TCCR0A |=  _BV(WGM00) | _BV(WGM01);
						TCCR0B |=  _BV(WGM02);
						break;
				}
			};
			Timer<0>::TimeMode operator=(Timer<0>::TimeMode x) {
				set(x);
				return x;
			};
		    operator Timer<0>::TimeMode() {
				return static_cast<Timer<0>::TimeMode>(
					(uint8_t)(TCCR0A & (_BV(WGM00) | _BV(WGM01))) | (uint8_t)((TCCR0B & _BV(WGM02)) >> 1)
				);
			};
		};
		modeType mode;

		struct prescalerType {
			Timer *parent;

			prescalerType(Timer *parentIn) : parent(parentIn) {};
			void set(Timer<0>::PrescaleValues x) {
				TCCR0B = (TCCR0B & 0b11111000) | (uint8_t)x;
			};
			Timer<0>::PrescaleValues operator=(Timer<0>::PrescaleValues x) {
				set(x);
				return x;
			};
		    operator Timer<0>::PrescaleValues() {
				return static_cast<Timer<0>::PrescaleValues>(
					(uint8_t)(TCCR0B & 0b00000111)
				);
			};
		};
		prescalerType prescale;
		
		struct counterAType {
			Timer *parent;

			counterAType(Timer *parentIn) : parent(parentIn) {};
			void set(int8_t x) {
				OCR0A = x;
			};
			uint8_t operator=(uint8_t x) {
				set(x);
				return x;
			};
		    operator uint8_t() {
				return OCR0A;
			};
		};
		counterAType counterA;
		
		struct counterBType {
			Timer *parent;

			counterBType(Timer *parentIn) : parent(parentIn) {};
			void set(int8_t x) {
				OCR0B = x;
			};
			uint8_t operator=(uint8_t x) {
				set(x);
				return x;
			};
		    operator uint8_t() {
				return OCR0B;
			};
		};
		counterBType counterB;

		struct interruptsType {
			Timer *parent;

			interruptsType(Timer *parentIn) : parent(parentIn) {};
			void set(uint8_t x) {
				TIMSK0 = x;
			};

			uint8_t add(InterruptValues x) {
				TIMSK0 |= x;
			};

			uint8_t remove(InterruptValues x) {
				TIMSK0 &= ~x;
			};

			
			// Set it...
			InterruptValues operator=(InterruptValues x) {
				set((uint8_t)x);
				return x;
			};

			uint8_t operator=(uint8_t x) {
				set(x);
				return x;
			};
			
			// Add to it...
			uint8_t operator+=(InterruptValues x) {
				return add(x);
			};
			
			uint8_t operator-=(InterruptValues x) {
				return remove(x);
			};
			
		    operator uint8_t() {
				return TIMSK0 & 0b00000111;
			};
		};
		interruptsType interrupts;

		Timer<0>() : mode(this), prescale(this), counterA(this), counterB(this), interrupts(this) {};
	};
	typedef Timer<0> Timer0;
	static Timer<0> timer0;

} // namespace Motate
#endif /* end of include guard: MOTATETIMERS_H_ONCE */