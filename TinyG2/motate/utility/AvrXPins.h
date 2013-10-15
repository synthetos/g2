/*
  utility/AvrXPins.h - Library for the Motate system
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

#ifndef AVRPINS_H_ONCE
#define AVRPINS_H_ONCE

#include <avr/io.h>
#include <util/atomic.h>

#include "../MotateTWI/MotateStaticAssert.h"

namespace Motate {
	enum PinMode {
		kUnchanged         = 0,
		kOutput            = 1,
		kInput             = 2,
	};
	
	// Note that these are out of order for the same of the compatibility masking.
	// The numbering corresponds to the PINnCTRL bit values on this architecture only.
	enum PinOptions {
		kNormal            = 0,
		kTotem             = 0, // alias
		kPullUp            = 3,
// Mask off functionality that the SAM and AVR don't have
#if !defined(MOTATE_AVR_COMPATIBILITY) && !defined(MOTATE_SAM_COMPATIBILITY)
		kBusKeeper         = 1,
		kPullDown          = 2,
		kWiredOr           = 4,
		kDriveHighOnly     = 4, // alias
		kWiredOrPull       = 6,
		kDriveHighPullDown = 6, // alias
#endif // !MOTATE_AVR_COMPATIBILITY && !MOTATE_SAM_COMPATIBILITY
// Mask off functionality that the AVR doesn't have
// NOTE: The SAM does, but it uses a different mechanism to control it
#if !defined(MOTATE_AVR_COMPATIBILITY)
		kWiredAnd          = 5,
		kDriveLowOnly      = 5, // alias
		kWiredAndPull      = 7,
		kDriveLowPullUp    = 7, // alias
#endif // !MOTATE_AVR_COMPATIBILITY
	};
	
	template <unsigned char portLetter>
	struct Port8 {
		typedef uint8_t uintPort_t;
		static const uint8_t letter = (uint8_t) portLetter;
		
		void setModes(const uintPort_t value, const uintPort_t mask = 0xff) {
			// stub
		};
		void setOptions(const PinOptions options, const uintPort_t mask) {
			// stub
		};
		void getModes() {
			// stub
		};
		void getOptions() {
			// stub
		};
		void set(const uintPort_t value) {
			// stub
		};
		void clear(const uintPort_t value) {
			// stub
		};
		void write(const uintPort_t value) {
			// stub
		};
		void write(const uintPort_t value, const uintPort_t mask) {
			// stub
		};
		uintPort_t getInputValues(const uintPort_t mask = 0xff) {
			// stub
			return 0;
		};
		uintPort_t getOutputValues(const uintPort_t mask = 0xff) {
			// stub
			return 0;
		};
	};
	
	template<int8_t pinNum>
	struct Pin {
		static const int8_t number = -1;
		static const uint8_t portLetter = 0;
		static const uint8_t mask = 0;
		static uint8_t maskForPort(const uint8_t otherPortLetter) {
			return 0x00;
		};
		bool isNull() { return true; };
	};

	template<int8_t pinNum>
	struct InputPin : Pin<pinNum> {
		InputPin() : Pin<pinNum>(kInput) {};
		InputPin(const PinOptions options) : Pin<pinNum>(kInput, options) {};
		void init(const PinOptions options = kNormal  ) {Pin<pinNum>::init(kInput, options);};
		uint8_t get() {
			return Pin<pinNum>::getInputValue();
		};
		/*Override these to pick up new methods */
		operator bool() { return (get() != 0); };
	private: /* Make these private to catch them early. These are intentionally not defined. */
		void init(const PinMode type, const PinOptions options = kNormal);
		void operator=(const bool value) { Pin<pinNum>::write(value); };
		void write(const bool);
	};

	template<int8_t pinNum>
	struct OutputPin : Pin<pinNum> {
		OutputPin() : Pin<pinNum>(kOutput) {};
		OutputPin(const PinOptions options) : Pin<pinNum>(kOutput, options) {};
		void init(const PinOptions options = kNormal) {Pin<pinNum>::init(kOutput, options);};
		uint8_t get() {
			return Pin<pinNum>::getOutputValue();
		};
		void operator=(const bool value) { Pin<pinNum>::write(value); };
		/*Override these to pick up new methods */
		operator bool() { return (get() != 0); };
	private: /* Make these private to catch them early. */
		void init(const PinMode type, const PinOptions options = kNormal); /* Intentially not defined. */
	};	
	
	typedef const int8_t pin_number;
	
	#define _MAKE_MOTATE_PIN(pinNum, registerLetter, registerChar, registerPin)\
		template<>\
		struct Pin<pinNum> {\
		private: /* Lock the copy contructor.*/\
			Pin(const Pin<pinNum>&){};\
		public:\
			static const int8_t number = pinNum;\
			static const uint8_t portLetter = (uint8_t) registerChar;\
			static const uint8_t mask = (1 << registerPin);\
				\
			Pin(){};\
			Pin(const PinMode type, const PinOptions options = kNormal) {\
				init(type, options);\
			};\
			void operator=(const bool value) { write(value); };\
			operator bool() { return (get() != 0); };\
		\
			void init(const PinMode type, const PinOptions options = kNormal) {\
				setMode(type);\
				setOptions(options);\
			};\
			void setMode(const PinMode type) {\
				switch (type) {\
					case kOutput:\
						(PORT ## registerLetter).DIR |= mask;\
						break;\
					case kInput:\
						(PORT ## registerLetter).DIR &= ~mask;\
						break;\
					default:\
						break;\
				}\
			};\
			PinMode getMode() {\
				return ((PORT ## registerLetter).DIR |= mask) ? kOutput : kInput;\
			};\
			void setOptions(const PinOptions options) {\
				switch (options) {\
					case kNormal:\
					/*case kTotem:*/\
						(PORT ## registerLetter).PIN ## registerPin ## CTRL = PORT_OPC_TOTEM_gc;\
						break;\
#ifdef MOTATE_AVR_COMPATIBILITY
					case kBusKeeper:\
						(PORT ## registerLetter).PIN ## registerPin ## CTRL = PORT_OPC_BUSKEEPER_gc;\
						break;\
					case kPullDown:\
						(PORT ## registerLetter).PIN ## registerPin ## CTRL = PORT_OPC_PULLDOWN_gc;\
						break;\
#endif // MOTATE_AVR_COMPATIBILITY
					case kPullUp:\
						(PORT ## registerLetter).PIN ## registerPin ## CTRL = PORT_OPC_PULLUP_gc;\
						break;\
#ifdef MOTATE_AVR_COMPATIBILITY
					case kWiredOr:\
						(PORT ## registerLetter).PIN ## registerPin ## CTRL = PORT_OPC_WIREDOR_gc;\
						break;\
					case kWiredAnd:\
						(PORT ## registerLetter).PIN ## registerPin ## CTRL = PORT_OPC_WIREDAND_gc;\
						break;\
					case kWiredOrPull:\
						(PORT ## registerLetter).PIN ## registerPin ## CTRL = PORT_OPC_WIREDORPULL_gc;\
						break;\
					case kWiredAndPull:\
						(PORT ## registerLetter).PIN ## registerPin ## CTRL = PORT_OPC_WIREDANDPULL_gc;\
						break;\
#endif // MOTATE_AVR_COMPATIBILITY
					default:\
						break;\
				}\
			};\
			PinOptions getOptions() {\
				return static_cast<PinOptions>((PORT ## registerLetter).PIN ## registerPin ## CTRL);\
			};\
			void set() {\
				(PORT ## registerLetter).OUTSET = mask;\
			};\
			void clear() {\
				(PORT ## registerLetter).OUTCLR = mask;\
			};\
			void write(bool value) {\
				if (!value)\
					clear();\
				else\
					set();\
			};\
			void toggle()  {\
				(PORT ## registerLetter).OUTTGL = mask;\
			};\
			uint8_t get() { /* WARNING: This will fail if the input buffer is disabled for this pin!!! Use getOutputValue() instead. */\
				return ((PORT ## registerLetter).IN & mask);\
			};\
			uint8_t getInputValue() {\
				return ((PORT ## registerLetter).IN & mask);\
			};\
			uint8_t getOutputValue() {\
				return ((PORT ## registerLetter).OUT & mask);\
			};\
			bool isNull() { return false; };\
			static uint8_t maskForPort(const uint8_t otherPortLetter) {\
				return portLetter == otherPortLetter ? mask : 0x00;\
			};\
		};\
		typedef Pin<pinNum> Pin ## pinNum;\
		static Pin<pinNum> pin ## pinNum(kOutput);\
		typedef InputPin<pinNum> InputPin ## pinNum;\
		typedef OutputPin<pinNum> OutputPin ## pinNum;


	#define _MAKE_MOTATE_PORT8(registerLetter, registerChar)\
	template <> inline void Port8<registerChar>::setModes(const uintPort_t value, const uintPort_t mask) {\
		uint8_t port_value = 0;\
		if (mask != 0xff) {\
			port_value = (PORT ## registerLetter).DIR & mask;\
		}\
		(PORT ## registerLetter).DIR = port_value | value;\
	};\
	template <> inline void Port8<registerChar>::setOptions(const PinOptions options, const uintPort_t mask) {\
		PORTCFG.MPCMASK = mask; /*Write the configuration to all the masked pins at once.*/\
		/* MPCMASK is automatically cleared after any PINnCTRL write completes.*/\
		switch (options) {\
			case kNormal:\
			/*case kTotem:*/\
				(PORT ## registerLetter).PIN0CTRL = PORT_OPC_TOTEM_gc;\
				break;\
#ifdef MOTATE_AVR_COMPATIBILITY
			case kBusKeeper:\
				(PORT ## registerLetter).PIN0CTRL = PORT_OPC_BUSKEEPER_gc;\
				break;\
			case kPullDown:\
				(PORT ## registerLetter).PIN0CTRL = PORT_OPC_PULLDOWN_gc;\
				break;\
#endif // MOTATE_AVR_COMPATIBILITY
			case kPullUp:\
				(PORT ## registerLetter).PIN0CTRL = PORT_OPC_PULLUP_gc;\
				break;\
#ifdef MOTATE_AVR_COMPATIBILITY
			case kWiredOr:\
				(PORT ## registerLetter).PIN0CTRL = PORT_OPC_WIREDOR_gc;\
				break;\
			case kWiredAnd:\
				(PORT ## registerLetter).PIN0CTRL = PORT_OPC_WIREDAND_gc;\
				break;\
			case kWiredOrPull:\
				(PORT ## registerLetter).PIN0CTRL = PORT_OPC_WIREDORPULL_gc;\
				break;\
			case kWiredAndPull:\
				(PORT ## registerLetter).PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;\
				break;\
#endif // MOTATE_AVR_COMPATIBILITY
			default:\
				break;\
		}\
	};\
	template <> inline void Port8<registerChar>::set(const uintPort_t value) {\
		(PORT ## registerLetter).OUTSET = value;\
	};\
	template <> inline void Port8<registerChar>::clear(const uintPort_t value) {\
		(PORT ## registerLetter).OUTCLR = value;\
	};\
	template <> inline void Port8<registerChar>::write(const uintPort_t value) {\
		(PORT ## registerLetter).OUT = value;\
	};\
	template <> inline void Port8<registerChar>::write(const uintPort_t value, const uintPort_t mask) {\
		uintPort_t port_value = 0;\
		if (mask != 0xff) {\
			port_value = (PORT ## registerLetter).OUT & mask;\
		}\
		(PORT ## registerLetter).OUT = port_value | value;\
	};\
	template <> inline uintPort_t Port8<registerChar>::getInputValues(const uintPort_t mask) {\
		return (PORT ## registerLetter).IN & (mask);\
	}\
	template <> inline uintPort_t Port8<registerChar>::getOutputValues(const uintPort_t mask) {\
		return (PORT ## registerLetter).OUT & (mask);\
	}\
	typedef Port8<registerChar> Port ## registerLetter;\
	static Port ## registerLetter port ## registerLetter;

	typedef Pin<-1> NullPin;
	static NullPin nullPin;
	
	// TODO: Support other, non-atxmega192a3 XMegas

	_MAKE_MOTATE_PORT8(A ,'A');
	_MAKE_MOTATE_PORT8(B ,'B');
	_MAKE_MOTATE_PORT8(C ,'C');
	_MAKE_MOTATE_PORT8(D ,'D');
	_MAKE_MOTATE_PORT8(E ,'E');
	_MAKE_MOTATE_PORT8(F ,'F');
	
	#include <motate_pin_assignments.h>

	// PinHolder - virtual ports
	template<uint8_t PinBit7num, uint8_t PinBit6num, uint8_t PinBit5num = -1, uint8_t PinBit4num = -1, uint8_t PinBit3num = -1, uint8_t PinBit2num = -1, uint8_t PinBit1num = -1, uint8_t PinBit0num = -1>
	class PinHolder8 {

		static Pin<PinBit7num> PinBit7;
		static Pin<PinBit6num> PinBit6;
		static Pin<PinBit5num> PinBit5;
		static Pin<PinBit4num> PinBit4;
		static Pin<PinBit3num> PinBit3;
		static Pin<PinBit2num> PinBit2;
		static Pin<PinBit1num> PinBit1;
		static Pin<PinBit0num> PinBit0;

#define _MOTATE_CREATE_CLEAR_AND_COPY_MASKS(aPortLetter) \
		static const uint8_t port ## aPortLetter ## ClearMask =\
			(Pin<PinBit7num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit7num>::mask : 0x00) |\
			(Pin<PinBit6num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit6num>::mask : 0x00) |\
			(Pin<PinBit5num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit5num>::mask : 0x00) |\
			(Pin<PinBit4num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit4num>::mask : 0x00) |\
			(Pin<PinBit3num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit3num>::mask : 0x00) |\
			(Pin<PinBit2num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit2num>::mask : 0x00) |\
			(Pin<PinBit1num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit1num>::mask : 0x00) |\
			(Pin<PinBit0num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit0num>::mask : 0x00);\
\
		static const uint8_t port ## aPortLetter ## CopyMask =\
			(Pin<PinBit7num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit7num>::mask == (1 << 7 ) \
				? Pin<PinBit7num>::mask : 0x00) |\
			(Pin<PinBit6num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit6num>::mask == (1 << 6 )\
				? Pin<PinBit6num>::mask : 0x00) |\
			(Pin<PinBit5num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit5num>::mask == (1 << 5 )\
				? Pin<PinBit5num>::mask : 0x00) |\
			(Pin<PinBit4num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit4num>::mask == (1 << 4 )\
				? Pin<PinBit4num>::mask : 0x00) |\
			(Pin<PinBit3num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit3num>::mask == (1 << 3 )\
				? Pin<PinBit3num>::mask : 0x00) |\
			(Pin<PinBit2num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit2num>::mask == (1 << 2 )\
				? Pin<PinBit2num>::mask : 0x00) |\
			(Pin<PinBit1num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit1num>::mask == (1 << 1 )\
				? Pin<PinBit1num>::mask : 0x00) |\
			(Pin<PinBit0num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit0num>::mask == (1 << 0 )\
				? Pin<PinBit0num>::mask : 0x00);

		_MOTATE_CREATE_CLEAR_AND_COPY_MASKS(A);
		_MOTATE_CREATE_CLEAR_AND_COPY_MASKS(B);
		_MOTATE_CREATE_CLEAR_AND_COPY_MASKS(C);
		_MOTATE_CREATE_CLEAR_AND_COPY_MASKS(D);
		_MOTATE_CREATE_CLEAR_AND_COPY_MASKS(E);
		_MOTATE_CREATE_CLEAR_AND_COPY_MASKS(F);
				
	public:
		PinHolder8() {
			
		};
		
		void write(const uint8_t in_value, const uint8_t mask = 0xff) {
			uint8_t port_value;
#define _MOTATE_PINHOLDER_CHECKANDSETPIN(portLetter, bitNumber, bitMask) \
			if (PinBit ## bitNumber.maskForPort(port ## portLetter.letter) &&\
					(PinBit ## bitNumber.mask != (bitMask)) && (in_value & mask & (bitMask))) {\
				port_value |= PinBit ## bitNumber.mask;\
			}
			
#define _MOTATE_PINHOLDER_SETPORT(portLetter) \
			if (port ## portLetter ## ClearMask && mask) {\
				port_value = 0x00;\
				_MOTATE_PINHOLDER_CHECKANDSETPIN(portLetter, 7, 0b10000000);\
				_MOTATE_PINHOLDER_CHECKANDSETPIN(portLetter, 6, 0b01000000);\
				_MOTATE_PINHOLDER_CHECKANDSETPIN(portLetter, 5, 0b00100000);\
				_MOTATE_PINHOLDER_CHECKANDSETPIN(portLetter, 4, 0b00010000);\
				_MOTATE_PINHOLDER_CHECKANDSETPIN(portLetter, 3, 0b00001000);\
				_MOTATE_PINHOLDER_CHECKANDSETPIN(portLetter, 2, 0b00000100);\
				_MOTATE_PINHOLDER_CHECKANDSETPIN(portLetter, 1, 0b00000010);\
				_MOTATE_PINHOLDER_CHECKANDSETPIN(portLetter, 0, 0b00000001);\
				port_value |= in_value & port ## portLetter ## CopyMask;\
				port ## portLetter.write(port_value, ~(mask & port ## portLetter ## ClearMask));\
			}
			
			_MOTATE_PINHOLDER_SETPORT(A);
			_MOTATE_PINHOLDER_SETPORT(B);
			_MOTATE_PINHOLDER_SETPORT(C);
			_MOTATE_PINHOLDER_SETPORT(D);
			_MOTATE_PINHOLDER_SETPORT(E);
			_MOTATE_PINHOLDER_SETPORT(F);
		}
		
	};
} // namespace Motate
#endif /* end of include guard: AVRPINS_H_ONCE */