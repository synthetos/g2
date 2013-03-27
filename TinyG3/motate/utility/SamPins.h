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

#ifndef SAMPINS_H_ONCE
#define SAMPINS_H_ONCE

// #include <chip.h>
#include "sam.h"

namespace Motate {
	// Numbering is arbitrary:
	enum PinMode {
		kUnchanged      = 0,
		kOutput         = 1,
		kInput          = 2,
		// These next two are NOT available on other platforms,
		// but cannot be masked out since they are required for
		// special pin functions. These should not be used in
		// end-user (sketch) code.
		kPeripheralA    = 3,
		kPeripheralB    = 4,
	};

	// Numbering is arbitrary, but bit unique for bitwise operations (unlike other architectures):
	enum PinOptions {
		kNormal         = 0,
		kTotem          = 0, // alias
		kPullUp         = 1<<1,
#if !defined(MOTATE_AVR_COMPATIBILITY)
		kWiredAnd       = 1<<2,
		kDriveLowOnly   = 1<<2, // alias
		kWiredAndPull   = kWiredAnd|kPullUp,
		kDriveLowPullUp = kDriveLowOnly|kPullUp, // alias
#endif // !MOTATE_AVR_COMPATIBILITY
#if !defined(MOTATE_AVR_COMPATIBILITY) && !defined(MOTATE_AVRX_COMPATIBILITY)
		kDeglitch       = 1<<4,
		kDebounce       = 1<<5,
#endif // !MOTATE_AVR_COMPATIBILITY && !MOTATE_SAM_COMPATIBILITY
	};
	
	typedef uint32_t uintPort_t;

	template <unsigned char portLetter>
	struct Port32 {
		static const uint8_t letter = (uint8_t) portLetter;
		static Pio* portPtr;
		static uint32_t pmcId;
		
		void setModes(const uintPort_t value, const uintPort_t mask = 0xffffffff) {
			// stub
		};
		void setOptions(const uint16_t options, const uintPort_t mask) {
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
		uintPort_t getInputValues(const uintPort_t mask = 0xffffffff) {
			// stub
			return 0;
		};
		uintPort_t getOutputValues(const uintPort_t mask = 0xffffffff) {
			// stub
			return 0;
		};
		
		/* SAM specific: */
		void enablePeripheralClock();
		void disablePeripheralClock();
	};

	template<int8_t pinNum>
	struct Pin {
		static const int8_t number = -1;
		static const uint8_t portLetter = 0;
		static const uint32_t mask = 0;
		static uint32_t maskForPort(const uint8_t otherPortLetter) {
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
	
	// TODO: Make the Pin<> use the appropriate Port<>, reducing duplication when there's no penalty
	
	// (type == OutputOpendrain) ? PIO_OPENDRAIN : PIO_DEFAULT
	#define _MAKE_MOTATE_PIN(pinNum, registerLetter, registerChar, registerPin)\
		template<>\
		struct Pin<pinNum> {\
		private: /* Lock the copy contructor.*/\
			Pin(const Pin<pinNum>&){};\
		public:\
			static const int8_t number = pinNum;\
			static const uint8_t portLetter = (uint8_t) registerChar;\
			static const uint32_t mask = (1u << registerPin);\
			static Pio* portPtr;\
			static Port32<registerChar> port;\
			\
			Pin() {};\
			Pin(const PinMode type, const PinOptions options = kNormal) {\
				init(type, options, /*fromConstructor=*/true);\
			};\
			void operator=(const bool value) { write(value); };\
			operator bool() { return (get() != 0); };\
			\
			void init(const PinMode type, const uint16_t options = kNormal, const bool fromConstructor=false) {\
				setMode(type, fromConstructor);\
				setOptions(options, fromConstructor);\
			};\
			void setMode(const PinMode type, const bool fromConstructor=false) {\
				switch (type) {\
					case kOutput:\
						portPtr->PIO_OER = mask ;\
						portPtr->PIO_PER = mask ;\
						/* if all pins are output, disable PIO Controller clocking, reduce power consumption */\
						if (!fromConstructor) {\
							if ( portPtr->PIO_OSR == 0xffffffff )\
							{\
								port.disablePeripheralClock();\
							}\
						}\
						break;\
					case kInput:\
						port.enablePeripheralClock();\
						portPtr->PIO_ODR = mask ;\
						portPtr->PIO_PER = mask ;\
						break;\
					default:\
						break;\
				}\
			};\
			PinMode getMode() {\
				return (portPtr->PIO_OSR & mask) ? kOutput : kInput;\
			};\
			void setOptions(const uint16_t options, const bool fromConstructor=false) {\
				if (kPullUp & options)\
				{\
					portPtr->PIO_PUER = mask ;\
				}\
				else\
				{\
					portPtr->PIO_PUDR = mask ;\
				}\
				if (kWiredAnd & options)\
				{/*kDriveLowOnly - Enable Multidrive*/\
					portPtr->PIO_MDER = mask ;\
				}\
				else\
				{\
					portPtr->PIO_MDDR = mask ;\
				}\
				if (kDeglitch & options)\
				{\
					portPtr->PIO_IFER = mask ;\
					portPtr->PIO_SCIFSR = mask ;\
					}\
					else\
					{\
					if (kDebounce & options)\
					{\
						portPtr->PIO_IFER = mask ;\
						portPtr->PIO_DIFSR = mask ;\
					}\
						else\
					{\
						portPtr->PIO_IFDR = mask ;\
					}\
				}\
			};\
			uint16_t getOptions() {\
				return ((portPtr->PIO_PUSR & mask) ? kPullUp : 0)\
					| ((portPtr->PIO_MDSR & mask) ? kWiredAnd : 0)\
					| ((portPtr->PIO_IFSR & mask) ? \
						((portPtr->PIO_IFDGSR & mask) ? kDebounce : kDeglitch) : 0);\
			};\
			void set() {\
				portPtr->PIO_SODR = mask;\
			};\
			void clear() {\
				portPtr->PIO_CODR = mask;\
			};\
			void write(bool value) {\
				if (!value)\
					clear();\
				else\
					set();\
			};\
			void toggle()  {\
				portPtr->PIO_ODSR ^= mask;\
			};\
			uint8_t get() { /* WARNING: This will fail if the peripheral clock is disabled for this pin!!! Use getOutputValue() instead. */\
				return portPtr->PIO_PDSR & mask;\
			};\
			uint8_t getInputValue() {\
				return portPtr->PIO_PDSR & mask;\
			};\
			uint8_t getOutputValue() {\
				return portPtr->PIO_OSR & mask;\
			};\
			bool isNull() { return false; };\
			static uint32_t maskForPort(const uint8_t otherPortLetter) {\
				return portLetter == otherPortLetter ? mask : 0x00u;\
			};\
		};\
		Pio* Pin<pinNum>::portPtr = (PIO ## registerLetter);\
		typedef Pin<pinNum> Pin ## pinNum;\
		static Pin ## pinNum pin ## pinNum;


	#define _MAKE_MOTATE_PORT32(registerLetter, registerChar)\
		template <> inline void Port32<registerChar>::enablePeripheralClock() {\
			if (pmcId < 32) {\
				uint32_t id_mask = 1u << ( pmcId );\
				if ((PMC->PMC_PCSR0 & id_mask) != id_mask) {\
					PMC->PMC_PCER0 = id_mask;\
				}\
			} else {\
				uint32_t id_mask = 1u << ( pmcId - 32 );\
				if ((PMC->PMC_PCSR1 & id_mask) != id_mask) {\
					PMC->PMC_PCER1 = id_mask;\
				}\
			}\
		};\
		template <> inline void Port32<registerChar>::disablePeripheralClock() {\
			if (pmcId < 32) {\
				uint32_t id_mask = 1u << ( pmcId );\
				if ((PMC->PMC_PCSR0 & id_mask) == id_mask) {\
					PMC->PMC_PCDR0 = id_mask;\
				}\
			} else {\
				uint32_t id_mask = 1u << ( pmcId-32 );\
				if ((PMC->PMC_PCSR1 & id_mask) == id_mask) {\
					PMC->PMC_PCDR1 = id_mask;\
				}\
			}\
		};\
		template <> inline void Port32<registerChar>::setModes(const uintPort_t value, const uintPort_t mask) {\
			portPtr->PIO_ODR = ~value & mask ;\
			portPtr->PIO_OER = value & mask ;\
			portPtr->PIO_PER = mask ;\
			/* if all pins are output, disable PIO Controller clocking, reduce power consumption */\
			if ( portPtr->PIO_OSR == 0xffffffff )\
			{\
				disablePeripheralClock();\
			} else {\
				enablePeripheralClock();\
			}\
		};\
		template <> inline void Port32<registerChar>::setOptions(const uint16_t options, const uintPort_t mask) {\
			if (kPullUp & options)\
			{\
				portPtr->PIO_PUER = mask ;\
			}\
			else\
			{\
				portPtr->PIO_PUDR = mask ;\
			}\
			if (kWiredAnd & options)\
			{/*kDriveLowOnly - Enable Multidrive*/\
				portPtr->PIO_MDER = mask ;\
			}\
			else\
			{\
				portPtr->PIO_MDDR = mask ;\
			}\
			if (kDeglitch & options)\
			{\
				portPtr->PIO_IFER = mask ;\
				portPtr->PIO_SCIFSR = mask ;\
				}\
				else\
				{\
				if (kDebounce & options)\
				{\
					portPtr->PIO_IFER = mask ;\
					portPtr->PIO_DIFSR = mask ;\
				}\
					else\
				{\
					portPtr->PIO_IFDR = mask ;\
				}\
			}\
		};\
		template <> inline void Port32<registerChar>::set(const uintPort_t value) {\
			portPtr->PIO_SODR = value;\
		};\
		template <> inline void Port32<registerChar>::clear(const uintPort_t value) {\
			portPtr->PIO_CODR = value;\
		};\
		template <> inline void Port32<registerChar>::write(const uintPort_t value) {\
			portPtr->PIO_OWER = 0xffffffff;/*Enable all registers for writing thru ODSR*/\
			portPtr->PIO_ODSR = value;\
			portPtr->PIO_OWDR = 0xffffffff;/*Disable all registers for writing thru ODSR*/\
		};\
		template <> inline void Port32<registerChar>::write(const uintPort_t value, const uintPort_t mask) {\
			portPtr->PIO_OWER = mask;/*Enable masked registers for writing thru ODSR*/\
			portPtr->PIO_ODSR = value;\
			portPtr->PIO_OWDR = mask;/*Disable masked registers for writing thru ODSR*/\
		};\
		template <> inline uintPort_t Port32<registerChar>::getInputValues(const uintPort_t mask) {\
			return portPtr->PIO_PDSR & mask;\
		};\
		template <> inline uintPort_t Port32<registerChar>::getOutputValues(const uintPort_t mask) {\
			return portPtr->PIO_OSR & mask;\
		};\
		template <> Pio* Port32<registerChar>::portPtr = ( PIO ## registerLetter );\
		template <> uint32_t Port32<registerChar>::pmcId = ( ID_PIO ## registerLetter );\
		typedef Port32<registerChar> Port ## registerLetter;\
		static Port ## registerLetter port ## registerLetter;

	typedef Pin<-1> NullPin;
	static NullPin nullPin;

	// #if part_is_defined( SAM3X8E )

	// DUE
		_MAKE_MOTATE_PORT32(A, 'A');
		_MAKE_MOTATE_PORT32(B, 'B');
		_MAKE_MOTATE_PORT32(C, 'C');
		_MAKE_MOTATE_PORT32(D, 'D');

		_MAKE_MOTATE_PIN( 0, A, 'A',  8);
		_MAKE_MOTATE_PIN( 1, A, 'A',  9);
		_MAKE_MOTATE_PIN( 2, B, 'B', 25);
		_MAKE_MOTATE_PIN( 3, C, 'C', 28);
		_MAKE_MOTATE_PIN( 4, C, 'C', 26);
		_MAKE_MOTATE_PIN( 5, C, 'C', 25);
		_MAKE_MOTATE_PIN( 6, C, 'C', 24);
		_MAKE_MOTATE_PIN( 7, C, 'C', 23);
		_MAKE_MOTATE_PIN( 8, C, 'C', 22);
		_MAKE_MOTATE_PIN( 9, C, 'C', 21);
		_MAKE_MOTATE_PIN(10, C, 'C', 29);
		_MAKE_MOTATE_PIN(11, D, 'D',  7);
		_MAKE_MOTATE_PIN(12, D, 'D',  8);
		_MAKE_MOTATE_PIN(13, B, 'B', 27);
		_MAKE_MOTATE_PIN(14, D, 'D',  4);
		_MAKE_MOTATE_PIN(15, D, 'D',  5);
		_MAKE_MOTATE_PIN(16, A, 'A', 13);
		_MAKE_MOTATE_PIN(17, A, 'A', 12);
		_MAKE_MOTATE_PIN(18, A, 'A', 11);
		_MAKE_MOTATE_PIN(19, A, 'A', 10);
		_MAKE_MOTATE_PIN(20, B, 'B', 12);
		_MAKE_MOTATE_PIN(21, B, 'B', 13);
		_MAKE_MOTATE_PIN(22, B, 'B', 26);
		_MAKE_MOTATE_PIN(23, A, 'A', 14);
		_MAKE_MOTATE_PIN(24, A, 'A', 15);
		_MAKE_MOTATE_PIN(25, D, 'D',  0);
		_MAKE_MOTATE_PIN(26, D, 'D',  1);
		_MAKE_MOTATE_PIN(27, D, 'D',  2);
		_MAKE_MOTATE_PIN(28, D, 'D',  3);
		_MAKE_MOTATE_PIN(29, D, 'D',  6);
		_MAKE_MOTATE_PIN(30, D, 'D',  9);
		_MAKE_MOTATE_PIN(31, A, 'A',  7);
		_MAKE_MOTATE_PIN(32, D, 'D', 10);
		_MAKE_MOTATE_PIN(33, C, 'C',  1);
		_MAKE_MOTATE_PIN(34, C, 'C',  2);
		_MAKE_MOTATE_PIN(35, C, 'C',  3);
		_MAKE_MOTATE_PIN(36, C, 'C',  4);
		_MAKE_MOTATE_PIN(37, C, 'C',  5);
		_MAKE_MOTATE_PIN(38, C, 'C',  6);
		_MAKE_MOTATE_PIN(39, C, 'C',  7);
		_MAKE_MOTATE_PIN(40, C, 'C',  8);
		_MAKE_MOTATE_PIN(41, C, 'C',  9);
		_MAKE_MOTATE_PIN(42, A, 'A', 19);
		_MAKE_MOTATE_PIN(43, A, 'A', 20);
		_MAKE_MOTATE_PIN(44, C, 'C', 19);
		_MAKE_MOTATE_PIN(45, C, 'C', 18);
		_MAKE_MOTATE_PIN(46, C, 'C', 17);
		_MAKE_MOTATE_PIN(47, C, 'C', 16);
		_MAKE_MOTATE_PIN(48, C, 'C', 15);
		_MAKE_MOTATE_PIN(49, C, 'C', 14);
		_MAKE_MOTATE_PIN(50, C, 'C', 13);
		_MAKE_MOTATE_PIN(51, C, 'C', 12);
		_MAKE_MOTATE_PIN(52, B, 'B', 21);
		_MAKE_MOTATE_PIN(53, B, 'B', 14);
		_MAKE_MOTATE_PIN(54, A, 'A', 16);
		_MAKE_MOTATE_PIN(55, A, 'A', 24);
		_MAKE_MOTATE_PIN(56, A, 'A', 23);
		_MAKE_MOTATE_PIN(57, A, 'A', 22);
		_MAKE_MOTATE_PIN(58, A, 'A',  6);
		_MAKE_MOTATE_PIN(59, A, 'A',  4);
		_MAKE_MOTATE_PIN(60, A, 'A',  3);
		_MAKE_MOTATE_PIN(61, A, 'A',  2);
		_MAKE_MOTATE_PIN(62, B, 'B', 17);
		_MAKE_MOTATE_PIN(63, B, 'B', 18);
		_MAKE_MOTATE_PIN(64, B, 'B', 19);
		_MAKE_MOTATE_PIN(65, B, 'B', 20);
		_MAKE_MOTATE_PIN(66, B, 'B', 15);
		_MAKE_MOTATE_PIN(67, B, 'B', 16);
		_MAKE_MOTATE_PIN(68, A, 'A',  1);
		_MAKE_MOTATE_PIN(69, A, 'A',  0);
		_MAKE_MOTATE_PIN(70, A, 'A', 17);
		_MAKE_MOTATE_PIN(71, A, 'A', 18);
		_MAKE_MOTATE_PIN(72, C, 'C', 30);
		_MAKE_MOTATE_PIN(73, A, 'A', 21);
		_MAKE_MOTATE_PIN(74, A, 'A', 25);
		_MAKE_MOTATE_PIN(75, A, 'A', 26);
		_MAKE_MOTATE_PIN(76, A, 'A', 27);
		_MAKE_MOTATE_PIN(77, A, 'A', 28);
		_MAKE_MOTATE_PIN(78, B, 'B', 23);
	// #endif

// disable pinholder for Due for now -- nned to convert to 32bit
	// PinHolder - 32bit virtual ports (I've never made a template with 32 parameters before.)
	template<
		int8_t PinBit31num,
		int8_t PinBit30num = -1,
		int8_t PinBit29num = -1,
		int8_t PinBit28num = -1,
		int8_t PinBit27num = -1,
		int8_t PinBit26num = -1,
		int8_t PinBit25num = -1,
		int8_t PinBit24num = -1,
		int8_t PinBit23num = -1,
		int8_t PinBit22num = -1,
		int8_t PinBit21num = -1,
		int8_t PinBit20num = -1,
		int8_t PinBit19num = -1,
		int8_t PinBit18num = -1,
		int8_t PinBit17num = -1,
		int8_t PinBit16num = -1,
		int8_t PinBit15num = -1,
		int8_t PinBit14num = -1,
		int8_t PinBit13num = -1,
		int8_t PinBit12num = -1,
		int8_t PinBit11num = -1,
		int8_t PinBit10num = -1,
		int8_t PinBit9num  = -1,
		int8_t PinBit8num  = -1,
		int8_t PinBit7num  = -1,
		int8_t PinBit6num  = -1,
		int8_t PinBit5num  = -1,
		int8_t PinBit4num  = -1,
		int8_t PinBit3num  = -1,
		int8_t PinBit2num  = -1,
		int8_t PinBit1num  = -1,
		int8_t PinBit0num  = -1>
	class PinHolder32 {

		static Pin<PinBit31num> PinBit31;
		static Pin<PinBit30num> PinBit30;
		static Pin<PinBit29num> PinBit29;
		static Pin<PinBit28num> PinBit28;
		static Pin<PinBit27num> PinBit27;
		static Pin<PinBit26num> PinBit26;
		static Pin<PinBit25num> PinBit25;
		static Pin<PinBit24num> PinBit24;
		static Pin<PinBit23num> PinBit23;
		static Pin<PinBit22num> PinBit22;
		static Pin<PinBit21num> PinBit21;
		static Pin<PinBit20num> PinBit20;
		static Pin<PinBit19num> PinBit19;
		static Pin<PinBit18num> PinBit18;
		static Pin<PinBit17num> PinBit17;
		static Pin<PinBit16num> PinBit16;
		static Pin<PinBit15num> PinBit15;
		static Pin<PinBit14num> PinBit14;
		static Pin<PinBit13num> PinBit13;
		static Pin<PinBit12num> PinBit12;
		static Pin<PinBit11num> PinBit11;
		static Pin<PinBit10num> PinBit10;
		static Pin<PinBit9num>  PinBit9;
		static Pin<PinBit8num>  PinBit8;
		static Pin<PinBit7num>  PinBit7;
		static Pin<PinBit6num>  PinBit6;
		static Pin<PinBit5num>  PinBit5;
		static Pin<PinBit4num>  PinBit4;
		static Pin<PinBit3num>  PinBit3;
		static Pin<PinBit2num>  PinBit2;
		static Pin<PinBit1num>  PinBit1;
		static Pin<PinBit0num>  PinBit0;

#define _MOTATE_PH32_CREATE_CLEAR_AND_COPY_MASKS(aPortLetter) \
		static const uint32_t port ## aPortLetter ## ClearMask =\
			(Pin<PinBit31num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit31num>::mask : 0x00) |\
			(Pin<PinBit30num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit30num>::mask : 0x00) |\
			(Pin<PinBit29num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit29num>::mask : 0x00) |\
			(Pin<PinBit28num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit28num>::mask : 0x00) |\
			(Pin<PinBit27num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit27num>::mask : 0x00) |\
			(Pin<PinBit26num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit26num>::mask : 0x00) |\
			(Pin<PinBit25num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit25num>::mask : 0x00) |\
			(Pin<PinBit24num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit24num>::mask : 0x00) |\
			(Pin<PinBit23num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit23num>::mask : 0x00) |\
			(Pin<PinBit22num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit22num>::mask : 0x00) |\
			(Pin<PinBit21num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit21num>::mask : 0x00) |\
			(Pin<PinBit20num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit20num>::mask : 0x00) |\
			(Pin<PinBit19num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit19num>::mask : 0x00) |\
			(Pin<PinBit18num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit18num>::mask : 0x00) |\
			(Pin<PinBit17num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit17num>::mask : 0x00) |\
			(Pin<PinBit16num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit16num>::mask : 0x00) |\
			(Pin<PinBit15num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit15num>::mask : 0x00) |\
			(Pin<PinBit14num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit14num>::mask : 0x00) |\
			(Pin<PinBit13num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit13num>::mask : 0x00) |\
			(Pin<PinBit12num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit12num>::mask : 0x00) |\
			(Pin<PinBit11num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit11num>::mask : 0x00) |\
			(Pin<PinBit10num>::portLetter == Port ## aPortLetter::letter ? Pin<PinBit10num>::mask : 0x00) |\
			(Pin<PinBit9num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit9num>::mask  : 0x00) |\
			(Pin<PinBit8num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit8num>::mask  : 0x00) |\
			(Pin<PinBit7num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit7num>::mask  : 0x00) |\
			(Pin<PinBit6num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit6num>::mask  : 0x00) |\
			(Pin<PinBit5num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit5num>::mask  : 0x00) |\
			(Pin<PinBit4num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit4num>::mask  : 0x00) |\
			(Pin<PinBit3num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit3num>::mask  : 0x00) |\
			(Pin<PinBit2num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit2num>::mask  : 0x00) |\
			(Pin<PinBit1num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit1num>::mask  : 0x00) |\
			(Pin<PinBit0num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit0num>::mask  : 0x00);\
\
		static const uint32_t port ## aPortLetter ## CopyMask =\
		(Pin<PinBit31num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit31num>::mask == 0x80000000u ? Pin<PinBit31num>::mask : 0x00) |\
		(Pin<PinBit30num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit30num>::mask == 0x40000000u ? Pin<PinBit30num>::mask : 0x00) |\
		(Pin<PinBit29num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit29num>::mask == 0x20000000u ? Pin<PinBit29num>::mask : 0x00) |\
		(Pin<PinBit28num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit28num>::mask == 0x10000000u ? Pin<PinBit28num>::mask : 0x00) |\
		(Pin<PinBit27num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit27num>::mask == 0x08000000u ? Pin<PinBit27num>::mask : 0x00) |\
		(Pin<PinBit26num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit26num>::mask == 0x04000000u ? Pin<PinBit26num>::mask : 0x00) |\
		(Pin<PinBit25num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit25num>::mask == 0x02000000u ? Pin<PinBit25num>::mask : 0x00) |\
		(Pin<PinBit24num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit24num>::mask == 0x01000000u ? Pin<PinBit24num>::mask : 0x00) |\
		(Pin<PinBit23num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit23num>::mask == 0x00800000u ? Pin<PinBit23num>::mask : 0x00) |\
		(Pin<PinBit22num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit22num>::mask == 0x00400000u ? Pin<PinBit22num>::mask : 0x00) |\
		(Pin<PinBit21num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit21num>::mask == 0x00200000u ? Pin<PinBit21num>::mask : 0x00) |\
		(Pin<PinBit20num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit20num>::mask == 0x00100000u ? Pin<PinBit20num>::mask : 0x00) |\
		(Pin<PinBit19num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit19num>::mask == 0x00080000u ? Pin<PinBit19num>::mask : 0x00) |\
		(Pin<PinBit18num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit18num>::mask == 0x00040000u ? Pin<PinBit18num>::mask : 0x00) |\
		(Pin<PinBit17num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit17num>::mask == 0x00020000u ? Pin<PinBit17num>::mask : 0x00) |\
		(Pin<PinBit16num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit16num>::mask == 0x00010000u ? Pin<PinBit16num>::mask : 0x00) |\
		(Pin<PinBit15num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit15num>::mask == 0x00008000u ? Pin<PinBit15num>::mask : 0x00) |\
		(Pin<PinBit14num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit14num>::mask == 0x00004000u ? Pin<PinBit14num>::mask : 0x00) |\
		(Pin<PinBit13num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit13num>::mask == 0x00002000u ? Pin<PinBit13num>::mask : 0x00) |\
		(Pin<PinBit12num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit12num>::mask == 0x00001000u ? Pin<PinBit12num>::mask : 0x00) |\
		(Pin<PinBit11num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit11num>::mask == 0x00000800u ? Pin<PinBit11num>::mask : 0x00) |\
		(Pin<PinBit10num>::portLetter == Port ## aPortLetter::letter && Pin<PinBit10num>::mask == 0x00000400u ? Pin<PinBit10num>::mask : 0x00) |\
		(Pin<PinBit9num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit9num>::mask  == 0x00000200u ? Pin<PinBit9num>::mask  : 0x00) |\
		(Pin<PinBit8num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit8num>::mask  == 0x00000100u ? Pin<PinBit8num>::mask  : 0x00) |\
		(Pin<PinBit7num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit7num>::mask  == 0x00000080u ? Pin<PinBit7num>::mask  : 0x00) |\
		(Pin<PinBit6num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit6num>::mask  == 0x00000040u ? Pin<PinBit6num>::mask  : 0x00) |\
		(Pin<PinBit5num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit5num>::mask  == 0x00000020u ? Pin<PinBit5num>::mask  : 0x00) |\
		(Pin<PinBit4num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit4num>::mask  == 0x00000010u ? Pin<PinBit4num>::mask  : 0x00) |\
		(Pin<PinBit3num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit3num>::mask  == 0x00000008u ? Pin<PinBit3num>::mask  : 0x00) |\
		(Pin<PinBit2num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit2num>::mask  == 0x00000004u ? Pin<PinBit2num>::mask  : 0x00) |\
		(Pin<PinBit1num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit1num>::mask  == 0x00000002u ? Pin<PinBit1num>::mask  : 0x00) |\
		(Pin<PinBit0num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit0num>::mask  == 0x00000001u ? Pin<PinBit0num>::mask  : 0x00);

		_MOTATE_PH32_CREATE_CLEAR_AND_COPY_MASKS(A);
		_MOTATE_PH32_CREATE_CLEAR_AND_COPY_MASKS(B);
		_MOTATE_PH32_CREATE_CLEAR_AND_COPY_MASKS(C);
		_MOTATE_PH32_CREATE_CLEAR_AND_COPY_MASKS(D);
				
	public:
		PinHolder32() {
			
		};
		
		void set(uint32_t in_value) {
			uint32_t port_value    = 0x00; // Port<> handles reading the port and setting the masked pins
#define _MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, bitNumber, bitMask) \
			if (PinBit ## bitNumber.maskForPort(Port ## portLetter::letter) &&\
					(PinBit ## bitNumber.mask != (bitMask)) && (in_value & (bitMask))) {\
				port_value |= PinBit ## bitNumber.mask;\
			}

// Using direct 0x00000000 notation instead of 1<<x, since the compiler occasionally won't precompile that.
// Shortcut: ruby -e '(0..31).each() { |x| print "_MOTATE_PINHOLDER_CHECKANDSETPIN(portLetter, %2d, 0x%08x);\\\n" % [31-x, (1<<(31-x))]}'
#define _MOTATE_PH32_PINHOLDER_SETPORT(portLetter) \
			if (port ## portLetter ## ClearMask != 0x00) {\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 31, 0x80000000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 30, 0x40000000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 29, 0x20000000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 28, 0x10000000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 27, 0x08000000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 26, 0x04000000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 25, 0x02000000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 24, 0x01000000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 23, 0x00800000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 22, 0x00400000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 21, 0x00200000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 20, 0x00100000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 19, 0x00080000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 18, 0x00040000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 17, 0x00020000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 16, 0x00010000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 15, 0x00008000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 14, 0x00004000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 13, 0x00002000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 12, 0x00001000u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 11, 0x00000800u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter, 10, 0x00000400u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter,  9, 0x00000200u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter,  8, 0x00000100u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter,  7, 0x00000080u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter,  6, 0x00000040u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter,  5, 0x00000020u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter,  4, 0x00000010u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter,  3, 0x00000008u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter,  2, 0x00000004u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter,  1, 0x00000002u);\
				_MOTATE_PH32_PINHOLDER_CHECKANDSETPIN(portLetter,  0, 0x00000001u);\
				port_value |= in_value & port ## portLetter ## CopyMask;\
				port ## portLetter.write(port_value, ~port ## portLetter ## ClearMask);\
			}
			
			_MOTATE_PH32_PINHOLDER_SETPORT(A);
			_MOTATE_PH32_PINHOLDER_SETPORT(B);
			_MOTATE_PH32_PINHOLDER_SETPORT(C);
			_MOTATE_PH32_PINHOLDER_SETPORT(D);
		}
	};
	
	// disable pinholder for Due for now -- nned to convert to 32bit
	// PinHolder - 32bit virtual ports (I've never made a template with 32 parameters before.)
	template<
		int8_t PinBit7num,
		int8_t PinBit6num  = -1,
		int8_t PinBit5num  = -1,
		int8_t PinBit4num  = -1,
		int8_t PinBit3num  = -1,
		int8_t PinBit2num  = -1,
		int8_t PinBit1num  = -1,
		int8_t PinBit0num  = -1>
	class PinHolder8 {

		static Pin<PinBit7num>  PinBit7;
		static Pin<PinBit6num>  PinBit6;
		static Pin<PinBit5num>  PinBit5;
		static Pin<PinBit4num>  PinBit4;
		static Pin<PinBit3num>  PinBit3;
		static Pin<PinBit2num>  PinBit2;
		static Pin<PinBit1num>  PinBit1;
		static Pin<PinBit0num>  PinBit0;
	public:
#define _MOTATE_PH8_CREATE_CLEAR_AND_COPY_MASKS(aPortLetter) \
		static const uint32_t port ## aPortLetter ## ClearMask =\
			(Pin<PinBit7num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit7num>::mask  : 0x00u) |\
			(Pin<PinBit6num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit6num>::mask  : 0x00u) |\
			(Pin<PinBit5num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit5num>::mask  : 0x00u) |\
			(Pin<PinBit4num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit4num>::mask  : 0x00u) |\
			(Pin<PinBit3num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit3num>::mask  : 0x00u) |\
			(Pin<PinBit2num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit2num>::mask  : 0x00u) |\
			(Pin<PinBit1num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit1num>::mask  : 0x00u) |\
			(Pin<PinBit0num>::portLetter  == Port ## aPortLetter::letter ? Pin<PinBit0num>::mask  : 0x00u);\
\
		static const uint32_t port ## aPortLetter ## CopyMask =\
		(Pin<PinBit7num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit7num>::mask  == 0x00000080u ? Pin<PinBit7num>::mask  : 0x00u) |\
		(Pin<PinBit6num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit6num>::mask  == 0x00000040u ? Pin<PinBit6num>::mask  : 0x00u) |\
		(Pin<PinBit5num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit5num>::mask  == 0x00000020u ? Pin<PinBit5num>::mask  : 0x00u) |\
		(Pin<PinBit4num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit4num>::mask  == 0x00000010u ? Pin<PinBit4num>::mask  : 0x00u) |\
		(Pin<PinBit3num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit3num>::mask  == 0x00000008u ? Pin<PinBit3num>::mask  : 0x00u) |\
		(Pin<PinBit2num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit2num>::mask  == 0x00000004u ? Pin<PinBit2num>::mask  : 0x00u) |\
		(Pin<PinBit1num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit1num>::mask  == 0x00000002u ? Pin<PinBit1num>::mask  : 0x00u) |\
		(Pin<PinBit0num>::portLetter  == Port ## aPortLetter::letter && Pin<PinBit0num>::mask  == 0x00000001u ? Pin<PinBit0num>::mask  : 0x00u);

		_MOTATE_PH8_CREATE_CLEAR_AND_COPY_MASKS(A);
		_MOTATE_PH8_CREATE_CLEAR_AND_COPY_MASKS(B);
		_MOTATE_PH8_CREATE_CLEAR_AND_COPY_MASKS(C);
		_MOTATE_PH8_CREATE_CLEAR_AND_COPY_MASKS(D);

	public:
		PinHolder8() {

		};

		void set(uint8_t in_value) {
			uint32_t port_value = 0; // Port<> handles reading the port and setting the masked pins
			#define _MOTATE_PH8_PINHOLDER_CHECKANDSETPIN(portLetter, bitNumber, bitMask) \
				if (PinBit ## bitNumber.maskForPort(Port ## portLetter::letter) &&\
						(PinBit ## bitNumber.mask != (bitMask)) && ((uint32_t)in_value & (bitMask))) {\
					port_value |= PinBit ## bitNumber.mask;\
				}

			// Using direct 0x00000000 notation instead of 1<<x, since the compiler occasionally won't precompile that.
			// Shortcut: ruby -e '(0..7).each() { |x| print "_MOTATE_PH8_PINHOLDER_CHECKANDSETPIN(portLetter, %2d, 0x%02x);\\\n" % [7-x, (1<<(7-x))]}'
			#define _MOTATE_PH8_PINHOLDER_SETPORT(portLetter) \
				if (port ## portLetter ## ClearMask) {\
					port_value = 0;\
					_MOTATE_PH8_PINHOLDER_CHECKANDSETPIN(portLetter,  7, 0x00000080u);\
					_MOTATE_PH8_PINHOLDER_CHECKANDSETPIN(portLetter,  6, 0x00000040u);\
					_MOTATE_PH8_PINHOLDER_CHECKANDSETPIN(portLetter,  5, 0x00000020u);\
					_MOTATE_PH8_PINHOLDER_CHECKANDSETPIN(portLetter,  4, 0x00000010u);\
					_MOTATE_PH8_PINHOLDER_CHECKANDSETPIN(portLetter,  3, 0x00000008u);\
					_MOTATE_PH8_PINHOLDER_CHECKANDSETPIN(portLetter,  2, 0x00000004u);\
					_MOTATE_PH8_PINHOLDER_CHECKANDSETPIN(portLetter,  1, 0x00000002u);\
					_MOTATE_PH8_PINHOLDER_CHECKANDSETPIN(portLetter,  0, 0x00000001u);\
					port_value |= (uint32_t)in_value & port ## portLetter ## CopyMask;\
					port ## portLetter.write(port_value, port ## portLetter ## ClearMask);\
				}

			_MOTATE_PH8_PINHOLDER_SETPORT(A);
			_MOTATE_PH8_PINHOLDER_SETPORT(B);
			_MOTATE_PH8_PINHOLDER_SETPORT(C);
			_MOTATE_PH8_PINHOLDER_SETPORT(D);
		}
	};
}
#endif /* end of include guard: SAMPINS_H_ONCE */