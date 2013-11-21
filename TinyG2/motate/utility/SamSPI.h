/*
  utility/SamSPI.h - Library for the Motate system
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

#ifndef SAMSPI_H_ONCE
#define SAMSPI_H_ONCE

// #include <chip.h>
#include "sam.h"

namespace Motate {
	
	template<int8_t spiPortNum>
	struct SPI {
		static const int8_t portNumber = -1;
		SPI() {};
		SPI(const uint32_t frequency, const uint16_t options = kNormal) {};

		void init(const uint16_t options = kNormal, const bool fromConstructor=false) {};
		void setOptions(const uint16_t options, const bool fromConstructor=false) {};
		uint16_t getOptions() { return kNormal; };

		int16_t readByte() {
//			return usb.readByte(read_endpoint);
		};
        
		uint16_t read(const uint8_t *buffer, const uint16_t length) {
			int16_t total_read = 0;
			int16_t to_read = length;
			const uint8_t *read_ptr = buffer;
            
			// BLOCKING!!
			while (to_read > 0) {
				// Oddity of english: "to read" and "amount read" makes the same read.
				// So, we'll call it "amount_read".
//				int16_t amount_read = usb.read(read_endpoint, read_ptr, length);
                
				total_read += amount_read;
				to_read -= amount_read;
				read_ptr += amount_read;
			};
            
			return total_read;
		};
        
		int32_t write(const uint8_t *data, const uint16_t length) {
			int16_t total_written = 0;
			int16_t written = 1; // start with a non-zero value
			const uint8_t *out_buffer = data;
			int16_t to_write = length;
            
			// BLOCKING!!
			do {
//				written = usb.write(write_endpoint, out_buffer, length);
                
				if (written < 0) // ERROR!
                    break;
                
				// TODO: Do this better... -Rob
				total_written += written;
				to_write -= written;
				out_buffer += written;
			} while (to_write);
            
			// HACK! Autoflush forced...
			if (total_written > 0)
                flush();
            
			return total_written;
		}
        
		void flush() {
			// TODO
		}
        
//		bool isConnected() {
//			return _line_state & (0x01 << 1);
//		}
        
		bool isNull() { return true; };
	};

}

#endif /* end of include guard: SAMSPI_H_ONCE */