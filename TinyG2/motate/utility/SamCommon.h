/*
 utility/SamCommon.h - Library for the Arduino-compatible Motate system
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

#ifndef SAMCOMMON_H_ONCE
#define SAMCOMMON_H_ONCE

#include "sam.h"

namespace Motate {
   
    template <class parent>
	struct SamCommon {
        
        // ToDo: Make this inherited! It's repeated in timer, pins, USB, and SPI.
		static void enablePeripheralClock() {
			if (parent::peripheralId() < 32) {
				uint32_t id_mask = 1u << ( parent::peripheralId() );
				if ((PMC->PMC_PCSR0 & id_mask) != id_mask) {
					PMC->PMC_PCER0 = id_mask;
				}
			} else {
				uint32_t id_mask = 1u << ( parent::peripheralId() - 32 );
				if ((PMC->PMC_PCSR1 & id_mask) != id_mask) {
					PMC->PMC_PCER1 = id_mask;
				}
			}
		};
        
		static void disablePeripheralClock() {
			if (parent::peripheralId() < 32) {
				uint32_t id_mask = 1u << ( parent::peripheralId() );
				if ((PMC->PMC_PCSR0 & id_mask) == id_mask) {
					PMC->PMC_PCDR0 = id_mask;
				}
			} else {
				uint32_t id_mask = 1u << ( parent::peripheralId() - 32 );
				if ((PMC->PMC_PCSR1 & id_mask) == id_mask) {
					PMC->PMC_PCDR1 = id_mask;
				}
			}
		};
    };
    
} // namespace Motate

#endif /* end of include guard: SAMCOMMON_H_ONCE */