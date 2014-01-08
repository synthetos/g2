/*
 SamPinss.cpp - Library for the Arduino-compatible Motate system
 http://tinkerin.gs/
 
 Copyright (c) 2014 Robert Giseburt
 
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


#if defined(__SAM3X8E__) || defined(__SAM3X8C__)

#include "utility/SamPins.h"

using namespace Motate;

#define CheckAndCallInterruptForPortPin(portChar, portLetter, portPin) \
if (ReversePinLookup<portChar, portPin>::interrupt != 0 && isr & 1 << portPin) {\
    ReversePinLookup<portChar, portPin>::interrupt();\
}

extern "C" void PIOA_Handler(void) {
    uint32_t isr = PIOA->PIO_ISR;
    NVIC_ClearPendingIRQ(PIOA_IRQn);
    
    CheckAndCallInterruptForPortPin('A', A,  0);
    CheckAndCallInterruptForPortPin('A', A,  1);
    CheckAndCallInterruptForPortPin('A', A,  2);
    CheckAndCallInterruptForPortPin('A', A,  3);
    CheckAndCallInterruptForPortPin('A', A,  4);
    CheckAndCallInterruptForPortPin('A', A,  5);
    CheckAndCallInterruptForPortPin('A', A,  6);
    CheckAndCallInterruptForPortPin('A', A,  7);
    CheckAndCallInterruptForPortPin('A', A,  8);
    CheckAndCallInterruptForPortPin('A', A,  9);
    CheckAndCallInterruptForPortPin('A', A, 10);
    CheckAndCallInterruptForPortPin('A', A, 11);
    CheckAndCallInterruptForPortPin('A', A, 12);
    CheckAndCallInterruptForPortPin('A', A, 13);
    CheckAndCallInterruptForPortPin('A', A, 14);
    CheckAndCallInterruptForPortPin('A', A, 15);
    CheckAndCallInterruptForPortPin('A', A, 16);
    CheckAndCallInterruptForPortPin('A', A, 17);
    CheckAndCallInterruptForPortPin('A', A, 18);
    CheckAndCallInterruptForPortPin('A', A, 19);
    CheckAndCallInterruptForPortPin('A', A, 20);
    CheckAndCallInterruptForPortPin('A', A, 21);
    CheckAndCallInterruptForPortPin('A', A, 22);
    CheckAndCallInterruptForPortPin('A', A, 23);
    CheckAndCallInterruptForPortPin('A', A, 24);
    CheckAndCallInterruptForPortPin('A', A, 25);
    CheckAndCallInterruptForPortPin('A', A, 26);
    CheckAndCallInterruptForPortPin('A', A, 27);
    CheckAndCallInterruptForPortPin('A', A, 28);
    CheckAndCallInterruptForPortPin('A', A, 29);
    CheckAndCallInterruptForPortPin('A', A, 30);
    CheckAndCallInterruptForPortPin('A', A, 31);
}


extern "C" void PIOB_Handler(void) {
    uint32_t isr = PIOB->PIO_ISR;
    NVIC_ClearPendingIRQ(PIOB_IRQn);

    CheckAndCallInterruptForPortPin('B', B,  0);
    CheckAndCallInterruptForPortPin('B', B,  1);
    CheckAndCallInterruptForPortPin('B', B,  2);
    CheckAndCallInterruptForPortPin('B', B,  3);
    CheckAndCallInterruptForPortPin('B', B,  4);
    CheckAndCallInterruptForPortPin('B', B,  5);
    CheckAndCallInterruptForPortPin('B', B,  6);
    CheckAndCallInterruptForPortPin('B', B,  7);
    CheckAndCallInterruptForPortPin('B', B,  8);
    CheckAndCallInterruptForPortPin('B', B,  9);
    CheckAndCallInterruptForPortPin('B', B, 10);
    CheckAndCallInterruptForPortPin('B', B, 11);
    CheckAndCallInterruptForPortPin('B', B, 12);
    CheckAndCallInterruptForPortPin('B', B, 13);
    CheckAndCallInterruptForPortPin('B', B, 14);
    CheckAndCallInterruptForPortPin('B', B, 15);
    CheckAndCallInterruptForPortPin('B', B, 16);
    CheckAndCallInterruptForPortPin('B', B, 17);
    CheckAndCallInterruptForPortPin('B', B, 18);
    CheckAndCallInterruptForPortPin('B', B, 19);
    CheckAndCallInterruptForPortPin('B', B, 20);
    CheckAndCallInterruptForPortPin('B', B, 21);
    CheckAndCallInterruptForPortPin('B', B, 22);
    CheckAndCallInterruptForPortPin('B', B, 23);
    CheckAndCallInterruptForPortPin('B', B, 24);
    CheckAndCallInterruptForPortPin('B', B, 25);
    CheckAndCallInterruptForPortPin('B', B, 26);
    CheckAndCallInterruptForPortPin('B', B, 27);
    CheckAndCallInterruptForPortPin('B', B, 28);
    CheckAndCallInterruptForPortPin('B', B, 29);
    CheckAndCallInterruptForPortPin('B', B, 30);
    CheckAndCallInterruptForPortPin('B', B, 31);
}

#ifdef PIOC
extern "C" void PIOC_Handler(void) {
    uint32_t isr = PIOC->PIO_ISR;
    NVIC_ClearPendingIRQ(PIOC_IRQn);

    CheckAndCallInterruptForPortPin('C', C,  0);
    CheckAndCallInterruptForPortPin('C', C,  1);
    CheckAndCallInterruptForPortPin('C', C,  2);
    CheckAndCallInterruptForPortPin('C', C,  3);
    CheckAndCallInterruptForPortPin('C', C,  4);
    CheckAndCallInterruptForPortPin('C', C,  5);
    CheckAndCallInterruptForPortPin('C', C,  6);
    CheckAndCallInterruptForPortPin('C', C,  7);
    CheckAndCallInterruptForPortPin('C', C,  8);
    CheckAndCallInterruptForPortPin('C', C,  9);
    CheckAndCallInterruptForPortPin('C', C, 10);
    CheckAndCallInterruptForPortPin('C', C, 11);
    CheckAndCallInterruptForPortPin('C', C, 12);
    CheckAndCallInterruptForPortPin('C', C, 13);
    CheckAndCallInterruptForPortPin('C', C, 14);
    CheckAndCallInterruptForPortPin('C', C, 15);
    CheckAndCallInterruptForPortPin('C', C, 16);
    CheckAndCallInterruptForPortPin('C', C, 17);
    CheckAndCallInterruptForPortPin('C', C, 18);
    CheckAndCallInterruptForPortPin('C', C, 19);
    CheckAndCallInterruptForPortPin('C', C, 20);
    CheckAndCallInterruptForPortPin('C', C, 21);
    CheckAndCallInterruptForPortPin('C', C, 22);
    CheckAndCallInterruptForPortPin('C', C, 23);
    CheckAndCallInterruptForPortPin('C', C, 24);
    CheckAndCallInterruptForPortPin('C', C, 25);
    CheckAndCallInterruptForPortPin('C', C, 26);
    CheckAndCallInterruptForPortPin('C', C, 27);
    CheckAndCallInterruptForPortPin('C', C, 28);
    CheckAndCallInterruptForPortPin('C', C, 29);
    CheckAndCallInterruptForPortPin('C', C, 30);
    CheckAndCallInterruptForPortPin('C', C, 31);
}

#endif // PORTC

#ifdef PIOD
extern "C" void PIOD_Handler(void) {
    uint32_t isr = PIOD->PIO_ISR;
    NVIC_ClearPendingIRQ(PIOD_IRQn);

    CheckAndCallInterruptForPortPin('D', D,  0);
    CheckAndCallInterruptForPortPin('D', D,  1);
    CheckAndCallInterruptForPortPin('D', D,  2);
    CheckAndCallInterruptForPortPin('D', D,  3);
    CheckAndCallInterruptForPortPin('D', D,  4);
    CheckAndCallInterruptForPortPin('D', D,  5);
    CheckAndCallInterruptForPortPin('D', D,  6);
    CheckAndCallInterruptForPortPin('D', D,  7);
    CheckAndCallInterruptForPortPin('D', D,  8);
    CheckAndCallInterruptForPortPin('D', D,  9);
    CheckAndCallInterruptForPortPin('D', D, 10);
    CheckAndCallInterruptForPortPin('D', D, 11);
    CheckAndCallInterruptForPortPin('D', D, 12);
    CheckAndCallInterruptForPortPin('D', D, 13);
    CheckAndCallInterruptForPortPin('D', D, 14);
    CheckAndCallInterruptForPortPin('D', D, 15);
    CheckAndCallInterruptForPortPin('D', D, 16);
    CheckAndCallInterruptForPortPin('D', D, 17);
    CheckAndCallInterruptForPortPin('D', D, 18);
    CheckAndCallInterruptForPortPin('D', D, 19);
    CheckAndCallInterruptForPortPin('D', D, 20);
    CheckAndCallInterruptForPortPin('D', D, 21);
    CheckAndCallInterruptForPortPin('D', D, 22);
    CheckAndCallInterruptForPortPin('D', D, 23);
    CheckAndCallInterruptForPortPin('D', D, 24);
    CheckAndCallInterruptForPortPin('D', D, 25);
    CheckAndCallInterruptForPortPin('D', D, 26);
    CheckAndCallInterruptForPortPin('D', D, 27);
    CheckAndCallInterruptForPortPin('D', D, 28);
    CheckAndCallInterruptForPortPin('D', D, 29);
    CheckAndCallInterruptForPortPin('D', D, 30);
    CheckAndCallInterruptForPortPin('D', D, 31);
}

#endif // PORTD

#endif