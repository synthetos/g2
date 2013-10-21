/*
  SamTimers.cpp - Library for the Arduino-compatible Motate system
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


#if defined(__SAM3X8E__) || defined(__SAM3X8C__)

#include "utility/SamTimers.h"
#include "Reset.h"

namespace Motate {
	template<> Tc * const        Timer<0>::tc()           { return TC0; };
	template<> TcChannel * const Timer<0>::tcChan()       { return TC0->TC_CHANNEL + 0; };
	template<> const uint32_t    Timer<0>::peripheralId() { return ID_TC0; };
	template<> const IRQn_Type   Timer<0>::tcIRQ()        { return TC0_IRQn; };
	//    static Timer<0> timer0;

	template<> Tc * const        Timer<1>::tc()           { return TC0; };
	template<> TcChannel * const Timer<1>::tcChan()       { return TC0->TC_CHANNEL + 1; };
	template<> const uint32_t    Timer<1>::peripheralId() { return ID_TC1; };
	template<> const IRQn_Type   Timer<1>::tcIRQ()        { return TC1_IRQn; };
	//    static Timer<1> timer1;

	template<> Tc * const        Timer<2>::tc()           { return TC0; };
	template<> TcChannel * const Timer<2>::tcChan()       { return TC0->TC_CHANNEL + 2; };
	template<> const uint32_t    Timer<2>::peripheralId() { return ID_TC2; };
	template<> const IRQn_Type   Timer<2>::tcIRQ()        { return TC2_IRQn; };
	//    static Timer<2> timer2;

	template<> Tc * const        Timer<3>::tc()           { return TC1; };
	template<> TcChannel * const Timer<3>::tcChan()       { return TC1->TC_CHANNEL + 0; };
	template<> const uint32_t    Timer<3>::peripheralId() { return ID_TC3; };
	template<> const IRQn_Type   Timer<3>::tcIRQ()        { return TC3_IRQn; };
	//    static Timer<3> timer3;

	template<> Tc * const        Timer<4>::tc()           { return TC1; };
	template<> TcChannel * const Timer<4>::tcChan()       { return TC1->TC_CHANNEL + 1; };
	template<> const uint32_t    Timer<4>::peripheralId() { return ID_TC4; };
	template<> const IRQn_Type   Timer<4>::tcIRQ()        { return TC4_IRQn; };
	//    static Timer<4> timer4;

	template<> Tc * const        Timer<5>::tc()           { return TC1; };
	template<> TcChannel * const Timer<5>::tcChan()       { return TC1->TC_CHANNEL + 2; };
	template<> const uint32_t    Timer<5>::peripheralId() { return ID_TC5; };
	template<> const IRQn_Type   Timer<5>::tcIRQ()        { return TC5_IRQn; };
	//    static Timer<5> timer5;

#ifdef TC2
	template<> Tc * const        Timer<6>::tc()           { return TC2; };
	template<> TcChannel * const Timer<6>::tcChan()       { return TC2->TC_CHANNEL + 0; };
	template<> const uint32_t    Timer<6>::peripheralId() { return ID_TC6; };
	template<> const IRQn_Type   Timer<6>::tcIRQ()        { return TC6_IRQn; };
	//    static Timer<6> timer6;

	template<> Tc * const        Timer<7>::tc()           { return TC2; };
	template<> TcChannel * const Timer<7>::tcChan()       { return TC2->TC_CHANNEL + 1; };
	template<> const uint32_t    Timer<7>::peripheralId() { return ID_TC7; };
	template<> const IRQn_Type   Timer<7>::tcIRQ()        { return TC7_IRQn; };
	//    static Timer<7> timer7;

	template<> Tc * const        Timer<8>::tc()           { return TC2; };
	template<> TcChannel * const Timer<8>::tcChan()       { return TC2->TC_CHANNEL + 2; };
	template<> const uint32_t    Timer<8>::peripheralId() { return ID_TC8; };
	template<> const IRQn_Type   Timer<8>::tcIRQ()        { return TC8_IRQn; };
	//    static Timer<8> timer8;
#endif

	template<> Pwm * const       PWMTimer<0>::pwm()           { return PWM; };
	template<> PwmCh_num * const PWMTimer<0>::pwmChan()       { return PWM->PWM_CH_NUM + 0; };
	template<> const uint32_t    PWMTimer<0>::peripheralId()  { return ID_PWM; };
	template<> const IRQn_Type   PWMTimer<0>::pwmIRQ()        { return PWM_IRQn; };

	template<> Pwm * const       PWMTimer<1>::pwm()           { return PWM; };
	template<> PwmCh_num * const PWMTimer<1>::pwmChan()       { return PWM->PWM_CH_NUM + 1; };
	template<> const uint32_t    PWMTimer<1>::peripheralId()  { return ID_PWM; };
	template<> const IRQn_Type   PWMTimer<1>::pwmIRQ()        { return PWM_IRQn; };

	template<> Pwm * const       PWMTimer<2>::pwm()           { return PWM; };
	template<> PwmCh_num * const PWMTimer<2>::pwmChan()       { return PWM->PWM_CH_NUM + 2; };
	template<> const uint32_t    PWMTimer<2>::peripheralId()  { return ID_PWM; };
	template<> const IRQn_Type   PWMTimer<2>::pwmIRQ()        { return PWM_IRQn; };

	template<> Pwm * const       PWMTimer<3>::pwm()           { return PWM; };
	template<> PwmCh_num * const PWMTimer<3>::pwmChan()       { return PWM->PWM_CH_NUM + 3; };
	template<> const uint32_t    PWMTimer<3>::peripheralId()  { return ID_PWM; };
	template<> const IRQn_Type   PWMTimer<3>::pwmIRQ()        { return PWM_IRQn; };

	template<> Pwm * const       PWMTimer<4>::pwm()           { return PWM; };
	template<> PwmCh_num * const PWMTimer<4>::pwmChan()       { return PWM->PWM_CH_NUM + 4; };
	template<> const uint32_t    PWMTimer<4>::peripheralId()  { return ID_PWM; };
	template<> const IRQn_Type   PWMTimer<4>::pwmIRQ()        { return PWM_IRQn; };

	template<> Pwm * const       PWMTimer<5>::pwm()           { return PWM; };
	template<> PwmCh_num * const PWMTimer<5>::pwmChan()       { return PWM->PWM_CH_NUM + 5; };
	template<> const uint32_t    PWMTimer<5>::peripheralId()  { return ID_PWM; };
	template<> const IRQn_Type   PWMTimer<5>::pwmIRQ()        { return PWM_IRQn; };

	template<> Pwm * const       PWMTimer<6>::pwm()           { return PWM; };
	template<> PwmCh_num * const PWMTimer<6>::pwmChan()       { return PWM->PWM_CH_NUM + 6; };
	template<> const uint32_t    PWMTimer<6>::peripheralId()  { return ID_PWM; };
	template<> const IRQn_Type   PWMTimer<6>::pwmIRQ()        { return PWM_IRQn; };

	template<> Pwm * const       PWMTimer<7>::pwm()           { return PWM; };
	template<> PwmCh_num * const PWMTimer<7>::pwmChan()       { return PWM->PWM_CH_NUM + 7; };
	template<> const uint32_t    PWMTimer<7>::peripheralId()  { return ID_PWM; };
	template<> const IRQn_Type   PWMTimer<7>::pwmIRQ()        { return PWM_IRQn; };

	/* System-wide tick counter */
	/*  Inspired by code from Atmel and Arduino.
	 *  Some of which is:   Copyright (c) 2012 Arduino. All right reserved.
	 *  Some of which is:   Copyright (c) 2011-2012, Atmel Corporation. All rights reserved.
	 */

	Timer<SysTickTimerNum> SysTickTimer;

	volatile uint32_t Timer<SysTickTimerNum>::_motateTickCount = 0;

} // namespace Motate

extern "C" void SysTick_Handler(void)
{
//	if (sysTickHook)
//		sysTickHook();

	tickReset();

	Motate::SysTickTimer._increment();

	if (Motate::SysTickTimer.interrupt) {
		Motate::SysTickTimer.interrupt();
	}
}

extern "C" {

    void TC0_Handler(void) { Motate::Timer<0>::interrupt(); }
    void TC1_Handler(void) { Motate::Timer<1>::interrupt(); }
    void TC2_Handler(void) { Motate::Timer<2>::interrupt(); }
    void TC3_Handler(void) { Motate::Timer<3>::interrupt(); }
    void TC4_Handler(void) { Motate::Timer<4>::interrupt(); }
    void TC5_Handler(void) { Motate::Timer<5>::interrupt(); }
    void TC6_Handler(void) { Motate::Timer<6>::interrupt(); }
    void TC7_Handler(void) { Motate::Timer<7>::interrupt(); }
    void TC8_Handler(void) { Motate::Timer<8>::interrupt(); }

}

#endif // __SAM3X8E__
