/*
 * http://tinkerin.gs/
 *
 * Copyright (c) 2013 Robert Giseburt
 * Copyright (c) 2013 Alden S. Hart Jr.
 *
 * This file is part of the Motate Library.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef sbv300_pinout_h
#define sbv300_pinout_h

#include <MotatePins.h>

namespace Motate {

    // NOT ALL OF THESE PINS ARE ON ALL PLATFORMS
    // Undefined pins will be equivalent to Motate::NullPin, and return 1 for Pin<>::isNull();

    pin_number kSerial_RX                       =   -1;
    pin_number kSerial_TX                       =   -1;

    pin_number kSerial0_RX                      =   -1;
    pin_number kSerial0_TX                      =   -1;

    pin_number kI2C_SDAPinNumber                =  -1;
    pin_number kI2C_SCLPinNumber                =  -1;

    pin_number kI2C0_SDAPinNumber               =  -1;
    pin_number kI2C0_SCLPinNumber               =  -1;

    pin_number kSPI_SCKPinNumber                =  -1;
    pin_number kSPI_MISOPinNumber               =  -1;
    pin_number kSPI_MOSIPinNumber               =  -1;

    pin_number kSPI0_SCKPinNumber               =  -1;
    pin_number kSPI0_MISOPinNumber              =  -1;
    pin_number kSPI0_MOSIPinNumber              =  -1;

    pin_number kDebug1_PinNumber                =  -1;
    pin_number kDebug2_PinNumber                =  49;
    pin_number kDebug3_PinNumber                =  50;
    pin_number kDebug4_PinNumber                =  -1;

    pin_number kKinen_SyncPinNumber             =  -1;

    pin_number kSocket1_SPISlaveSelectPinNumber =  -1;
    pin_number kSocket1_InterruptPinNumber      =  -1;
    pin_number kSocket1_StepPinNumber           =   0;
    pin_number kSocket1_DirPinNumber            =   6;
    pin_number kSocket1_EnablePinNumber         =  -1;  // TODO Global Enable
    pin_number kSocket1_Microstep_0PinNumber    =  -1;
    pin_number kSocket1_Microstep_1PinNumber    =  -1;
    pin_number kSocket1_Microstep_2PinNumber    =  -1;
    pin_number kSocket1_VrefPinNumber           =  -1;

    pin_number kSocket2_SPISlaveSelectPinNumber =  -1;
    pin_number kSocket2_InterruptPinNumber      =  -1;
    pin_number kSocket2_StepPinNumber           =   1;
    pin_number kSocket2_DirPinNumber            =   7;
    pin_number kSocket2_EnablePinNumber         =  -1;  // TODO Global Enable
    pin_number kSocket2_Microstep_0PinNumber    =  -1;
    pin_number kSocket2_Microstep_1PinNumber    =  -1;
    pin_number kSocket2_Microstep_2PinNumber    =  -1;
    pin_number kSocket2_VrefPinNumber           =  -1;

    pin_number kSocket3_SPISlaveSelectPinNumber =  -1;
    pin_number kSocket3_InterruptPinNumber      =  -1;
    pin_number kSocket3_StepPinNumber           =   2;
    pin_number kSocket3_DirPinNumber            =   8;
    pin_number kSocket3_EnablePinNumber         =  -1;  // TODO Global Enable
    pin_number kSocket3_Microstep_0PinNumber    =  -1;
    pin_number kSocket3_Microstep_1PinNumber    =  -1;
    pin_number kSocket3_Microstep_2PinNumber    =  -1;
    pin_number kSocket3_VrefPinNumber           =  -1;

    pin_number kSocket4_SPISlaveSelectPinNumber =  -1;
    pin_number kSocket4_InterruptPinNumber      =  -1;
    pin_number kSocket4_StepPinNumber           =  3;
    pin_number kSocket4_DirPinNumber            =  9;
    pin_number kSocket4_EnablePinNumber         =  -1;
    pin_number kSocket4_Microstep_0PinNumber    =  -1;
    pin_number kSocket4_Microstep_1PinNumber    =  -1;
    pin_number kSocket4_Microstep_2PinNumber    =  -1;
    pin_number kSocket4_VrefPinNumber           =  -1;

    pin_number kSocket5_SPISlaveSelectPinNumber =  -1;
    pin_number kSocket5_InterruptPinNumber      =  -1;
    pin_number kSocket5_StepPinNumber           =  4;
    pin_number kSocket5_DirPinNumber            =  10;
    pin_number kSocket5_EnablePinNumber         =  -1;
    pin_number kSocket5_Microstep_0PinNumber    =  -1;
    pin_number kSocket5_Microstep_1PinNumber    =  -1;
    pin_number kSocket5_Microstep_2PinNumber    =  -1;
    pin_number kSocket5_VrefPinNumber           =  -1;

    pin_number kSocket6_SPISlaveSelectPinNumber =  -1;
    pin_number kSocket6_InterruptPinNumber      =  -1;
    pin_number kSocket6_StepPinNumber           =  5;
    pin_number kSocket6_DirPinNumber            =  11;
    pin_number kSocket6_EnablePinNumber         =  -1;
    pin_number kSocket6_Microstep_0PinNumber    =  -1;
    pin_number kSocket6_Microstep_1PinNumber    =  -1;
    pin_number kSocket6_Microstep_2PinNumber    =  -1;
    pin_number kSocket6_VrefPinNumber           =  -1;

    // SPINDLE ON 1 = B 15
    // PERMISSIVE 4 = D 7
    pin_number kSpindle_EnablePinNumber         =  14;  // TODO enable spindle
    pin_number kSpindle_DirPinNumber            =  15;

    pin_number kCoolant_EnablePinNumber         =  -1;

    pin_number kSpindle_PwmPinNumber            =  -1;
    pin_number kSpindle_Pwm2PinNumber           =  -1;

    pin_number kInput1_PinNumber              =  16;
    pin_number kInput2_PinNumber              =  17;
    pin_number kInput3_PinNumber              =  18;
    pin_number kInput4_PinNumber              =  19;
    pin_number kInput5_PinNumber              =  20;
    pin_number kInput6_PinNumber              =  21;

    pin_number kInput7_PinNumber              =  22;
    pin_number kInput8_PinNumber              =  23;
    pin_number kInput9_PinNumber              =  24;
    pin_number kInput10_PinNumber              =  25;
    pin_number kInput11_PinNumber              =  26;
    pin_number kInput12_PinNumber              =  27;

    pin_number kSD_CardDetect                   =  -1;
    pin_number kInterlock_In                    =  -1;

    pin_number kLED_USBRXPinNumber              =  12;
    pin_number kLED_USBTXPinNumber              =  13;


    // GRBL / gShield compatibility pins -- Due board ONLY

    pin_number kGRBL_ResetPinNumber             =  -1;
    pin_number kGRBL_FeedHoldPinNumber          =  -1;
    pin_number kGRBL_CycleStartPinNumber        =  -1;

    pin_number kGRBL_CommonEnablePinNumber      =  -1;
    
    /** NOTE: When adding pin definitions here, they must be
     *        added to ALL board pin assignment files, even if
     *        they are defined as -1.
     **/

} // namespace Motate

#endif
