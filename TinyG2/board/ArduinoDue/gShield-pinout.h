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

#ifndef gShield_pinout_h
#define gShield_pinout_h

#include <MotatePins.h>

namespace Motate {

    // NOT ALL OF THESE PINS ARE ON ALL PLATFORMS
    // Undefined pins will be equivalent to Motate::NullPin, and return 1 for Pin<>::isNull();

    pin_number kSerial_RX                       =   0;
    pin_number kSerial_TX                       =   1;

    pin_number kSerial0_RX                      =   0;
    pin_number kSerial0_TX                      =   1;

    pin_number kI2C_SDAPinNumber                =  20;
    pin_number kI2C_SCLPinNumber                =  21;

    pin_number kI2C0_SDAPinNumber               =  20;
    pin_number kI2C0_SCLPinNumber               =  21;

    pin_number kSPI_SCKPinNumber                =  76;
    pin_number kSPI_MISOPinNumber               =  74;
    pin_number kSPI_MOSIPinNumber               =  75;

    pin_number kSPI0_SCKPinNumber               =  76;
    pin_number kSPI0_MISOPinNumber              =  74;
    pin_number kSPI0_MOSIPinNumber              =  75;

//    pin_number kX_StepPinNumber                 =  53;
//    pin_number kX_DirPinNumber                  =  52;
//    pin_number kX_EnablePinNumber               =  -1;
//
//    pin_number kY_StepPinNumber                 =  51;
//    pin_number kY_DirPinNumber                  =  50;
//    pin_number kY_EnablePinNumber               =  -1;
//
//    pin_number kZ_StepPinNumber                 =  49;
//    pin_number kZ_DirPinNumber                  =  48;
//    pin_number kZ_EnablePinNumber               =  -1;

    pin_number kDebug1_PinNumber                =  49;
    pin_number kDebug2_PinNumber                =  47;
    pin_number kDebug3_PinNumber                =  45;
    pin_number kDebug4_PinNumber                =  -1;

    pin_number kKinen_SyncPinNumber             =  53;

    pin_number kSocket1_SPISlaveSelectPinNumber =  -1;//10;
    pin_number kSocket1_InterruptPinNumber      =  -1;
    pin_number kSocket1_StepPinNumber           =   2;
    pin_number kSocket1_DirPinNumber            =   5;
    pin_number kSocket1_EnablePinNumber         =  22;
    pin_number kSocket1_Microstep_0PinNumber    =  23;
    pin_number kSocket1_Microstep_1PinNumber    =  24;
    pin_number kSocket1_Microstep_2PinNumber    =  -1;
    pin_number kSocket1_VrefPinNumber           =  -1;//34; //PWMTimer<0>

    pin_number kSocket2_SPISlaveSelectPinNumber =  65;
    pin_number kSocket2_InterruptPinNumber      =  -1;
    pin_number kSocket2_StepPinNumber           =   3;
    pin_number kSocket2_DirPinNumber            =   6;
    pin_number kSocket2_EnablePinNumber         =  25;
    pin_number kSocket2_Microstep_0PinNumber    =  26;
    pin_number kSocket2_Microstep_1PinNumber    =  27;
    pin_number kSocket2_Microstep_2PinNumber    =  -1;
    pin_number kSocket2_VrefPinNumber           =  -1;//62; //PWMTimer<1>

    pin_number kSocket3_SPISlaveSelectPinNumber =  52;
    pin_number kSocket3_InterruptPinNumber      =  -1;
    pin_number kSocket3_StepPinNumber           =   4;
    pin_number kSocket3_DirPinNumber            =   7;
    pin_number kSocket3_EnablePinNumber         =  28;
    pin_number kSocket3_Microstep_0PinNumber    =  29;
    pin_number kSocket3_Microstep_1PinNumber    =  30;
    pin_number kSocket3_Microstep_2PinNumber    =  -1;
    pin_number kSocket3_VrefPinNumber           =  -1;//63; //PWMTimer<2>

    pin_number kSocket4_SPISlaveSelectPinNumber =  77;
    pin_number kSocket4_InterruptPinNumber      =  -1;
    pin_number kSocket4_StepPinNumber           =  31;
    pin_number kSocket4_DirPinNumber            =  32;
    pin_number kSocket4_EnablePinNumber         =  33;
    pin_number kSocket4_Microstep_0PinNumber    =  35;
    pin_number kSocket4_Microstep_1PinNumber    =  36;
    pin_number kSocket4_Microstep_2PinNumber    =  -1;
    pin_number kSocket4_VrefPinNumber           =  -1;//64; //PWMTimer<3>

    pin_number kSocket5_SPISlaveSelectPinNumber =  78;
    pin_number kSocket5_InterruptPinNumber      =  -1;
    pin_number kSocket5_StepPinNumber           =  37;
    pin_number kSocket5_DirPinNumber            =  38;
    pin_number kSocket5_EnablePinNumber         =  39;
    pin_number kSocket5_Microstep_0PinNumber    =  40;
    pin_number kSocket5_Microstep_1PinNumber    =  41;
    pin_number kSocket5_Microstep_2PinNumber    =  -1;
    pin_number kSocket5_VrefPinNumber           =  -1;//66; //PWMTimer<3>

    pin_number kSocket6_SPISlaveSelectPinNumber =  50;
    pin_number kSocket6_InterruptPinNumber      =  -1;
    pin_number kSocket6_StepPinNumber           =  42;
    pin_number kSocket6_DirPinNumber            =  43;
    pin_number kSocket6_EnablePinNumber         =  44;
    pin_number kSocket6_Microstep_0PinNumber    =  -1;//45;
    pin_number kSocket6_Microstep_1PinNumber    =  46;
    pin_number kSocket6_Microstep_2PinNumber    =  -1;
    pin_number kSocket6_VrefPinNumber           =  -1;//67; //PWMTimer<0>


    pin_number kInput1_PinNumber              =  14;
    pin_number kInput2_PinNumber              =  15;
    pin_number kInput3_PinNumber              =  16;
    pin_number kInput4_PinNumber              =  17;
    pin_number kInput5_PinNumber              =  18;
    pin_number kInput6_PinNumber              =  19;

    pin_number kInput7_PinNumber              =  58;
    pin_number kInput8_PinNumber              =  59;
    pin_number kInput9_PinNumber              =  60;
    pin_number kInput10_PinNumber              =  61;
    pin_number kInput11_PinNumber              =  65;
    pin_number kInput12_PinNumber              =  51;

    pin_number kSpindle_EnablePinNumber         =  12;
    pin_number kSpindle_DirPinNumber            =  -1;//13;
    pin_number kSpindle_PwmPinNumber            =  11;
    pin_number kSpindle_Pwm2PinNumber           =   9;
    pin_number kCoolant_EnablePinNumber         =  57;

    pin_number kSD_CardDetectPinNumber          =  -1;
    pin_number kInterlock_InPinNumber           =  -1;

    pin_number kLED_USBRXPinNumber              =  72;
    pin_number kLED_USBTXPinNumber              =  73;


    // GRBL / gShield compatibility pins -- Due board ONLY

    pin_number kGRBL_ResetPinNumber             =  54;
    pin_number kGRBL_FeedHoldPinNumber          =  55;
    pin_number kGRBL_CycleStartPinNumber        =  56;

    pin_number kGRBL_CommonEnablePinNumber      =   8;
    
    /** NOTE: When adding pin definitions here, they must be
     *        added to ALL board pin assignment files, even if
     *        they are defined as -1.
     **/

} // namespace Motate

#endif
