/*
 * board_stepper.cpp - board-specific code for stepper.cpp
 * For: /board/gQuintic
 * This file is part of the g2core project
 *
 * Copyright (c) 2016 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2019 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
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
 */

#include "board_stepper.h"

#include "MotateTimers.h"

// These are identical to board_stepper.h, except for the word "extern" and the initialization
#if QUINTIC_REVISION == 'C'
HOT_DATA Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket2_StepPinNumber,
             Motate::kSocket2_DirPinNumber,
             Motate::kSocket2_EnablePinNumber>
    motor_1 {spiBus, spiCSPinMux.getCS(3)};
HOT_DATA Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket3_StepPinNumber,
             Motate::kSocket3_DirPinNumber,
             Motate::kSocket3_EnablePinNumber>
    motor_2 {spiBus, spiCSPinMux.getCS(2)};
HOT_DATA Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket4_StepPinNumber,
             Motate::kSocket4_DirPinNumber,
             Motate::kSocket4_EnablePinNumber>
    motor_3 {spiBus, spiCSPinMux.getCS(1)};
HOT_DATA Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket5_StepPinNumber,
             Motate::kSocket5_DirPinNumber,
             Motate::kSocket5_EnablePinNumber>
    motor_4 {spiBus, spiCSPinMux.getCS(0)};
#if HAS_HOBBY_SERVO_MOTOR
HOT_DATA StepDirHobbyServo<Motate::kOutput10_PinNumber> motor_5;
Stepper* const Motors[MOTORS] = {&motor_1, &motor_2, &motor_3, &motor_4, &motor_5};
#else
Stepper* const Motors[MOTORS] = {&motor_1, &motor_2, &motor_3, &motor_4};
#endif
#endif // 'C'

#if QUINTIC_REVISION == 'D'
HOT_DATA Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket1_StepPinNumber,
             Motate::kSocket1_DirPinNumber,
             Motate::kSocket1_EnablePinNumber>
    motor_1 {spiBus, spiCSPinMux.getCS(4)};
HOT_DATA Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket2_StepPinNumber,
             Motate::kSocket2_DirPinNumber,
             Motate::kSocket2_EnablePinNumber>
    motor_2 {spiBus, spiCSPinMux.getCS(3)};
HOT_DATA Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket3_StepPinNumber,
             Motate::kSocket3_DirPinNumber,
             Motate::kSocket3_EnablePinNumber>
    motor_3 {spiBus, spiCSPinMux.getCS(2)};
HOT_DATA Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket4_StepPinNumber,
             Motate::kSocket4_DirPinNumber,
             Motate::kSocket4_EnablePinNumber>
    motor_4 {spiBus, spiCSPinMux.getCS(1)};
HOT_DATA Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket5_StepPinNumber,
             Motate::kSocket5_DirPinNumber,
             Motate::kSocket5_EnablePinNumber>
    motor_5 {spiBus, spiCSPinMux.getCS(0)};
#if HAS_HOBBY_SERVO_MOTOR
HOT_DATA StepDirHobbyServo<Motate::kOutput10_PinNumber> motor_6;
Stepper* const Motors[MOTORS] = {&motor_1, &motor_2, &motor_3, &motor_4, &motor_5, &motor_6};
#elif HAS_LASER
// laser_tool is defined over in hardware.cpp
extern LaserTool_used_t laser_tool;
LaserTool_used_t &motor_6 = laser_tool;
Stepper* const Motors[MOTORS] = {&motor_1, &motor_2, &motor_3, &motor_4, &motor_5, &motor_6};
#else
Stepper* const Motors[MOTORS] = {&motor_1, &motor_2, &motor_3, &motor_4, &motor_5};
#endif
#endif // 'D'


#if (KINEMATICS == KINE_FOUR_CABLE)
HOT_DATA encoder_0_t encoder_0{plex0, M1_ENCODER_INPUT_A, M1_ENCODER_INPUT_B, 1 << 0};
HOT_DATA encoder_1_t encoder_1{plex0, M2_ENCODER_INPUT_A, M2_ENCODER_INPUT_B, 1 << 1};
HOT_DATA encoder_2_t encoder_2{plex0, M3_ENCODER_INPUT_A, M3_ENCODER_INPUT_B, 1 << 2};
HOT_DATA encoder_3_t encoder_3{plex0, M4_ENCODER_INPUT_A, M4_ENCODER_INPUT_B, 1 << 3};

ExternalEncoder* const ExternalEncoders[4] = {&encoder_0, &encoder_1, &encoder_2, &encoder_3};

int8_t ee_sample_counter_ = 100;
Motate::SysTickEvent external_encoders_tick_event {[&] {
    if (!--ee_sample_counter_) {

        encoder_0.requestAngleFraction();
        encoder_1.requestAngleFraction();
        encoder_2.requestAngleFraction();
        encoder_3.requestAngleFraction();
        ee_sample_counter_ = 1;
    }
}, nullptr};

#else
ExternalEncoder* const ExternalEncoders[0] = {};
#endif

void board_stepper_init() {
    for (uint8_t motor = 0; motor < MOTORS; motor++) { Motors[motor]->init(); }
    // Motate::SysTickTimer.registerEvent(&external_encoders_tick_event);
}
