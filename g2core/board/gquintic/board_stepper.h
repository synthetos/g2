/*
 * board_stepper.h - board-specific code for stepper.h
 * For: /board/gQuintic
 * This file is part of the g2core project
 *
 * Copyright (c) 2016 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2018 Robert Giseburt
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
#ifndef BOARD_STEPPER_H_ONCE
#define BOARD_STEPPER_H_ONCE

#include "hardware.h"  // for MOTORS
#include "tmc2130.h"
#include "step_dir_hobbyservo.h"

// These are identical to board_stepper.h, except for the word "extern" and the initialization
#if QUINTIC_REVISION == 'C'
extern Trinamic2130<SPIBus_used_t::SPIBusDevice,
                    Motate::kSocket2_StepPinNumber,
                    Motate::kSocket2_DirPinNumber,
                    Motate::kSocket2_EnablePinNumber>
    motor_1;
extern Trinamic2130<SPIBus_used_t::SPIBusDevice,
                    Motate::kSocket3_StepPinNumber,
                    Motate::kSocket3_DirPinNumber,
                    Motate::kSocket3_EnablePinNumber>
    motor_2;
extern Trinamic2130<SPIBus_used_t::SPIBusDevice,
                    Motate::kSocket4_StepPinNumber,
                    Motate::kSocket4_DirPinNumber,
                    Motate::kSocket4_EnablePinNumber>
    motor_3;
extern Trinamic2130<SPIBus_used_t::SPIBusDevice,
                    Motate::kSocket5_StepPinNumber,
                    Motate::kSocket5_DirPinNumber,
                    Motate::kSocket5_EnablePinNumber>
    motor_4;
#if HAS_HOBBY_SERVO_MOTOR
extern StepDirHobbyServo<Motate::kOutput10_PinNumber> motor_5;
#endif
#endif // 'C'

#if QUINTIC_REVISION == 'D'
extern Trinamic2130<SPIBus_used_t::SPIBusDevice,
                    Motate::kSocket1_StepPinNumber,
                    Motate::kSocket1_DirPinNumber,
                    Motate::kSocket1_EnablePinNumber>
    motor_1;

extern Trinamic2130<SPIBus_used_t::SPIBusDevice,
                    Motate::kSocket2_StepPinNumber,
                    Motate::kSocket2_DirPinNumber,
                    Motate::kSocket2_EnablePinNumber>
    motor_2;

extern Trinamic2130<SPIBus_used_t::SPIBusDevice,
                    Motate::kSocket3_StepPinNumber,
                    Motate::kSocket3_DirPinNumber,
                    Motate::kSocket3_EnablePinNumber>
    motor_3;

extern Trinamic2130<SPIBus_used_t::SPIBusDevice,
                    Motate::kSocket4_StepPinNumber,
                    Motate::kSocket4_DirPinNumber,
                    Motate::kSocket4_EnablePinNumber>
    motor_4;

extern Trinamic2130<SPIBus_used_t::SPIBusDevice,
                    Motate::kSocket5_StepPinNumber,
                    Motate::kSocket5_DirPinNumber,
                    Motate::kSocket5_EnablePinNumber>
    motor_5;
#if HAS_HOBBY_SERVO_MOTOR
extern StepDirHobbyServo<Motate::kOutput10_PinNumber> motor_6;
#elif HAS_LASER

#include "laser_toolhead.h"
#include "kinematics_cartesian.h"
typedef LaserTool<BASE_KINEMATICS, LASER_FIRE_PIN_NUMBER> LaserTool_used_t;
extern LaserTool_used_t &motor_6;

#endif
#endif // 'D'

extern Stepper* const Motors[MOTORS];

#if (KINEMATICS == KINE_FOUR_CABLE)
using encoder_0_t = decltype(I2C_AS5601{plex0, M1_ENCODER_INPUT_A, M1_ENCODER_INPUT_B, 1 << 0});
extern HOT_DATA encoder_0_t encoder_0;
using encoder_1_t = decltype(I2C_AS5601{plex0, M2_ENCODER_INPUT_A, M2_ENCODER_INPUT_B, 1 << 1});
extern HOT_DATA encoder_1_t encoder_1;
using encoder_2_t = decltype(I2C_AS5601{plex0, M3_ENCODER_INPUT_A, M3_ENCODER_INPUT_B, 1 << 2});
extern HOT_DATA encoder_2_t encoder_2;
using encoder_3_t = decltype(I2C_AS5601{plex0, M4_ENCODER_INPUT_A, M4_ENCODER_INPUT_B, 1 << 3});
extern HOT_DATA encoder_3_t encoder_3;

extern ExternalEncoder* const ExternalEncoders[4];
#else
extern ExternalEncoder* const ExternalEncoders[0];
#endif

void board_stepper_init();

#endif  // BOARD_STEPPER_H_ONCE
