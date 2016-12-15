/*
 * board_stepper.h - board-specific code for stepper.h
 * This file is part of the g2core project
 *
 * Copyright (c) 2016 Alden S. Hart, Jr.
 * Copyright (c) 2016 Robert Giseburt
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
#include "step_dir_driver.h"
#include "step_dir_hobbyservo.h"

extern StepDirStepper<Motate::kSocket1_StepPinNumber,
                      Motate::kSocket1_DirPinNumber,
                      Motate::kSocket1_EnablePinNumber,
                      Motate::kSocket1_Microstep_0PinNumber,
                      Motate::kSocket1_Microstep_1PinNumber,
                      Motate::kSocket1_Microstep_2PinNumber,
                      Motate::kSocket1_VrefPinNumber>
    motor_1;

extern StepDirStepper<Motate::kSocket2_StepPinNumber,
                      Motate::kSocket2_DirPinNumber,
                      Motate::kSocket2_EnablePinNumber,
                      Motate::kSocket2_Microstep_0PinNumber,
                      Motate::kSocket2_Microstep_1PinNumber,
                      Motate::kSocket2_Microstep_2PinNumber,
                      Motate::kSocket2_VrefPinNumber>
    motor_2;

 extern StepDirHobbyServo<Motate::kServo1_PinNumber> motor_3;

// extern StepDirStepper<
//    Motate::kSocket5_StepPinNumber,
//    Motate::kSocket5_DirPinNumber,
//    Motate::kSocket5_EnablePinNumber,
//    Motate::kSocket5_Microstep_0PinNumber,
//    Motate::kSocket5_Microstep_1PinNumber,
//    Motate::kSocket5_Microstep_2PinNumber,
//    Motate::kSocket5_VrefPinNumber> motor_5;
//
// extern StepDirStepper<
//    Motate::kSocket6_StepPinNumber,
//    Motate::kSocket6_DirPinNumber,
//    Motate::kSocket6_EnablePinNumber,
//    Motate::kSocket6_Microstep_0PinNumber,
//    Motate::kSocket6_Microstep_1PinNumber,
//    Motate::kSocket6_Microstep_2PinNumber,
//    Motate::kSocket6_VrefPinNumber> motor_6 {};

extern Stepper* Motors[MOTORS];

void board_stepper_init();

#endif  // BOARD_STEPPER_H_ONCE
