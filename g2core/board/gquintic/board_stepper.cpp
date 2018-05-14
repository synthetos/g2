/*
 * board_stepper.cpp - board-specific code for stepper.cpp
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

#include "board_stepper.h"


Motate::SPIChipSelectPinMux<Motate::kSocket1_SPISlaveSelectPinNumber,
                            Motate::kSocket2_SPISlaveSelectPinNumber,
                            Motate::kSocket3_SPISlaveSelectPinNumber,
                            -1>
              spiCSPinMux;
SPIBus_used_t spiBus;

// These are identical to board_stepper.h, except for the word "extern" and the initialization
Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket1_StepPinNumber,
             Motate::kSocket1_DirPinNumber,
             Motate::kSocket1_EnablePinNumber>
    motor_1{spiBus, spiCSPinMux.getCS(4)};

Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket2_StepPinNumber,
             Motate::kSocket2_DirPinNumber,
             Motate::kSocket2_EnablePinNumber>
    motor_2{spiBus, spiCSPinMux.getCS(3)};

Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket3_StepPinNumber,
             Motate::kSocket3_DirPinNumber,
             Motate::kSocket3_EnablePinNumber>
    motor_3{spiBus, spiCSPinMux.getCS(2)};

Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket4_StepPinNumber,
             Motate::kSocket4_DirPinNumber,
             Motate::kSocket4_EnablePinNumber>
    motor_4{spiBus, spiCSPinMux.getCS(1)};

Trinamic2130<SPIBus_used_t::SPIBusDevice,
             Motate::kSocket5_StepPinNumber,
             Motate::kSocket5_DirPinNumber,
             Motate::kSocket5_EnablePinNumber>
    motor_5{spiBus, spiCSPinMux.getCS(0)};

Stepper* Motors[MOTORS] = {&motor_1, &motor_2, &motor_3, &motor_4, &motor_5};

void board_stepper_init() {
    spiBus.init();

    for (uint8_t motor = 0; motor < MOTORS; motor++) { Motors[motor]->init(); }
}
