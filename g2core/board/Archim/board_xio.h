/*
 * board_xio.h - extended IO functions that are board-specific
 * This file is part of the g2core project
 *
 * Copyright (c) 2016 Alden S. Hart Jr.
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

#ifndef board_xio_h
#define board_xio_h

//******** USB ********
#include "MotateUSB.h"
#include "MotateUSBCDC.h"

// extern Motate::USBDevice< Motate::USBCDC > usb;
extern Motate::USBDevice<Motate::USBCDC, Motate::USBCDC> usb;
extern decltype(usb.mixin<0>::Serial)& SerialUSB;
extern decltype(usb.mixin<1>::Serial)& SerialUSB1;



//******** SPI ********
//#include "MotateSPI.h"
// extern Motate::SPI<Motate::kSocket4_SPISlaveSelectPinNumber> spi;



//******** UART ********
#define XIO_HAS_UART 0
//#include "MotateUART.h"
// extern Motate::UART<Motate::kSerial_RX, Motate::kSerial_TX, Motate::kSerial_RTS, Motate::kSerial_CTS> Serial;



//******* Generic Functions *******
void board_xio_init(void);

#endif  // board_xio_h
