/*
 * board_xio.cpp - extended IO functions that are board-specific
 * Part of TinyG project
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

#include "tinyg2.h"
#include "config.h"
#include "hardware.h"
#include "board_xio.h"

//******** USB ********
const Motate::USBSettings_t Motate::USBSettings = {
    /*gVendorID         = */ 0x1d50,
    /*gProductID        = */ 0x606d,
    /*gProductVersion   = */ TINYG_FIRMWARE_VERSION,
    /*gAttributes       = */ kUSBConfigAttributeSelfPowered,
    /*gPowerConsumption = */ 500
};
/*gProductVersion   = */ //0.1,

//Motate::USBDevice< Motate::USBCDC > usb;
Motate::USBDevice< Motate::USBCDC, Motate::USBCDC > usb;

decltype(usb.mixin<0>::Serial) &SerialUSB = usb.mixin<0>::Serial;
decltype(usb.mixin<1>::Serial) &SerialUSB1 = usb.mixin<1>::Serial;

// 115200 is the default, as well.
//UART<kSerial_RX, kSerial_TX, kSerial_RTS, kSerial_CTS> Serial {115200, UARTMode::RTSCTSFlowControl};


MOTATE_SET_USB_VENDOR_STRING( {'S' ,'y', 'n', 't', 'h', 'e', 't', 'o', 's'} )
MOTATE_SET_USB_PRODUCT_STRING( {'T', 'i', 'n', 'y', 'G', ' ', 'v', '2'} )
MOTATE_SET_USB_SERIAL_NUMBER_STRING_FROM_CHIPID()



//******** SPI ********
//Motate::SPI<kSocket4_SPISlaveSelectPinNumber> spi;



//******** UART ********
Motate::UART<Motate::kSerial_RX, Motate::kSerial_TX, Motate::kSerial_RTS, Motate::kSerial_CTS> Serial {115200, Motate::UARTMode::RTSCTSFlowControl};

void board_hardware_init(void) // called 1st
{
    // Init USB
    usb.attach();                   // USB setup. Runs in "background" as the rest of this executes
}


void board_xio_init(void) // called later than board_hardware_init (there are thing in between)
{
    // Init SPI (handled internally for now)

    // Init UART
    Serial.init();
}