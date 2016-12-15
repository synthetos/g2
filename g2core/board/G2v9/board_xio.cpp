/*
 * board_xio.cpp - extended IO functions that are board-specific
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

#include "g2core.h"
#include "config.h"
#include "hardware.h"
#include "board_xio.h"

//******** USB ********
#if XIO_HAS_USB
const Motate::USBSettings_t Motate::USBSettings = {
    /*gVendorID         = */ 0x1d50,
    /*gProductID        = */ 0x606d,
    /*gProductVersion   = */ G2CORE_FIRMWARE_VERSION,
    /*gAttributes       = */ kUSBConfigAttributeSelfPowered,
    /*gPowerConsumption = */ 500
};
/*gProductVersion   = */ //0.1,

XIOUSBDevice_t usb;

decltype(usb.mixin<0>::Serial) &SerialUSB = usb.mixin<0>::Serial;
#if USB_SERIAL_PORTS_EXPOSED == 2
decltype(usb.mixin<1>::Serial) &SerialUSB1 = usb.mixin<1>::Serial;
#endif

MOTATE_SET_USB_VENDOR_STRING( u"Synthetos" )
MOTATE_SET_USB_PRODUCT_STRING( u"TinyG v2" )
MOTATE_SET_USB_SERIAL_NUMBER_STRING_FROM_CHIPID()
#endif // XIO_HAS_USB


//******** SPI ********
#if XIO_HAS_SPI
Motate::SPI<kSocket4_SPISlaveSelectPinNumber> spi;
#endif


//******** UART ********
#if XIO_HAS_UART
Motate::UART<Motate::kSerial_RXPinNumber, Motate::kSerial_TXPinNumber, Motate::kSerial_RTSPinNumber, Motate::kSerial_CTSPinNumber> Serial{
    115200, Motate::UARTMode::RTSCTSFlowControl};
#endif

void board_hardware_init(void)  // called 1st
{
#if XIO_HAS_USB
    // Init USB
    usb.attach();                   // USB setup. Runs in "background" as the rest of this executes
#endif // XIO_HAS_USB
}


void board_xio_init(void)  // called later than board_hardware_init (there are thing in between)
{
// Init SPI
#if XIO_HAS_SPI
// handled internally for now
#endif

// Init UART
#if XIO_HAS_UART
    Serial.init();
#endif
}
