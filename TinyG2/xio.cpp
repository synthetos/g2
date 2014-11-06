/*
 * xio.cpp - extended IO functions
 * This file is part of the TinyG2 project
 *
 * Copyright (c) 2013 - 2014 Alden S. Hart Jr.
 * Copyright (c) 2013 - 2014 Robert Giseburt
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
/*
 * XIO acts as an entry point into lower level IO routines - mostly serial IO. It supports
 * the USB, SPI and file IO sub-systems, as well as providing low level character functions
 * used by stdio (printf()).
 *
 * NOTE: This file is specific to TinyG2/C++/ARM. The TinyG/C/Xmega file is completely different
 */
#include "tinyg2.h"
#include "config.h"
#include "util.h"
#include "hardware.h"
#include "text_parser.h"
#include "canonical_machine.h"
#include "xio.h"
#include "report.h"
#include "controller.h"

/**** Structures ****/

// We need a buffer to hold single character commands, like !~%
// We also want it to have a NULL character, so we make it two characters.
char_t single_char_buffer[2] = " ";

// Use a templated subclass so we don't have to create a new subclass for every type of Device.
// All this requires is the readByte() function to exist in type Device.
// Note that we expect Device to be a pointer type!
// See here for a discussion of what this means if you are not familiar with C++
// https://github.com/synthetos/g2/wiki/Dual-Endpoint-USB-Internals#c-classes-virtual-functions-and-inheritance

template<typename Device>
struct xioDeviceWrapper : xioDeviceWrapperBase {	// describes a device for reading and writing
    Device _dev;
    xioDeviceWrapper(Device dev, uint8_t _caps) : xioDeviceWrapperBase(_caps), _dev{dev}
    {
        _dev->setConnectionCallback([&](bool connected) {	// lambda function
            if (connected) {
                if (!(flags & DEV_IS_CONNECTED)) {
                    //USB0 has just connected
                    //Case 1: This is the first channel to connect -
                    //  set it as CTRL+DATA+PRIMARY channel
                    //Case 2: This is the second (or later) channel to connect -
                    //  set it as DATA channel, remove DATA flag from PRIMARY channel
                    //... inactive channels are counted as closed

                    flags |= (DEV_IS_CONNECTED | DEV_IS_READY);

                    if(!xio.others_connected(this)) {
                        // Case 1
                        flags |= (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_PRIMARY | DEV_IS_ACTIVE);
                        // report that we now have a connection (only for the first one)
                        controller_set_connected(true);
                    } else {
                        // Case 2
                        xio.remove_data_from_primary();
                        flags |= (DEV_IS_DATA | DEV_IS_ACTIVE);
                    }
                } // flags & DEV_IS_DISCONNECTED

            } else { // disconnected
                if (flags & DEV_IS_CONNECTED) {

                    //USB0 has just disconnected
                    //Case 1: This channel disconnected while it was a ctrl+data channel (and no other channels are open) -
                    //  finalize this channel
                    //Case 2: This channel disconnected while it was a primary ctrl channel (and other channels are open) -
                    //  finalize this channel, deactivate other channels
                    //Case 3: This channel disconnected while it was a non-primary ctrl channel (and other channels are open) -
                    //  finalize this channel, leave other channels alone
                    //Case 4: This channel disconnected while it was a data channel (and other channels are open, including a primary)
                    //  finalize this channel, set primary channel as a CTRL+DATA channel if this was the last data channel
                    //Case 5: This channel disconnected while it was inactive -
                    //  don't need to do anything!
                    //... inactive channels are counted as closed

                    devflags_t oldflags = flags;
                    flags = DEV_FLAGS_CLEAR;
                    flushRead();

                    if((oldflags & DEV_IS_ACTIVE) == 0) {
                        // Case 5
                    } else if((oldflags & (DEV_IS_CTRL | DEV_IS_DATA)) == (DEV_IS_CTRL | DEV_IS_DATA) || !xio.others_connected(this)) {
                        // Case 1
                        if((oldflags & (DEV_IS_CTRL | DEV_IS_DATA)) != (DEV_IS_CTRL | DEV_IS_DATA) || xio.others_connected(this)) {
                            rpt_exception(STAT_XIO_ASSERTION_FAILURE, NULL); // where is this supposed to go!?
                        }
                        controller_set_connected(false);
                    } else if((oldflags & (DEV_IS_CTRL | DEV_IS_PRIMARY)) == (DEV_IS_CTRL | DEV_IS_PRIMARY)) {
                        // Case 2
                        xio.deactivate_all_channels();
                    } else if((oldflags & (DEV_IS_CTRL)) == DEV_IS_CTRL) {
                        // Case 3
                    } else if((oldflags & (DEV_IS_DATA)) == DEV_IS_DATA) {
                        // Case 4
                        xio.remove_data_from_primary();
                    }
                } // flags & DEV_IS_CONNECTED
            }
        });
    };

    virtual int16_t readchar() final {
		return _dev->readByte();					// readByte calls the USB endpoint's read function
	};
    
    virtual void flushRead() final {
        return _dev->flushRead();
    }

    virtual int16_t write(const uint8_t *buffer, int16_t len) final {
        return _dev->write(buffer, len);
    }
};

// ALLOCATIONS
// Declare a device wrapper class for USB0 and USB1 and put pointer in an array as elements 0 and 1
xioDeviceWrapper<decltype(&SerialUSB)> serialUSB0Wrapper {&SerialUSB, (DEV_CAN_READ | DEV_CAN_WRITE | DEV_CAN_BE_CTRL | DEV_CAN_BE_DATA)};
xioDeviceWrapper<decltype(&SerialUSB1)> serialUSB1Wrapper {&SerialUSB1, (DEV_CAN_READ | DEV_CAN_WRITE | DEV_CAN_BE_CTRL | DEV_CAN_BE_DATA)};

// Define the xio singleton (and initilize it)
xio_t xio = { &serialUSB0Wrapper, &serialUSB1Wrapper };

/**** CODE ****/

/*
 * xio_init()
 *
 *	A lambda function closure is provided for trapping connection state changes from USB devices.
 *	The function is installed as a callback from the lower USB layers. It is called only on edges
 *	(connect/disconnect transitions). 'Connected' is true if the USB channel has just connected,
 *	false if it has just disconnected. It is only called on an edge — when it changes - so you
 *	shouldn't see two back-to-back connected=true calls with the same callback.
 *
 *	See here for some good info on lambda functions in C++
 *	http://www.cprogramming.com/c++11/c++11-lambda-closures.html
 */

void xio_init()
{
    // Nothing left to do here, move along...
}

stat_t xio_test_assertions()
{
	if ((xio.magic_start != MAGICNUM) || (xio.magic_end != MAGICNUM)) {
		return (STAT_XIO_ASSERTION_FAILURE);
	}
	return (STAT_OK);
}

/*
 * write() - write a buffer to a device
 */

size_t write(uint8_t *buffer, size_t size)
{
    return xio.write(buffer, size);
}

/*
 * readline() - read a complete line from a device
 *
 *	Defers to xio.readline().
 */

char_t *readline(devflags_t &flags, uint16_t &size)
{
    return xio.readline(flags, size);
}

void xio_flush_device(devflags_t flags) {
    // TODO -- this function
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * xio_set_spi() = 0=disable, 1=enable
 */
//stat_t xio_set_spi(nvObj_t *nv)
//{
//	xio.spi_state = (uint8_t)nv->value;
//
//#ifdef __ARM
//	if (fp_EQ(nv->value, SPI_ENABLE)) {
//		spi_miso_pin.setMode(kOutput);
//		spi_mosi_pin.setMode(kOutput);
//		spi_sck_pin.setMode(kOutput);
//
//    } else if (fp_EQ(nv->value, SPI_DISABLE)) {
//		spi_miso_pin.setMode(kInput);
//		spi_mosi_pin.setMode(kInput);
//		spi_sck_pin.setMode(kInput);
//	}
//#endif
//	return (STAT_OK);
//}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_spi[] PROGMEM = "[spi] SPI state%20d [0=disabled,1=enabled]\n";
void xio_print_spi(nvObj_t *nv) { text_print_ui8(nv, fmt_spi);}

#endif // __TEXT_MODE
