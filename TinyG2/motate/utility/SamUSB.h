/*
  utility/SamUSB.h - Library for the Motate system
  http://tinkerin.gs/

  Copyright (c) 2013 Robert Giseburt

	This file is part of the Motate Library.

	This file ("the software") is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License, version 2 as published by the
	Free Software Foundation. You should have received a copy of the GNU General Public
	License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.

	As a special exception, you may use this file as part of a software library without
	restriction. Specifically, if other files instantiate templates or use macros or
	inline functions from this file, or you compile this file and link it with  other
	files to produce an executable, this file does not by itself cause the resulting
	executable to be covered by the GNU General Public License. This exception does not
	however invalidate any other reasons why the executable file might be covered by the
	GNU General Public License.

	THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
	WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
	SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
	OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef SAMUSB_ONCE
#define SAMUSB_ONCE

#include "MotateUSBHelpers.h"

#include "sam.h"

// TEMPORARY!! Grab samlib stuff.

#include "chip.h"

namespace Motate {


	/*** ENDPOINT CONFIGURATION ***/

	typedef uint32_t EndpointBufferSettings_t;

	enum USBEndpointBufferSettingsFlags_t {
		// null endpoint is all zeros
		kEndpointBufferNull            = -1,

		// endpoint direction
		kEndpointBufferOutputFromHost  = UOTGHS_DEVEPTCFG_EPDIR_OUT,
		kEndpointBufferInputToHost     = UOTGHS_DEVEPTCFG_EPDIR_IN,

		// This mask is not part of the public interface:
		kEndpointBufferDirectionMask   = UOTGHS_DEVEPTCFG_EPDIR,

		// buffer sizes
		kEnpointBufferSizeUpTo8        = UOTGHS_DEVEPTCFG_EPSIZE_8_BYTE,
		kEnpointBufferSizeUpTo16       = UOTGHS_DEVEPTCFG_EPSIZE_16_BYTE,
		kEnpointBufferSizeUpTo32       = UOTGHS_DEVEPTCFG_EPSIZE_32_BYTE,
		kEnpointBufferSizeUpTo64       = UOTGHS_DEVEPTCFG_EPSIZE_64_BYTE,
		kEnpointBufferSizeUpTo128      = UOTGHS_DEVEPTCFG_EPSIZE_128_BYTE,
		kEnpointBufferSizeUpTo256      = UOTGHS_DEVEPTCFG_EPSIZE_256_BYTE,
		kEnpointBufferSizeUpTo512      = UOTGHS_DEVEPTCFG_EPSIZE_512_BYTE,
		kEnpointBufferSizeUpTo1024     = UOTGHS_DEVEPTCFG_EPSIZE_1024_BYTE,

		// This mask is not part of the public interface:
		kEnpointBufferSizeMask         = UOTGHS_DEVEPTCFG_EPSIZE_Msk,

		// buffer "blocks" -- 2 == "ping pong"
		// Note that there must be one, or this is a null endpoint.
		kEndpointBufferBlocks1         = UOTGHS_DEVEPTCFG_EPBK_1_BANK,
		kEndpointBufferBlocksUpTo2     = UOTGHS_DEVEPTCFG_EPBK_2_BANK,
		kEndpointBufferBlocksUpTo3     = UOTGHS_DEVEPTCFG_EPBK_3_BANK,

		// This mask is not part of the public interface:
		kEndpointBufferBlocksMask      = UOTGHS_DEVEPTCFG_EPBK_Msk,

		// endpoint types (mildly redundant from the config)
		kEndpointBufferTypeControl     = UOTGHS_DEVEPTCFG_EPTYPE_CTRL,
		kEndpointBufferTypeIsochronous = UOTGHS_DEVEPTCFG_EPTYPE_ISO,
		kEndpointBufferTypeBulk        = UOTGHS_DEVEPTCFG_EPTYPE_BLK,
		kEndpointBufferTypeInterrupt   = UOTGHS_DEVEPTCFG_EPTYPE_INTRPT,

		// This mask is not part of the public interface:
		kEndpointBufferTypeMask        = UOTGHS_DEVEPTCFG_EPTYPE_Msk
	};

	
	/*** PROXY ***/

	struct USBProxy_t {
		bool (*sendDescriptorOrConfig)(Setup_t &setup);
		bool (*handleNonstandardRequest)(Setup_t &setup);
		const uint8_t (*getEndpointCount)(uint8_t &firstEnpointNum);
		const EndpointBufferSettings_t (*getEndpointConfig)(const uint8_t endpoint);
	};
	extern USBProxy_t USBProxy;


	/*** STRINGS ***/

	const uint16_t *getUSBVendorString(uint8_t &length) ATTR_WEAK;
	const uint16_t *getUSBProductString(uint8_t &length) ATTR_WEAK;

	// We break the rules here, sortof, by providing a macro shortcut that gets used in userland.
	// I apologize, but this also opens it up to later optimization without changing user code.
#define MOTATE_SET_USB_VENDOR_STRING(...)\
	const uint16_t MOTATE_USBVendorString[] = __VA_ARGS__;\
	const uint16_t *Motate::getUSBVendorString(uint8_t &length) {\
		length = sizeof(MOTATE_USBVendorString);\
		return MOTATE_USBVendorString;\
	}

#define MOTATE_SET_USB_PRODUCT_STRING(...)\
	const uint16_t MOTATE_USBProductString[] = __VA_ARGS__;\
	const uint16_t *Motate::getUSBProductString(uint8_t &length) {\
		length = sizeof(MOTATE_USBProductString);\
		return MOTATE_USBProductString;\
	}


	/*** USBDeviceHardware ***/

	// This is the stub for the interrupt
	void _usb_interrupt();

	extern uint32_t _inited;
	extern uint32_t _configuration;

	// USBDeviceHardware actually talks to the hardware, and marshalls data to/from the interfaces.
	template< typename parent >
	class USBDeviceHardware
	{
		parent* const parent_this;

	public:

		// Init
		USBDeviceHardware() : parent_this(static_cast< parent* >(this))
		{
			USBProxy.sendDescriptorOrConfig   = parent::sendDescriptorOrConfig;
			USBProxy.handleNonstandardRequest = parent::handleNonstandardRequest;
			USBProxy.getEndpointConfig        = parent::getEndpointConfig;
			USBProxy.getEndpointCount         = parent::getEndpointCount;

			UDD_SetStack(&_usb_interrupt);

			if (UDD_Init() == 0UL)
			{
				_inited = 1UL;
			}
			_configuration = 0UL;
		};

		static bool attach() {
			if (_inited) {
				UDD_Attach();
				_configuration = 0;
				return true;
			}
			return false;
		};

		static bool detach() {
			if (_inited) {
				UDD_Detach();
				return true;
			}
			return false;
		};

		static int32_t available(const uint8_t ep) {
			return UDD_FifoByteCount(ep & 0xF);
		}

		static int32_t readByte(const uint8_t ep) {
			uint8_t c;
			if (read(ep & 0xF, &c, 1) == 1)
				return c;
			return -1;
		};

		/* Data is const. The pointer to data is not. */
		static int32_t read(const uint8_t ep, const uint8_t * buffer, uint16_t length) {
//			if (!_usbConfiguration || len < 0)
//				return -1;
//
//			LockEP lock(ep);
			uint32_t n = UDD_FifoByteCount(ep & 0xF);
			length = min(n, length);
			n = length;
			uint8_t* dst = (uint8_t*)buffer;
			while (n--)
				*dst++ = UDD_Recv8(ep & 0xF);
			if (length && !UDD_FifoByteCount(ep & 0xF)) // release empty buffer
				UDD_ReleaseRX(ep & 0xF);
			
			return length;
		};

		/* Data is const. The pointer to data is not. */
		static int32_t write(const uint8_t ep, const uint8_t * buffer, uint16_t length) {
			uint32_t n;
			int r = length;
			const uint8_t* data = (const uint8_t*)buffer;

//			if (!_usbConfiguration)
//			{
//				TRACE_CORE(printf("pb conf\n\r");)
//				return -1;
//			}

			while (length)
			{
				if(ep==0) n = EP0_SIZE;
				else n =  EPX_SIZE;
				if (n > length)
					n = length;
				length -= n;

				UDD_Send(ep & 0xF, data, n);
				data += n;
			}

			return r;
		};

		// This is static to be called from the interrupt.
		static void sendString(const uint8_t string_num) {
			uint8_t length = 0;
			const uint16_t *string = 0;
			if (kManufacturerStringId == string_num && getUSBVendorString) {
				string = getUSBVendorString(length);
			} else
			if (kProductStringId == string_num && getUSBProductString) {
				string = getUSBProductString(length);
			}

			USBDescriptorStringHeader_t string_header(length);
			write(0, (const uint8_t *)(&string_header), sizeof(string_header));
			if (string)
				write(0, (const uint8_t *)(string), length);
		};
	}; //class USBDeviceHardware
}

#endif
//SAMUSB_ONCE
