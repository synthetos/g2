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

// This is used to store the buffer sizes -- MOVE TO THE NAMESPACE
extern uint16_t enpointSizes[10];

namespace Motate {

	/*** ENDPOINT CONFIGURATION ***/

	typedef uint32_t EndpointBufferSettings_t;

	enum USBEndpointBufferSettingsFlags_t {
		// null endpoint is all zeros
		kEndpointBufferNull            = 0,

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

	// Convert from number to EndpointBufferSettings_t
	// This should optimize out.
	static const EndpointBufferSettings_t getBufferSizeFlags(const uint16_t size) {
		if (size > 512) {
			return kEnpointBufferSizeUpTo1024;
		} else if (size > 128) {
			return kEnpointBufferSizeUpTo512;
		} else if (size > 64) {
			return kEnpointBufferSizeUpTo128;
		} else if (size > 32) {
			return kEnpointBufferSizeUpTo64;
		} else if (size > 16) {
			return kEnpointBufferSizeUpTo32;
		} else if (size > 8) {
			return kEnpointBufferSizeUpTo16;
		} else {
			return kEnpointBufferSizeUpTo8;
		}
		return kEndpointBufferNull;
	};

	/*** PROXY ***/

	struct USBProxy_t {
		bool (*sendDescriptorOrConfig)(Setup_t &setup);
		bool (*handleNonstandardRequest)(Setup_t &setup);
		const uint8_t (*getEndpointCount)(uint8_t &firstEnpointNum);
		uint16_t (*getEndpointSize)(const uint8_t &endpointNum, const bool otherSpeed);
		const EndpointBufferSettings_t (*getEndpointConfig)(const uint8_t endpoint, const bool otherSpeed);
	};
	extern USBProxy_t USBProxy;


	/*** STRINGS ***/

	const uint16_t *getUSBVendorString(int16_t &length) ATTR_WEAK;
	const uint16_t *getUSBProductString(int16_t &length) ATTR_WEAK;
	const uint16_t *getUSBSerialNumberString(int16_t &length) ATTR_WEAK;

	// We break the rules here, sortof, by providing a macro shortcut that gets used in userland.
	// I apologize, but this also opens it up to later optimization without changing user code.
#define MOTATE_SET_USB_VENDOR_STRING(...)\
	const uint16_t MOTATE_USBVendorString[] = __VA_ARGS__;\
	const uint16_t *Motate::getUSBVendorString(int16_t &length) {\
		length = sizeof(MOTATE_USBVendorString);\
		return MOTATE_USBVendorString;\
	}

#define MOTATE_SET_USB_PRODUCT_STRING(...)\
	const uint16_t MOTATE_USBProductString[] = __VA_ARGS__;\
	const uint16_t *Motate::getUSBProductString(int16_t &length) {\
		length = sizeof(MOTATE_USBProductString);\
		return MOTATE_USBProductString;\
	}

#define MOTATE_SET_USB_SERIAL_NUMBER_STRING(...)\
    const uint16_t MOTATE_USBSerialNumberString[] = __VA_ARGS__;\
    const uint16_t *Motate::getUSBSerialNumberString(int16_t &length) {\
        length = sizeof(MOTATE_USBSerialNumberString);\
        return MOTATE_USBSerialNumberString;\
    }

#define MOTATE_SET_USB_SERIAL_NUMBER_STRING_FROM_CHIPID()\
    const uint16_t *Motate::getUSBSerialNumberString(int16_t &length) {\
        const uint16_t *uuid = readUniqueIdString();\
        length = UNIQUE_ID_STRING_LEN * sizeof(uint16_t);\
        return uuid;\
    }

	// This needs to be provided in the hardware file
	const uint16_t *getUSBLanguageString(int16_t &length);


	/*** USBDeviceHardware ***/

	extern int32_t _getEndpointBufferCount(const uint8_t endpoint);
	extern int16_t _readFromControlEndpoint(const uint8_t endpoint, uint8_t* data, int16_t len, bool continuation);
	extern int16_t _readFromEndpoint(const uint8_t endpoint, uint8_t* data, int16_t len);
	extern int16_t _readByteFromEndpoint(const uint8_t endpoint);
	extern int16_t _sendToEndpoint(const uint8_t endpoint, const uint8_t* data, int16_t length);
	extern int16_t _sendToControlEndpoint(const uint8_t endpoint, const uint8_t* data, int16_t length, bool continuation);
	extern void _unfreezeUSBClock();
	extern void _waitForUsableUSBClock();
	extern void _enableResetInterrupt();
	extern void _resetEndpointBuffer(const uint8_t endpoint);
	extern void _freezeUSBClock();
	extern void _flushEndpoint(uint8_t endpoint);
    extern void _flushReadEndpoint(uint8_t endpoint);

	extern uint32_t _inited;
	extern uint32_t _configuration;

	// USBDeviceHardware actually talks to the hardware, and marshalls data to/from the interfaces.
	template< typename parent >
	class USBDeviceHardware
	{
		parent* const parent_this;

	public:

		static const uint8_t master_control_endpoint = 0;

		static void _init() {
			uint32_t endpoint;

            // FORCE disable the USB hardware:
            UOTGHS->UOTGHS_CTRL &= ~(UOTGHS_CTRL_USBE);

			for (endpoint = 0; endpoint < 10; ++endpoint)
			{
				_resetEndpointBuffer(endpoint);
			}

			// Enables the USB Clock
//			if (ID_UOTGHS < 32) {
//				uint32_t id_mask = 1u << (ID_UOTGHS);
//				if ((PMC->PMC_PCSR0 & id_mask) != id_mask) {
//					PMC->PMC_PCER0 = id_mask;
//				}
//#if (SAM3S_SERIES || SAM3XA_SERIES || SAM4S_SERIES)
//			} else {
				uint32_t id_mask = 1u << (ID_UOTGHS - 32);
				if ((PMC->PMC_PCSR1 & id_mask) != id_mask) {
					PMC->PMC_PCER1 = id_mask;
				}
//#endif
//			}

			// Enable UPLL clock.
			PMC->CKGR_UCKR = CKGR_UCKR_UPLLCOUNT(3) | CKGR_UCKR_UPLLEN;

			/* Wait UTMI PLL Lock Status */
			while (!(PMC->PMC_SR & PMC_SR_LOCKU))
				;


			// Switch UDP (USB) clock source selection to UPLL clock.
			// Clock divisor is set to 1 (USBDIV + 1)
			PMC->PMC_USB = PMC_USB_USBS | PMC_USB_USBDIV(/*USBDIV = */0);

			// Enable UDP (USB) clock.
# if (SAM3S_SERIES || SAM4S_SERIES)
			PMC->PMC_SCER = PMC_SCER_UDP;
# else
			PMC->PMC_SCER = PMC_SCER_UOTGCLK;
# endif

			// Configure interrupts
			NVIC_SetPriority((IRQn_Type) ID_UOTGHS, 0UL);
			NVIC_EnableIRQ((IRQn_Type) ID_UOTGHS);

			// Always authorize asynchrone USB interrupts to exit from sleep mode
			//   for SAM3 USB wake up device except BACKUP mode
			// Set the wake-up inputs for fast startup mode registers (event generation).
			/* -- DISBALED --
			 ul_inputs &= (~ PMC_FAST_STARTUP_Msk);
			 PMC->PMC_FSMR |= PMC_FSMR_USBAL;
			 */

			// Disable external OTG_ID pin (ignored by USB)
			UOTGHS->UOTGHS_CTRL &= ~UOTGHS_CTRL_UIDE;
			//! Force device mode
			UOTGHS->UOTGHS_CTRL |= UOTGHS_CTRL_UIMOD;

			// Enable USB hardware
			//  Enable OTG pad
			UOTGHS->UOTGHS_CTRL |= UOTGHS_CTRL_OTGPADE;
			//  Enable USB macro
			UOTGHS->UOTGHS_CTRL |= UOTGHS_CTRL_USBE;
			//  Unfreeze internal USB clock
			_unfreezeUSBClock();

			// Check USB clock
			//_waitForUsableUSBClock();

			// Enable High Speed
			//  Disable "Forced" Low Speed first..
			UOTGHS->UOTGHS_DEVCTRL &= ~UOTGHS_DEVCTRL_LS;
			//  Then enable High Speed
			/* UOTGHS_DEVCTRL_SPDCONF_NORMAL means:
			 * "The peripheral starts in full-speed mode and performs a high-speed reset to switch to the high-speed mode if
			 *  the host is high-speed capable."
			 */
			UOTGHS->UOTGHS_DEVCTRL = (UOTGHS->UOTGHS_DEVCTRL & ~ UOTGHS_DEVCTRL_SPDCONF_Msk) | UOTGHS_DEVCTRL_SPDCONF_NORMAL;

			// // otg_ack_vbus_transition();
			// UOTGHS->UOTGHS_SCR = UOTGHS_SCR_VBUSTIC
			// Force Vbus interrupt in case of Vbus always with a high level
			// This is possible with a short timing between a Host mode stop/start.
			/*
			 if (UOTGHS->UOTGHS_SR & UOTGHS_SR_VBUS) {
			 UOTGHS->UOTGHS_SFR = UOTGHS_SFR_VBUSTIS;
			 }
			 UOTGHS->UOTGHS_CTRL |= UOTGHS_CTRL_VBUSTE;
			 */
			_freezeUSBClock();

		};

		static void _attach() {
			//		irqflags_t flags = cpu_irq_save();

			_unfreezeUSBClock();

			// Check USB clock because the source can be a PLL
			_waitForUsableUSBClock();

			// Authorize attach if Vbus is present
			UOTGHS->UOTGHS_DEVCTRL &= ~UOTGHS_DEVCTRL_DETACH;

			// Enable USB line events
			_enableResetInterrupt();
			//	udd_enable_sof_interrupt();

			//		cpu_irq_restore(flags);

		};

		static void _detach() {
			UOTGHS->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_DETACH;
		};

		// Init
		USBDeviceHardware() : parent_this(static_cast< parent* >(this))
		{
			USBProxy.sendDescriptorOrConfig   = parent::sendDescriptorOrConfig;
			USBProxy.handleNonstandardRequest = parent::handleNonstandardRequest;
			USBProxy.getEndpointConfig        = parent::getEndpointConfig;
			USBProxy.getEndpointCount         = parent::getEndpointCount;
			USBProxy.getEndpointSize          = parent::getEndpointSize;

			USBDeviceHardware::_init();
			_inited = 1UL;
			_configuration = 0UL;
		};

		static bool attach() {
			if (_inited) {
				_attach();
				_configuration = 0;
				return true;
			}
			return false;
		};

		static bool detach() {
			if (_inited) {
				_detach();
				return true;
			}
			return false;
		};

		static int16_t availableToRead(const uint8_t endpoint) {
			return _getEndpointBufferCount(endpoint);
		}

		static int16_t readByte(const uint8_t endpoint) {
			return _readByteFromEndpoint(endpoint);
		};

		/* Data is const. The pointer to data is not. */
		static int16_t read(const uint8_t endpoint, uint8_t *buffer, int16_t length) {
			if (!_configuration || length < 0)
				return -1;
//
//			LockEP lock(ep);
			return _readFromEndpoint(endpoint, buffer, length);
		};

		/* Data is const. The pointer to data is not. */
		static int16_t write(const uint8_t endpoint, const uint8_t * buffer, int16_t length) {
			if (!_configuration || length < 0)
				return -1;

			return _sendToEndpoint(endpoint, buffer, length);
		};

		static void flush(const uint8_t endpoint) {
			_flushEndpoint(endpoint);
		};
        
        static void flushRead(const uint8_t endpoint) {
            _flushReadEndpoint(endpoint);
        }

		/* Data is const. The pointer to data is not. */
		static int16_t readFromControl(const uint8_t endpoint, uint8_t *buffer, int16_t length) {
			if (!_configuration || length < 0)
				return -1;
            bool continuation = false;
            int16_t to_read = length;
            while (to_read > 0) {
                int16_t amt_read = _readFromControlEndpoint(endpoint, buffer, to_read, continuation);
                to_read -= amt_read;
                buffer += amt_read;
                continuation = true;
            }
			return length;
//            return _readFromControlEndpoint(endpoint, buffer, to_read, continuation);
		};

		/* Data is const. The pointer to data is not. */
		static int16_t writeToControl(const uint8_t endpoint, const uint8_t *buffer, int16_t length) {
            bool continuation = false;
            int16_t to_send = length;
            while (to_send > 0) {
                int16_t sent = _sendToControlEndpoint(endpoint, buffer, to_send, continuation);
                to_send -= sent;
                buffer += sent;
                continuation = true;
            }
            return length;
		};

		// This is static to be called from the interrupt.
		static void sendString(const uint8_t stringNum, int16_t maxLength) {
			int16_t length = 0;
			int16_t to_send;
			const uint16_t *string;
			if (0 == stringNum) {
				// Language ID
				string = getUSBLanguageString(length);
			}
			else
            if (kManufacturerStringId == stringNum && getUSBVendorString) {
				string = getUSBVendorString(length);
			} else
			if (kProductStringId == stringNum && getUSBProductString) {
				string = getUSBProductString(length);
			} else
            if (kSerialNumberId == stringNum && getUSBSerialNumberString) {
                string = getUSBSerialNumberString(length);
			} else
				return; // This is wrong, but works...?

			USBDescriptorStringHeader_t string_header(length);

			// Make sure we don't send more than maxLength!
			// If the string is longer, then the host will have to ask again,
			//  with a bigger maxLength, and probably will.
            
            // The optimizer does this, but I'll do it explicitly for readability:
            
            int16_t header_size = sizeof(string_header);
            
			to_send = string_header.Header.Size;
			if (to_send > maxLength)
				to_send = maxLength;

			to_send -= _sendToControlEndpoint(0, (const uint8_t *)(&string_header), to_send > header_size ? header_size : to_send, /*continuation = */false);
            
            const uint8_t * buffer = (const uint8_t *)(string);
            bool continuation = false;
			while (to_send > 0) {
                int16_t sent = _sendToControlEndpoint(0, buffer, to_send, continuation);
				to_send -= sent;
                buffer += sent;
                continuation = true;
            }
		};
        
        // Request the speed that the device is communicating at. It is unclear at what point this becomes valid,
        // but it's assumed that this would be decided *before* he configuration and descriptors are sent, and
        // the endpoints (other than 0) are configured.
        static const USBDeviceSpeed_t getDeviceSpeed() {
            switch (UOTGHS->UOTGHS_SR & UOTGHS_SR_SPEED_Msk) {
                case UOTGHS_SR_SPEED_HIGH_SPEED:
                    return kUSBDeviceHighSpeed;
                    
                case UOTGHS_SR_SPEED_FULL_SPEED:
                    return kUSBDeviceFullSpeed;
                    
                case UOTGHS_SR_SPEED_LOW_SPEED:
                    return kUSBDeviceLowSpeed;
                    
                    // This shouldn't be possible, but "3" is a reserved value...
                default:
                    return kUSBDeviceLowSpeed;
            }
        }

		static uint16_t getEndpointSizeFromHardware(const uint8_t &endpoint, const bool otherSpeed) {
			if (endpoint == 0) {
                if (getDeviceSpeed() == kUSBDeviceLowSpeed) {
                    return 8;
                }
				return 64;
			}
            
            // Indicate that we didn't set one...
			return 0;
		};

		static const EndpointBufferSettings_t getEndpointConfigFromHardware(const uint8_t endpoint) {
			if (endpoint == 0)
			{
				return getBufferSizeFlags(getEndpointSizeFromHardware(endpoint, false)) | kEndpointBufferBlocks1 | kEndpointBufferTypeControl;
			}
			return kEndpointBufferNull;
		};
	}; //class USBDeviceHardware
}

#endif
//SAMUSB_ONCE
