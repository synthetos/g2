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

#ifdef __SAM3X8E__

#include "utility/SamUSB.h"

#define TRACE_CORE(x)

namespace Motate {
	const uint16_t kUSBControlEnpointSize = 0x10;
	const uint16_t kUSBNormalEnpointSize = 0x200;

	uint32_t _inited = 0;
	uint32_t _configuration = 0;
	uint32_t _set_interface = 0; // the interface set by the host
	uint8_t  _halted = 0; // Make this into a generic _flags?? -rg
	uint8_t  _remoteWakeupEnabled = 0;

	USBProxy_t USBProxy;

	/* ############################################# */
	/* #                                           # */
	/* #        HW-SPECIFIC ENDPOINT LIMITS        # */
	/* #                                           # */
	/* ############################################# */

	// Enpoint 3 config - max 1024b buffer, with two blocks
	// Enpoint 4 config - max 1024b buffer, with two blocks
	// Enpoint 5 config - max 1024b buffer, with two blocks
	// Enpoint 6 config - max 1024b buffer, with two blocks
	// Enpoint 7 config - max 1024b buffer, with two blocks
	// Enpoint 8 config - max 1024b buffer, with two blocks
	// Enpoint 9 config - max 1024b buffer, with two blocks
	static const EndpointBufferSettings_t _enforce_enpoint_limits(const uint8_t endpoint, EndpointBufferSettings_t config) {
		if (endpoint > 9)
			return kEndpointBufferNull;

		if (endpoint == 0) {
			if ((config & kEnpointBufferSizeMask) > kEnpointBufferSizeUpTo64)
				config = (config & ~kEnpointBufferSizeMask) | kEnpointBufferSizeUpTo64;

			config = (config & ~kEndpointBufferBlocksMask) | kEndpointBufferBlocks1;
		} else {

			// Enpoint 1 config - max 1024b buffer, with three blocks
			// Enpoint 2 config - max 1024b buffer, with three blocks

			if ((config & kEnpointBufferSizeMask) > kEnpointBufferSizeUpTo1024)
				config = (config & ~kEnpointBufferSizeMask) | kEnpointBufferSizeUpTo1024;

			if (endpoint < 3) {
				if ((config & ~kEndpointBufferBlocksMask) > kEndpointBufferBlocksUpTo3)
					config = (config & ~kEndpointBufferBlocksMask) | kEndpointBufferBlocksUpTo3;
			} else {
				if ((config & ~kEndpointBufferBlocksMask) > kEndpointBufferBlocksUpTo2)
					config = (config & ~kEndpointBufferBlocksMask) | kEndpointBufferBlocksUpTo2;
			}
		}
		config |= UOTGHS_DEVEPTCFG_NBTRANS_1_TRANS | UOTGHS_DEVEPTCFG_ALLOC;

		return config;
	};

	// uint32_t since the registers are 32-bit, we'll handle the conversion on entry to the function
	void _hw_init_endpoint(uint32_t endpoint, const uint32_t configuration) {
		endpoint = endpoint & 0xF; // EP range is 0..9, hence mask is 0xF.

//		TRACE_UOTGHS_DEVICE(printf("=> UDD_InitEP : init EP %lu\r\n", ul_ep_nb);)
		uint32_t configuration_fixed = _enforce_enpoint_limits(endpoint, configuration);

		// Configure EP
		// If we get here, and it's a null endpoint, this will disable it.
		UOTGHS->UOTGHS_DEVEPTCFG[endpoint] = configuration_fixed;
		
		// Enable EP
		if (configuration_fixed != kEndpointBufferNull) {
			udd_enable_endpoint(endpoint);

			if (!Is_udd_endpoint_configured(endpoint)) {
	//			TRACE_UOTGHS_DEVICE(printf("=> UDD_InitEP : ERROR FAILED TO INIT EP %lu\r\n", ul_ep_nb);)
				while(1);
			}
		}
	}

	void _usb_interrupt() {
		// End of bus reset
		if (Is_udd_reset())
		{
			TRACE_CORE(printf(">>> End of Reset\r\n");)

			// Reset USB address to 0
			udd_configure_address(0);
			udd_enable_address();

			// Configure EP 0
			_hw_init_endpoint(0, USBProxy.getEndpointConfig(0));
			udd_enable_setup_received_interrupt(0);
			udd_enable_endpoint_interrupt(0);

			_configuration = 0;
			udd_ack_reset();
		}
/*
		if (Is_udd_endpoint_interrupt(CDC_RX))
		{
			udd_ack_out_received(CDC_RX);

			// Handle received bytes
			while (USBD_Available(CDC_RX))
				SerialUSB.accept();

			udd_ack_fifocon(CDC_RX);
		}
*/
		if (Is_udd_sof())
		{
			udd_ack_sof();
			//	USBD_Flush(CDC_TX); // jcb
		}

		// EP 0 Interrupt
		if (Is_udd_endpoint_interrupt(0) )
		{
			if (!UDD_ReceivedSetupInt())
			{
				return;
			}

			Setup_t setup;
			UDD_Recv(EP0, (uint8_t*)&setup, 8);
			UDD_ClearSetupInt();

			if (setup.isADeviceToHostRequest())
			{
				TRACE_CORE(puts(">>> EP0 Int: IN Request\r\n");)
				UDD_WaitIN();
			}
			else
			{
				TRACE_CORE(puts(">>> EP0 Int: OUT Request\r\n");)
				UDD_ClearIN();
			}

			bool ok = true;
			if (setup.isAStandardRequestType())
			{
				// Standard Requests
				if (setup.isAGetStatusRequest())
				{
					if( setup.isADeviceRequest() )
					{
						// Send the device status
						TRACE_CORE(puts(">>> EP0 Int: kGetStatus\r\n");)
						// Check current configuration for power mode (if device is configured)
						// TODO
						// Check if remote wake-up is enabled
						// TODO
						UDD_Send8(EP0, 0); // TODO
						UDD_Send8(EP0, 0);
					}
					// if( setup.isAnEndpointRequest() )
					else
					{
						// Send the endpoint status
						// Check if the endpoint if currently halted
						if( _halted == 1 )
							UDD_Send8(EP0, 1); // TODO
						else
							UDD_Send8(EP0, 0); // TODO
						UDD_Send8(EP0, 0);
					}
				}
				else if ( setup.isAClearFeatureRequest() )
				{
					// Check which is the selected feature
					if( setup.featureToSetOrClear() == Setup_t::kSetupDeviceRemoteWakeup )
					{
						// Enable remote wake-up and send a ZLP
						if( _remoteWakeupEnabled == 1 )
							UDD_Send8(EP0, 1);
						else
							UDD_Send8(EP0, 0);
						UDD_Send8(EP0, 0);
					}
					else // if( setup.featureToSetOrClear() == kSetupEndpointHalt )
					{
						_halted = 0;  // TODO
						UDD_Send8(EP0, 0);
						UDD_Send8(EP0, 0);
					}

				}
				else if (setup.isASetFeatureRequest())
				{
					// Check which is the selected feature
					if( setup.featureToSetOrClear() == Setup_t::kSetupDeviceRemoteWakeup )
					{
						// Enable remote wake-up and send a ZLP
						_remoteWakeupEnabled = 1;
						UDD_Send8(EP0, 0);
					}
					if( setup.featureToSetOrClear() == Setup_t::kSetupEndpointHalt )
					{
						// Halt endpoint
						_halted = 1;
						//USBD_Halt(USBGenericRequest_GetEndpointNumber(pRequest));
						UDD_Send8(EP0, 0);
					}
					if( setup.featureToSetOrClear() == Setup_t::kSetupTestMode )
					{
#if 0
						// 7.1.20 Test Mode Support, 9.4.9 SetFeature
						if( (setup.bmRequestType == 0 /*USBGenericRequest_DEVICE*/) &&
						   ((setup.wIndex & 0x000F) == 0) )
						{
							// the lower byte of wIndex must be zero
							// the most significant byte of wIndex is used to specify the specific test mode

							UOTGHS->UOTGHS_DEVIDR &= ~UOTGHS_DEVIDR_SUSPEC;
							UOTGHS->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_SPDCONF_HIGH_SPEED; // remove suspend ?

							Test_Mode_Support( (setup.wIndex & 0xFF00)>>8 );
						}
#endif
					}
				}
				else if (setup.isASetAddressRequest())
				{
					TRACE_CORE(puts(">>> EP0 Int: kSetAddress\r\n");)
					UDD_WaitIN();
					UDD_SetAddress(setup.valueLow());
				}
				else if (setup.isAGetDescriptorRequest())
				{
					TRACE_CORE(puts(">>> EP0 Int: kGetDescriptor\r\n");)
					ok = USBProxy.sendDescriptorOrConfig(setup);
				}
				else if (setup.isASetDescriptorRequest())
				{
					TRACE_CORE(puts(">>> EP0 Int: kSetDescriptor\r\n");)
					ok = false;
				}
				else if (setup.isAGetConfigurationRequest())
				{
					TRACE_CORE(puts(">>> EP0 Int: kGetConfiguration\r\n");)
					UDD_Send8(EP0, _configuration);
				}
				else if (setup.isASetConfigurationRequest())
				{
					if (setup.isADeviceRequest())
					{
						TRACE_CORE(printf(">>> EP0 Int: kSetConfiguration REQUEST_DEVICE %d\r\n", setup.wValueL);)

//						UDD_InitEndpoints(EndPoints, (sizeof(EndPoints) / sizeof(EndPoints[0])));

						uint8_t first_endpoint, total_endpoints;
						total_endpoints = USBProxy.getEndpointCount(first_endpoint);
						for (uint8_t ep = first_endpoint; ep < total_endpoints; ep++)
							_hw_init_endpoint(ep, USBProxy.getEndpointConfig(ep));

						_configuration = setup.valueLow();

#if 0
						// Enable interrupt for CDC reception from host (OUT packet)
						udd_enable_out_received_interrupt(CDC_RX);
						udd_enable_endpoint_interrupt(CDC_RX);
#endif
					}
					else
					{
						TRACE_CORE(puts(">>> EP0 Int: kSetConfiguration failed!\r\n");)
						ok = false;
					}
				}
				else if (setup.isAGetInterfaceRequest())
				{
					TRACE_CORE(puts(">>> EP0 Int: kGetInterface\r\n");)
					UDD_Send8(EP0, _set_interface);
				}
				else if (setup.isASetInterfaceRequest())
				{
					_set_interface = setup.valueLow();
					TRACE_CORE(puts(">>> EP0 Int: kSetInterface\r\n");)
				}
			}
			else
			{
				TRACE_CORE(puts(">>> EP0 Int: ClassInterfaceRequest\r\n");)

				UDD_WaitIN(); // Workaround: need tempo here, else CDC serial won't open correctly

				// Note: setup._wLength may hold the max length of transfer
				ok = USBProxy.handleNonstandardRequest(setup);
			}

			if (ok)
			{
				TRACE_CORE(puts(">>> EP0 Int: Send packet\r\n");)
				UDD_ClearIN();
			}
			else
			{
				TRACE_CORE(puts(">>> EP0 Int: Stall\r\n");)
				UDD_Stall();
			}
		}
	}

}

#endif
//__SAM3X8E__
