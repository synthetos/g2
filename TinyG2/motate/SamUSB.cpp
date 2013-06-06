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
	USBProxy_t USBProxy;

	template< typename parent >
	void USBDeviceHardware<parent>::interrupt() {
		// End of bus reset
		if (Is_udd_reset())
		{
			TRACE_CORE(printf(">>> End of Reset\r\n");)

			// Reset USB address to 0
			udd_configure_address(0);
			udd_enable_address();

			// Configure EP 0
			UDD_InitEP(0, EP_TYPE_CONTROL);
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

						UDD_InitEP(0, EP_TYPE_CONTROL);

						_configuration = setup.valueLow;

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
					_set_interface = setup.valueLow;
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
