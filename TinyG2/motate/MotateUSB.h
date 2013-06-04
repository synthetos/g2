/*
 utility/MotateUSB.h - Library for the Motate system
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

#ifndef MOTATEUSB_ONCE
#define MOTATEUSB_ONCE

#include "utility/MotateUSBHelpers.h"

#ifdef __AVR_XMEGA__

#include <utility/AvrXUSB.h>

#else

#ifdef __AVR__
#include <utility/AvrUSB.h>
#endif

#endif

#ifdef __SAM3X8E__
#include <utility/SamUSB.h>
#endif

namespace Motate {

	/* ############################################ */
	/* #                                          # */
	/* #             USB Device (etc.)            # */
	/* #                                          # */
	/* ############################################ */

	struct USBSettings_t {
		const uint16_t vendorID;
		const uint16_t productID;
		const float productVersion;
		const uint8_t  attributes;
		const uint16_t powerConsumption;
	};

	extern const USBSettings_t USBSettings;

	// You will need to have something like this in your project (in a .cpp file):
	//
	// const USBSettings_t Motate::USBSettings = {
	// 	/* vendorID         = */ 0x1d50,
	// 	/* productID        = */ 0x606d,
	// 	/* productVersion   = */ 0.1,
	// 	/* attributes       = */ kUSBConfigAttributeSelfPowered,
	// 	/* powerConsumption = */ 500
	// };

	// USBController is our primary controller class, and "owns" the interfaces.
	// USBController actually talks to the hardware, and marshalls data to/from the interfaces.
	// There should only be one USBController per hardware USB device -- there will almost always be just one.
	template<class interface0type, class interface1type = USBNullInterface, class interface2type = USBNullInterface>
	class USBDevice :
		public USBDeviceHardware< USBDevice<interface0type, interface1type, interface2type> >,
		public USBMixin<interface0type, interface1type, interface2type, 0>,
		public USBMixin<interface0type, interface1type, interface2type, 1>,
		public USBMixin<interface0type, interface1type, interface2type, 2>
	{
	public:
		// Shortcut typedefs
		typedef USBDevice<interface0type, interface1type, interface2type> _this_type;

		typedef USBDeviceHardware< USBDevice<interface0type, interface1type, interface2type> > _hardware_type;
		
		typedef USBMixin<interface0type, interface1type, interface2type, 0> _mixin_0_type;
		typedef USBMixin<interface0type, interface1type, interface2type, 1> _mixin_1_type;
		typedef USBMixin<interface0type, interface1type, interface2type, 2> _mixin_2_type;
		typedef USBDefaultDescriptor<interface0type, interface1type, interface2type> _descriptor_type;
		typedef USBDescriptorConfiguration_t<interface0type, interface1type, interface2type> _config_type;

		// Keep track of the endpoint usage
		// Endpoint zero is the control interface, and is owned by nobody.
		static const uint8_t _interface_0_first_endpoint = 1;
		static const uint8_t _interface_1_first_endpoint = _interface_0_first_endpoint + _mixin_0_type::endpoints_used;
		static const uint8_t _interface_2_first_endpoint = _interface_1_first_endpoint + _mixin_1_type::endpoints_used;
		static const uint8_t _total_endpoints_used       = _interface_2_first_endpoint + _mixin_2_type::endpoints_used;

		// Keep track of endpoint descriptor size
		static const uint16_t _interface_0_descriptor_size = _mixin_0_type::descriptor_size;
		static const uint16_t _interface_1_descriptor_size = _mixin_1_type::descriptor_size;
		static const uint16_t _interface_2_descriptor_size = _mixin_2_type::descriptor_size;
		static const uint16_t _total_interface_descriptor_size = _interface_0_descriptor_size + _interface_1_descriptor_size + _interface_2_descriptor_size;
				
		// Init
		USBDevice() :
			_hardware_type(),
			_mixin_0_type(*this, _interface_0_first_endpoint),
			_mixin_1_type(*this, _interface_1_first_endpoint),
			_mixin_2_type(*this, _interface_2_first_endpoint)
		{
			// USBDeviceHardware should handle all of the rest of the init
		};

		static void sendDescriptor() {
			const _descriptor_type descriptor(USBSettings.vendorID, USBSettings.productID, USBFloatToBCD(USBSettings.productVersion));
			_this_type::write(0, (const uint8_t *)(&descriptor), sizeof(_descriptor_type));
		}

		static void sendConfig() {
			const _config_type config(USBSettings.attributes, USBSettings.powerConsumption);
			_this_type::write(0, (const uint8_t *)(&config), sizeof(_config_type));
		}

	};


	// Declare the base (Null) USBMixin
	// We use template specialization (later) on a combination of *one* of the three interfaces,
	// along with the position to expose different content into the USBDevice.
	template < typename interface0type, typename interface1type, typename interface2type, int position >
	struct USBMixin {
		static const uint8_t endpoints_used = 0;
		typedef USBDevice<interface0type, interface1type, interface2type> usb_parent_type;
		USBMixin (usb_parent_type &usb_parent,
				  const uint8_t new_endpoint_offset
				) {};

		static bool isNull() { return true; };
	};

	template < typename interface0type, typename interface1type, typename interface2type >
	struct USBDefaultDescriptor : USBDescriptorDevice_t {
		USBDefaultDescriptor(const uint16_t vendorID, const uint16_t productID, const uint16_t productVersionBCD) :
			USBDescriptorDevice_t(
								  /*    USBSpecificationBCD = */ USBFloatToBCD(2.0),
								  /*                  Class = */ kNoDeviceClass,
								  /*               SubClass = */ kNoDeviceSubclass,
								  /*               Protocol = */ kNoDeviceProtocol,

								  /*          Endpoint0Size = */ 64, /* !!!!!!!!!!! FIXME */

								  /*               VendorID = */ vendorID,
								  /*              ProductID = */ productID,
								  /*          ReleaseNumber = */ productVersionBCD,

								  /*   ManufacturerStrIndex = */ kManufacturerStringId,
								  /*        ProductStrIndex = */ kProductStringId,
								  /*      SerialNumStrIndex = */ kSerialNumberId,

								  /* NumberOfConfigurations = */ 1  /* !!!!!!!!!!! FIXME */
			)
		{};
	};
}

#endif
// MOTATEUSB_ONCE
