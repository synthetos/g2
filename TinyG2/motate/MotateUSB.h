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

#if defined(__SAM3X8E__) || defined(__SAM3X8C__)
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

	// USBDevice is our primary controller class, and "owns" the interfaces.
	// USBDevice actually talks to the hardware, and marshalls data to/from the interfaces.
	// There should only be one USBDevice per hardware USB device -- there will almost always be just one.
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
		typedef USBDefaultQualifier<interface0type, interface1type, interface2type> _qualifier_type;

		// Keep track of the endpoint usage
		static const uint8_t _interface_0_first_endpoint = 1; // TODO: Verify with mixin control endpoint usage.
		static const uint8_t _interface_1_first_endpoint = _interface_0_first_endpoint + _mixin_0_type::endpoints_used;
		static const uint8_t _interface_2_first_endpoint = _interface_1_first_endpoint + _mixin_1_type::endpoints_used;
		static const uint8_t _total_endpoints_used       = _interface_2_first_endpoint + _mixin_2_type::endpoints_used;
				
		// Init
		USBDevice() :
			_hardware_type(),
            _mixin_0_type(*this, _interface_0_first_endpoint, _config_type::_interface_0_number),
			_mixin_1_type(*this, _interface_1_first_endpoint, _config_type::_interface_1_number),
			_mixin_2_type(*this, _interface_2_first_endpoint, _config_type::_interface_2_number)
		{
			// USBDeviceHardware should handle all of the rest of the init
			_singleton = this;
		};

		static bool sendDescriptorOrConfig(Setup_t &setup) {
			const uint8_t type = setup.valueHigh();
			if (type == kConfigurationDescriptor) {
				sendConfig(setup.length(), /*other = */ setup.valueLow() == 2);
				return true;
			}
			else
			if (type == kOtherDescriptor) {
				sendConfig(setup.length(), /*other = */ true);
				return true;
			}
			else
			if (type == kDeviceDescriptor) {
				sendDescriptor(setup.length());
				return true;
			}
			else
			if (type == kDeviceQualifierDescriptor) {
				sendQualifierDescriptor(setup.length());
				return true;
			}
			else
			if (type == kStringDescriptor) {
				_this_type::sendString(setup.valueLow(), setup.length());
				return true;
			}
			else
			{
				return _mixin_0_type::sendSpecialDescriptorOrConfig(setup) ||
				       _mixin_1_type::sendSpecialDescriptorOrConfig(setup) ||
				       _mixin_2_type::sendSpecialDescriptorOrConfig(setup);
			}
			return false;
		};

		static void sendDescriptor(int16_t maxLength) {
			const _descriptor_type descriptor(USBSettings.vendorID, USBSettings.productID, USBFloatToBCD(USBSettings.productVersion), _hardware_type::getDeviceSpeed());
			int16_t length = sizeof(_descriptor_type);
            int16_t to_send = maxLength < length ? maxLength : length;
            const uint8_t *buffer = (const uint8_t *)(&descriptor);
            _this_type::writeToControl(0, buffer, to_send);
		};

		static void sendQualifierDescriptor(int16_t maxLength) {
			const _qualifier_type qualifier;
			int16_t length = sizeof(_qualifier_type);
            int16_t to_send = maxLength < length ? maxLength : length;
            const uint8_t *buffer = (const uint8_t *)(&qualifier);
            _this_type::writeToControl(0, buffer, to_send);
		};

		static void sendConfig(int16_t maxLength, const bool other) {
			const _config_type config(USBSettings.attributes, USBSettings.powerConsumption, _hardware_type::getDeviceSpeed(), other);
			int16_t length = sizeof(_config_type);
            int16_t to_send = maxLength < length ? maxLength : length;
            const uint8_t *buffer = (const uint8_t *)(&config);
            _this_type::writeToControl(0, buffer, to_send);
		};

		static bool handleNonstandardRequest(Setup_t &setup) {
			return _mixin_0_type::handleNonstandardRequestInMixin(setup) ||
			       _mixin_1_type::handleNonstandardRequestInMixin(setup) ||
			       _mixin_2_type::handleNonstandardRequestInMixin(setup);
		};

		static const EndpointBufferSettings_t getEndpointConfig(const uint8_t endpoint, const bool otherSpeed) {
			EndpointBufferSettings_t ebs = _hardware_type::getEndpointConfigFromHardware(endpoint);

			if (!_mixin_0_type::isNull() && ebs == kEndpointBufferNull)
				ebs = _mixin_0_type::getEndpointConfigFromMixin(endpoint, _hardware_type::getDeviceSpeed(), otherSpeed);
			if (!_mixin_1_type::isNull() && ebs == kEndpointBufferNull)
				ebs = _mixin_1_type::getEndpointConfigFromMixin(endpoint, _hardware_type::getDeviceSpeed(), otherSpeed);
			if (!_mixin_2_type::isNull() && ebs == kEndpointBufferNull)
				ebs = _mixin_2_type::getEndpointConfigFromMixin(endpoint, _hardware_type::getDeviceSpeed(), otherSpeed);
			return ebs;
		};

		static const uint8_t getEndpointCount(uint8_t &firstEnpointNum) {
			firstEnpointNum = _interface_0_first_endpoint;
			return _total_endpoints_used;
		};

		static uint16_t getEndpointSize(const uint8_t &endpointNum, const bool otherSpeed) {
			uint16_t size = _hardware_type::getEndpointSizeFromHardware(endpointNum, otherSpeed);
			if (size == 0)
				size = _mixin_0_type::getEndpointSizeFromMixin(endpointNum, _hardware_type::getDeviceSpeed(), otherSpeed);
			if (size == 0)
				size = _mixin_1_type::getEndpointSizeFromMixin(endpointNum, _hardware_type::getDeviceSpeed(), otherSpeed);
			if (size == 0)
				size = _mixin_2_type::getEndpointSizeFromMixin(endpointNum, _hardware_type::getDeviceSpeed(), otherSpeed);
			return size;
		};


		// This could be abused...
		static _this_type *_singleton;
	}; // USBDevice


	template<class interface0type, class interface1type, class interface2type>
	ATTR_WEAK USBDevice<interface0type, interface1type, interface2type> *USBDevice<interface0type, interface1type, interface2type>::_singleton;

	// Declare the base (Null) USBMixin
	// We use template specialization (later) on a combination of *one* of the three interfaces,
	// along with the position to expose different content into the USBDevice.
	template < typename interface0type, typename interface1type, typename interface2type, int position >
	struct USBMixin {
		static const uint8_t endpoints_used = 0;
		typedef USBDevice<interface0type, interface1type, interface2type> usb_parent_type;
		USBMixin (usb_parent_type &usb_parent,
				  const uint8_t new_endpoint_offset,
                  const uint8_t first_interface_number
                  ) {};

		static bool isNull() { return true; };
		static const EndpointBufferSettings_t getEndpointConfigFromMixin(const uint8_t endpoint, const USBDeviceSpeed_t deviceSpeed, const bool other_speed) {
			return kEndpointBufferNull;
		};
		static bool handleNonstandardRequestInMixin(Setup_t &setup) { return false; };
		static bool sendSpecialDescriptorOrConfig(Setup_t &setup) { return false; };
		static uint16_t getEndpointSizeFromMixin(const uint8_t &endpointNum, const USBDeviceSpeed_t deviceSpeed, const bool otherSpeed) { return 8; };
	};

	template < typename interface0type, typename interface1type, typename interface2type >
	struct USBDefaultDescriptor : USBDescriptorDevice_t {
		USBDefaultDescriptor(const uint16_t vendorID, const uint16_t productID, const uint16_t productVersionBCD, const USBDeviceSpeed_t deviceSpeed) :
			USBDescriptorDevice_t(
								  /*    USBSpecificationBCD = */ USBFloatToBCD(2.0),
								  /*                  Class = */ kNoDeviceClass,
								  /*               SubClass = */ kNoDeviceSubclass,
								  /*               Protocol = */ kNoDeviceProtocol,

								  /*          Endpoint0Size = */ getEndpointSize(0, kEndpointTypeControl, deviceSpeed, false),

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

	template < typename interface0type, typename interface1type, typename interface2type >
	struct USBDefaultQualifier : USBDescriptorDeviceQualifier_t {
		USBDefaultQualifier() :
		USBDescriptorDeviceQualifier_t() // use the defaults
		{};
	};
} // namespace Motate

#endif
// MOTATEUSB_ONCE

