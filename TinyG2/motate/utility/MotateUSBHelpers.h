/*
 utility/MotateUSBHelpers.h - Library for the Motate system
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


#ifndef MOTATEUSBHELPERS_ONCE
#define MOTATEUSBHELPERS_ONCE

#include <inttypes.h>

namespace Motate {

#ifndef ATTR_PACKED
	#define ATTR_PACKED  __attribute__ ((packed))
#endif
#ifndef ATTR_WEAK
	#define ATTR_WEAK  __attribute__ ((weak))
#endif

	/* Enumerate the possible speed settings. */
	enum USBDeviceSpeed_t {
		kUSBDeviceLowSpeed  = 0,
		kUSBDeviceFullSpeed = 1,
		kUSBDeviceHighSpeed = 2
	};

	/* ############################################# */
	/* #                                           # */
	/* #            DESCRIPTORS (etc.)             # */
	/* #                                           # */
	/* #   Design Influenced by that of LUFA Lib   # */
	/* #        http://www.lufa-lib.org            # */
	/* #      Copyright 2013 (C) Dean Camera       # */
	/* #                                           # */
	/* # (The C99 design doesn't translate to C++) # */
	/* #                                           # */
	/* ############################################# */

	/* USB Configuration Descriptor Attribute Masks */
	enum USBConfigAttributes_t {
		/** Mask for the reserved bit in the Configuration Descriptor's \c ConfigAttributes field, which must be set on all
		 *  devices for historical purposes.
		 */
		kUSBConfigAttributeReserved     = 0x80,

		/** Can be masked with other configuration descriptor attributes for a \ref USB_Descriptor_Configuration_Header_t
		 *  descriptor's \c ConfigAttributes value to indicate that the specified configuration can draw its power
		 *  from the device's own power source.
		 */
		kUSBConfigAttributeSelfPowered  = 0x40,

		/** Can be masked with other configuration descriptor attributes for a \ref USB_Descriptor_Configuration_Header_t
		 *  descriptor's \c ConfigAttributes value to indicate that the specified configuration supports the
		 *  remote wakeup feature of the USB standard, allowing a suspended USB device to wake up the host upon
		 *  request.
		 */
		kUSBConfigAttributeRemoteWakeup = 0x20
	};

	/* Enum for the possible standard descriptor types, as given in each descriptor's header. */
	enum USBDescriptorTypes_t
	{
		kDeviceDescriptor               = 0x01, /* device descriptor. */
		kConfigurationDescriptor        = 0x02, /* configuration descriptor. */
		kStringDescriptor               = 0x03, /* string descriptor. */
		kInterfaceDescriptor            = 0x04, /* interface descriptor. */
		kEndpointDescriptor             = 0x05, /* endpoint descriptor. */
		kDeviceQualifierDescriptor      = 0x06, /* device qualifier descriptor. */
		kOtherDescriptor                = 0x07, /* other type. */
		kInterfacePowerDescriptor       = 0x08, /* interface power descriptor. */
		kInterfaceAssociationDescriptor = 0x0B, /* interface association descriptor. */
		kCSInterfaceDescriptor          = 0x24, /* class specific interface descriptor. */
		kCSEndpointDescriptor           = 0x25, /* class specific endpoint descriptor. */
	};


	/* Enum for possible Class, Subclass and Protocol values of device and interface descriptors. */
	enum USBDescriptorClassSubclassProtocol_t
	{
		kNoDeviceClass          = 0x00, /*   Descriptor Class value indicating that the device does not belong
										 *   to a particular class at the device level.
										 */
		kNoDeviceSubclass       = 0x00, /*   Descriptor Subclass value indicating that the device does not belong
										 *   to a particular subclass at the device level.
										 */
		kNoDeviceProtocol       = 0x00, /*   Descriptor Protocol value indicating that the device does not belong
										 *   to a particular protocol at the device level.
										 */
		kVendorSpecificClass    = 0xFF, /*   Descriptor Class value indicating that the device/interface belongs
										 *   to a vendor specific class.
										 */
		kVendorSpecificSubclass = 0xFF, /*   Descriptor Subclass value indicating that the device/interface belongs
										 *   to a vendor specific subclass.
										 */
		kVendorSpecificProtocol = 0xFF, /*   Descriptor Protocol value indicating that the device/interface belongs
										 *   to a vendor specific protocol.
										 */
		kIADDeviceClass         = 0xEF, /*   Descriptor Class value indicating that the device belongs to the
										 *   Interface Association Descriptor class.
										 */
		kIADDeviceSubclass      = 0x02, /*   Descriptor Subclass value indicating that the device belongs to the
										 *   Interface Association Descriptor subclass.
										 */
		kIADDeviceProtocol      = 0x01, /*   Descriptor Protocol value indicating that the device belongs to the
										 *   Interface Association Descriptor protocol.
										 */
	};

	/** Enum for the device string descriptor IDs within the device. Each string descriptor should
	 *  have a unique ID index associated with it, which can be used to refer to the string from
	 *  other descriptors.
	 */
	enum USBStringDescriptors_t
	{
		kLanguageStringId     = 0, /* Supported Languages string descriptor ID (must be zero) */
		kNoDescriptorId       = 0, /* Indicates a lack of a string to give */
		kManufacturerStringId = 1, /* Manufacturer string ID */
		kProductStringId      = 2, /* Product string ID */
		kSerialNumberId       = 3, /* Serial Number ID */
	};

	/* Endpoint Descriptor Attribute Masks */
	enum USBEndpointAttributes_k {
		/*  Attributes value to indicate that the specified endpoint is not synchronized. */
		kEndpointAttrNoSync   = (0 << 2),

		/*  Attributes value to indicate that the specified endpoint is asynchronous. */
		kEndpointAttrAsync    = (1 << 2),

		/* Attributes value to indicate that the specified endpoint is adaptive. */
		kEndpointAttrAdaptive = (2 << 2),

		/* Attributes value to indicate that the specified endpoint is synchronized. */
		kEndpointAttrSync     = (3 << 2)
	};


	/* Endpoint Descriptor Usage Masks */
	enum USBEndpointUsage_k {
		/* Attributes value to indicate that the specified endpoint is used for data transfers. */
		kEndpointUsageData             = (0 << 4),

		/* Attributes value to indicate that the specified endpoint is used for feedback. */
		kEndpointUsageFeedback         = (1 << 4),

		/* Attributes value to indicate that the specified endpoint is used for implicit feedback. */
		kEndpointUsageImplicitFeedback = (2 << 4)
	};

	enum USBEndpointType_t {
		/** Mask for a CONTROL type endpoint or pipe. */
		kEndpointTypeControl     = 0x00,

		/** Mask for an ISOCHRONOUS type endpoint or pipe. */
		kEndpointTypeIsochronous = 0x01,

		/** Mask for a BULK type endpoint or pipe. */
		kEndpointTypeBulk        = 0x02,

		/** Mask for an INTERRUPT type endpoint or pipe. */
		kEndpointTypeInterrupt   = 0x03,

		/** This is NOT and endpoint type, but just the bits needed to mask this from the attributes. */
		kEndpointTypeMask        = 0x03
	};

	// *NOTE* These struct/classes are carefully constructed to store in the correct binary format
	//        for streaming directly to the host.

	// Sometimes, the inlining and optimized fail you, and you end up with a define.
	// Alas, it's not the end of the world.
#define USBFloatToBCD(in) ( \
((uint16_t)(in / 10)) << 12 | (((uint16_t)in % 10)) << 8 | (((uint16_t)(in*10) % 10)) << 4 | (((uint16_t)((in+0.001)*100) % 10))\
)

// /* Here's the C++ version of the above define, just in case we can make it work later: */
//	inline static uint16_t USBFloatToBCD(const float in) {
//		return ((uint16_t)(in / 10)) << 12 | (((uint16_t)in % 10)) << 8 | (((uint16_t)(in*10) % 10)) << 4 | (((uint16_t)((in+0.001)*100) % 10));
//	}

	enum USBPowerOptions_t {
		kUSBSelfPowered  = 0xC0,
		kUSBRemoteWakeup = 0x20 // <- This doesn't belong here ... ?
	};

	
	// Note: I'm adding pragma marks for XCode, since it has trouble parsing the oddly formed constructors.
#pragma mark USBDescriptorHeader_t
	// Header, used in all of the descriptors
	struct USBDescriptorHeader_t
	{
		uint8_t Size; /* Size of the descriptor, in bytes. */
		uint8_t Type; /* Type of the descriptor, either a value in USBDescriptorTypes or a value
					   * given by the specific class.
					   */

		/* Initialization */

		USBDescriptorHeader_t(uint8_t _size, uint8_t _type) : Size(_size), Type(_type) {};
	} ATTR_PACKED;


	/* ############################################# */
	/* #                                           # */
	/* #             DEVICE DESCRIPTOR             # */
	/* #                                           # */
	/* ############################################# */

#pragma mark USBDescriptorDevice_t

	//	Device
	struct USBDescriptorDevice_t
	{
		USBDescriptorHeader_t Header; /* Descriptor header, including type and size. */

		uint16_t USBSpecificationBCD; /* BCD of the supported USB specification. */
		uint8_t  Class;            /* USB device class. */
		uint8_t  SubClass;         /* USB device subclass. */
		uint8_t  Protocol;         /* USB device protocol. */

		uint8_t  Endpoint0Size;    /* Size of the control (address 0) endpoint's bank in bytes. */

		uint16_t VendorID;         /* Vendor ID for the USB product. */
		uint16_t ProductID;        /* Unique product ID for the USB product. */
		uint16_t ReleaseNumber;    /* Product release (version) number. */
		uint8_t  ManufacturerStrIndex; /*   String index for the manufacturer's name. The
										*   host will request this string via a separate
										*   control request for the string descriptor.
										*/
		uint8_t  ProductStrIndex;   /* String index for the product name/details. */
		uint8_t  SerialNumStrIndex; /*   String index for the product's globally unique hexadecimal
									 *   serial number, in uppercase Unicode ASCII.
									 */
		uint8_t  NumberOfConfigurations; /*   Total number of configurations supported by
										  *   the device.
										  */
		/* Initialization */

		USBDescriptorDevice_t(
						 uint16_t _USBSpecificationBCD,
						 uint8_t  _Class,
						 uint8_t  _SubClass,

						 uint8_t  _Protocol,

						 uint8_t  _Endpoint0Size,

						 uint16_t _VendorID,
						 uint16_t _ProductID,
						 float _ReleaseNumber,

						 uint8_t  _ManufacturerStrIndex,
						 uint8_t  _ProductStrIndex,
						 uint8_t  _SerialNumStrIndex,

						 uint8_t  _NumberOfConfigurations
						 )
		: Header(sizeof(USBDescriptorDevice_t), kDeviceDescriptor),
		USBSpecificationBCD(_USBSpecificationBCD),
		Class(_Class),
		SubClass(_SubClass),

		Protocol(_Protocol),

		Endpoint0Size(_Endpoint0Size),

		VendorID(_VendorID),
		ProductID(_ProductID),
		ReleaseNumber(_ReleaseNumber),

		ManufacturerStrIndex(_ManufacturerStrIndex),
		ProductStrIndex(_ProductStrIndex),
		SerialNumStrIndex(_SerialNumStrIndex),

		NumberOfConfigurations(_NumberOfConfigurations)
		{};
	} ATTR_PACKED;


#pragma mark USBDescriptorDeviceQualifier_t
	struct USBDescriptorDeviceQualifier_t
	{
		USBDescriptorHeader_t Header; /* Descriptor header, including type and size. */

		uint16_t USBSpecification; /* BCD of the supported USB specification. */
		uint8_t  Class; /* USB device class. */
		uint8_t  SubClass; /* USB device subclass. */
		uint8_t  Protocol; /* USB device protocol. */

		uint8_t  Endpoint0Size; /* Size of the control (address 0) endpoint's bank in bytes. */
		uint8_t  NumberOfConfigurations; /* Total number of configurations supported by
										  *   the device.
										  */
		uint8_t  Reserved; /* Reserved for future use, must be 0. */

		/* Initialization */

		USBDescriptorDeviceQualifier_t(
									 uint16_t _USBSpecification = 0x0200,
									 uint8_t  _Class = 0,
									 uint8_t  _SubClass = 0,
									 uint8_t  _Protocol = 0,

									 uint8_t  _Endpoint0Size = 64, /* Only high-speed (or faster) devices
																	*   are allowed to respond to Device Qualifier requests.
																	* High-speed devices must have a control endpoint size of 64.
																	*/
									 uint8_t  _NumberOfConfigurations = 1
									)
		: Header(sizeof(USBDescriptorDeviceQualifier_t), kDeviceQualifierDescriptor),
		USBSpecification(_USBSpecification),
		Class(_Class),
		SubClass(_SubClass),
		Protocol(_Protocol),

		Endpoint0Size(_Endpoint0Size),
		NumberOfConfigurations(_NumberOfConfigurations),

		Reserved(0)
		{};
	} ATTR_PACKED;

	/* ############################################# */
	/* #                                           # */
	/* #           DEVICE CONFIGURATION            # */
	/* #                                           # */
	/* ############################################# */
#pragma mark USBDescriptorConfigurationHeader_t

	struct USBDescriptorConfigurationHeader_t
	{
		USBDescriptorHeader_t Header; /* Descriptor header, including type and size. */

		uint16_t TotalConfigurationSize; /*   Size of the configuration descriptor header,
										  *   and all sub descriptors inside the configuration.
										  */
		uint8_t  TotalInterfaces; /* Total number of interfaces in the configuration. */

		uint8_t  ConfigurationNumber; /* Configuration index of the current configuration. */
		uint8_t  ConfigurationStrIndex; /* Index of a string descriptor describing the configuration. */

		uint8_t  ConfigAttributes; /*   Configuration attributes, comprised of a mask of \c USB_CONFIG_ATTR_* masks. */

		uint8_t  MaxPowerConsumption; /*   Maximum power consumption of the device while in the
									   *   current configuration, calculated by the \ref USB_CONFIG_POWER_MA()
									   *   macro.
									   */

		/* Initialization */

		USBDescriptorConfigurationHeader_t(
										 uint16_t _TotalConfigurationSize,
										 uint8_t  _TotalInterfaces,

										 uint8_t  _ConfigurationNumber,
										 uint8_t  _ConfigurationStrIndex,

										 uint8_t  _ConfigAttributes,

										 uint16_t  _MaxPowerConsumption
										)
		: Header(sizeof(USBDescriptorConfigurationHeader_t), kConfigurationDescriptor),
        TotalConfigurationSize(_TotalConfigurationSize),
        TotalInterfaces(_TotalInterfaces),

        ConfigurationNumber(_ConfigurationNumber),
        ConfigurationStrIndex(_ConfigurationStrIndex),

        ConfigAttributes(_ConfigAttributes | 0x80), /* Set the 0x80 Reserved flag, as necessary for historical reasons. */

        MaxPowerConsumption((uint8_t)(_MaxPowerConsumption >> 1)) /* MaxPowerConsumption is expressed in mA/2 */
		{};
	} ATTR_PACKED;

#pragma mark USBDescriptorConfiguration_t
	// Note that USBDescriptorConfiguration_t* is also designed to be cast to uint8_t*,
	// so all of the non-static variables for this and all inherited types must be in the order and size
	// defined by the USB spec for configuration and header interfaces.

	// USBNullInterface is the default interface placeholder
	struct USBNullInterface {};

	// Forward declare the USBMixin template.
	// Mixins are described more below.
	template < typename interface0type, typename interface1type, typename interface2type, int position > struct USBMixin;

	// This is a templated version of the default descriptor.
	// Specializations of this can change the default parameters based on mixin proxies.
	template < typename interface0type, typename interface1type, typename interface2type > struct USBDefaultDescriptor;
	template < typename interface0type, typename interface1type, typename interface2type > struct USBDefaultQualifier;

	// Forward declare the USBMixin template.
	// Mixins are described more below.
	template < typename interface0type, typename interface1type, typename interface2type, int position > struct USBConfigMixin;

	template<class interface0type, class interface1type = USBNullInterface, class interface2type = USBNullInterface>
	struct USBDescriptorConfiguration_t :
		USBDescriptorConfigurationHeader_t,
		USBConfigMixin<interface0type, interface1type, interface2type, 0>,
		USBConfigMixin<interface0type, interface1type, interface2type, 1>,
		USBConfigMixin<interface0type, interface1type, interface2type, 2>
	{
		// Shortcut typedefs
		typedef USBDescriptorConfiguration_t<interface0type, interface1type, interface2type> _this_type;

		typedef USBConfigMixin<interface0type, interface1type, interface2type, 0> _config_mixin_0_type;
		typedef USBConfigMixin<interface0type, interface1type, interface2type, 1> _config_mixin_1_type;
		typedef USBConfigMixin<interface0type, interface1type, interface2type, 2> _config_mixin_2_type;

		static const uint8_t _interface_0_number    = 0;
		static const uint8_t _interface_1_number    = _interface_0_number + _config_mixin_0_type::interfaces;
		static const uint8_t _interface_2_number    = _interface_1_number + _config_mixin_1_type::interfaces;
		static const uint8_t _total_interfaces_used = _interface_2_number + _config_mixin_2_type::interfaces;

		typedef USBMixin<interface0type, interface1type, interface2type, 0> _mixin_0_type;
		typedef USBMixin<interface0type, interface1type, interface2type, 1> _mixin_1_type;
		typedef USBMixin<interface0type, interface1type, interface2type, 2> _mixin_2_type;
		typedef USBDefaultDescriptor<interface0type, interface1type, interface2type> _descriptor_type;

		// Keep track of the endpoint usage
		// Endpoint zero is the control interface, and is owned by nobody.
		static const uint8_t _interface_0_first_endpoint = 1;
		static const uint8_t _interface_1_first_endpoint = _interface_0_first_endpoint + _mixin_0_type::endpoints_used;
		static const uint8_t _interface_2_first_endpoint = _interface_1_first_endpoint + _mixin_1_type::endpoints_used;
		static const uint8_t _total_endpoints_used       = _interface_2_first_endpoint + _mixin_2_type::endpoints_used;

		USBDescriptorConfiguration_t(
									 uint8_t _ConfigAttributes,
									 uint16_t _MaxPowerConsumption,
                                     const USBDeviceSpeed_t _deviceSpeed,
									 bool _otherConfig
									 ) :
			USBDescriptorConfigurationHeader_t(
											   /* _TotalConfigurationSize = */ sizeof(_this_type),
											   /*        _TotalInterfaces = */ _total_interfaces_used,

											   /*    _ConfigurationNumber = */ _otherConfig ? 2 : 1,
											   /*  _ConfigurationStrIndex = */ 0, /* Fixme? */

											   /*       _ConfigAttributes = */ _ConfigAttributes,

											   /*    _MaxPowerConsumption = */ _MaxPowerConsumption
											   ),
			_config_mixin_0_type(_interface_0_first_endpoint, _interface_0_number, _deviceSpeed, _otherConfig),
			_config_mixin_1_type(_interface_1_first_endpoint, _interface_1_number, _deviceSpeed, _otherConfig),
			_config_mixin_2_type(_interface_2_first_endpoint, _interface_2_number, _deviceSpeed, _otherConfig)
		{};
	} ATTR_PACKED;

	// Declare the base (Null) USBConfigMixin
	// We use template specialization (later) on a combination of *one* of the three interfaces,
	// along with the position to expose different content into the USBDevice.
	// This is the same emchanism as USBMixin<>.
	template < typename usbIFA, typename usbIFB, typename usbIFC, int position >
	struct USBConfigMixin {
		static const uint8_t interfaces = 0;
		USBConfigMixin (const uint8_t _first_endpoint_number, const uint8_t _first_interface_number, const USBDeviceSpeed_t _device_speed, const bool _other_config) {};
		static bool isNull() { return true; };
	};

	/* ############################################# */
	/* #                                           # */
	/* #             DEVICE INTERFACE              # */
	/* #                                           # */
	/* ############################################# */
#pragma mark USBDescriptorInterface_t

	struct USBDescriptorInterface_t
	{
		USBDescriptorHeader_t Header; /* Descriptor header, including type and size. */

		uint8_t InterfaceNumber; /*  Index of the interface in the current configuration. */
		uint8_t AlternateSetting; /*  Alternate setting for the interface number. The same
								   *   interface number can have multiple alternate settings
								   *   with different endpoint configurations, which can be
								   *   selected by the host.
								   */
		uint8_t TotalEndpoints; /* Total number of endpoints in the interface. */

		uint8_t Class; /* Interface class ID. */
		uint8_t SubClass; /* Interface subclass ID. */
		uint8_t Protocol; /* Interface protocol ID. */

		uint8_t InterfaceStrIndex; /* Index of the string descriptor describing the interface. */

		/* Initialization */

		USBDescriptorInterface_t(
								  uint8_t _InterfaceNumber,
								  uint8_t _AlternateSetting,

								  uint8_t _TotalEndpoints,

								  uint8_t _Class,
								  uint8_t _SubClass,
								  uint8_t _Protocol,

								  uint8_t _InterfaceStrIndex
								)
		: Header(sizeof(USBDescriptorInterface_t), kInterfaceDescriptor),
		InterfaceNumber(_InterfaceNumber),
		AlternateSetting(_AlternateSetting),
		TotalEndpoints(_TotalEndpoints),
		Class(_Class),
		SubClass(_SubClass),
		Protocol(_Protocol),

		InterfaceStrIndex(_InterfaceStrIndex)
		{};
	} ATTR_PACKED ;


#pragma mark USBDescriptorInterfaceAssociation_t
	struct USBDescriptorInterfaceAssociation_t
	{
		USBDescriptorHeader_t Header; /* Descriptor header, including type and size. */

		uint8_t FirstInterfaceIndex; /* Index of the first associated interface. */
		uint8_t TotalInterfaces; /* Total number of associated interfaces. */

		uint8_t Class; /* Interface class ID. */
		uint8_t SubClass; /* Interface subclass ID. */
		uint8_t Protocol; /* Interface protocol ID. */

		uint8_t IADStrIndex; /* Index of the string descriptor describing the
							  *   interface association.
							  */

		/* Initialization */

		USBDescriptorInterfaceAssociation_t(
										  uint8_t _FirstInterfaceIndex,
										  uint8_t _TotalInterfaces,

										  uint8_t _Class,
										  uint8_t _SubClass,
										  uint8_t _Protocol,

										  uint8_t _IADStrIndex
										  )
		: Header(sizeof(USBDescriptorInterfaceAssociation_t), kInterfaceAssociationDescriptor),
		FirstInterfaceIndex(_FirstInterfaceIndex),
		TotalInterfaces(_TotalInterfaces),
		
		Class(_Class),
		SubClass(_SubClass),
		Protocol(_Protocol),
		
		IADStrIndex(_IADStrIndex)
		{};
	} ATTR_PACKED;


	/***
	 Endpoint maximimum packet sizes, by type, according to the specs:
	 Control Endpoints:
	 low speed devices : 8 bytes
	 full speed devices: 8, 16, 32 or 64 bytes
	 high speed devices: 64 bytes

	 Interrupt Endpoints:
	 low speed devices : 8 bytes
	 full speed devices: up to 64 bytes
	 high speed devices: up to 1024 bytes

	 Isochronous Endpoints:
	 low speed devices : not allowed
	 full speed devices: up to 1023 bytes
	 high speed devices: up to 1024 bytes

	 Bulk Endpoints:
	 low speed devices : not allowed
	 full speed devices: 8, 16, 32 or 64 bytes
	 high speed devices: up to 512 bytes

	 Useful resources:
	 http://www.beyondlogic.org/usbnutshell/usb4.shtml
	 http://wiki.osdev.org/Universal_Serial_Bus

	 ***/

	extern uint16_t checkEndpointSizeHardwareLimits(const uint16_t tempSize, const uint8_t endpointNumber, const USBEndpointType_t endpointType, const bool otherSpeed);

	// Here we use the above rules for endpoints, and then determine, based on the endpoint number, type, and which if it's the main speed or "other speed."
	static /*inline*/ uint16_t getEndpointSize(const uint8_t endpointNumber, const USBEndpointType_t endpointType, const USBDeviceSpeed_t USBDeviceSpeed, const bool otherSpeed, bool limitedSize = false) {
		uint16_t tempSize = 0;
        uint16_t suggestedSize = 512;
        if (otherSpeed) {
            suggestedSize = 64;
        } else if (limitedSize) {
            suggestedSize = 256;
        }

		if (USBDeviceSpeed == kUSBDeviceHighSpeed) {
			// Note that other_speed only applies to high-speed devices
			if (endpointType == kEndpointTypeIsochronous) {
				tempSize = suggestedSize;
			}
			if (endpointType == kEndpointTypeInterrupt) {
				tempSize = suggestedSize;
			}
			else if (endpointType == kEndpointTypeBulk) {
				tempSize = suggestedSize;
			} else {
				tempSize = 64; // maximum size for all other full-speed endpoints is 64
			}
		}

		if (USBDeviceSpeed == kUSBDeviceFullSpeed) {
			if (endpointType == kEndpointTypeIsochronous) {
				tempSize = suggestedSize; // WTF?!?
			} else {
				tempSize = 64; // maximum size for all other full-speed endpoints is 64
			}
		}

		// low speed devices have more restrictions, let's get that out of the way...
		if (USBDeviceSpeed == kUSBDeviceLowSpeed) {
			if (endpointType == kEndpointTypeControl || endpointType == kEndpointTypeInterrupt) {
				tempSize = 8;
			}
		}

		tempSize = checkEndpointSizeHardwareLimits(tempSize, endpointNumber, endpointType, otherSpeed);

		return tempSize;
	};

#pragma mark USBDescriptorEndpoint_t
	struct USBDescriptorEndpoint_t
	{
		USBDescriptorHeader_t Header; /* Descriptor header, including type and size. */

		uint8_t  EndpointAddress; /* Logical address of the endpoint within the device for the current
		                           *   configuration, including direction mask.
		                           */
		uint8_t  Attributes; /* Endpoint attributes, comprised of a mask of the endpoint type (EP_TYPE_*)
		                      *   and attributes (ENDPOINT_ATTR_*) masks.
		                      */
		uint16_t EndpointSize; /* Size of the endpoint bank, in bytes. This indicates the maximum packet
		                        *   size that the endpoint can receive at a time.
		                        */
		uint8_t  PollingIntervalMS; /* Polling interval in milliseconds for the endpoint if it is an INTERRUPT
		                             *   or ISOCHRONOUS type.
		                             */

		/* Initialization */


		USBDescriptorEndpoint_t (
								 USBDeviceSpeed_t _deviceSpeed,
                                 bool             _otherSpeed,
								 bool             _input,
								 uint8_t          _EndpointAddress,

								 uint8_t          _Attributes,

								 uint8_t          _PollingIntervalMS,

								 bool             _limited_size = false
								 )
		: Header(sizeof(USBDescriptorEndpoint_t), kEndpointDescriptor),
		EndpointAddress(_EndpointAddress | (_input ? 0x80 : 0x00)),

		Attributes(_Attributes),

		EndpointSize(getEndpointSize(_EndpointAddress, (USBEndpointType_t)(_Attributes & kEndpointTypeMask), _deviceSpeed, _otherSpeed, _limited_size)),

		PollingIntervalMS(_PollingIntervalMS)
		{};
	} ATTR_PACKED;


	// Flags for endpoint information, for use by the hardware to configure the endpoints internally.

	// These are to be defined in the the hardware-specific USB files, but *all* values must be defined.
	// For example: On a platform that can only have 64 byte buffers, everything above kEnpointBufferSizeUpTo64
	//  is equal to kEnpointBufferSizeUpTo64.

	// There is also a typedef: EndpointBufferSettings_t

	/*
	 kEndpointBufferNull

	 // endpoint direction
	 kEndpointBufferInput
	 kEndpointBufferOutput

	 // buffer sizes
	 kEnpointBufferSizeUpTo8
	 kEnpointBufferSizeUpTo16
	 kEnpointBufferSizeUpTo32
	 kEnpointBufferSizeUpTo64
	 kEnpointBufferSizeUpTo128
	 kEnpointBufferSizeUpTo256
	 kEnpointBufferSizeUpTo512
	 kEnpointBufferSizeUpTo1024

	 // buffer "blocks" -- 2 == "ping pong"
	 // Note that there must be one, or this is a null endpoint.
	 kEndpointBufferBlocks1
	 kEndpointBufferBlocksUpTo2
	 kEndpointBufferBlocksUpTo3

	 // endpoint types (mildly redundant from the config)
	 kEndpointBufferTypeControl
	 kEndpointBufferTypeIsochronous
	 kEndpointBufferTypeBulk
	 kEndpointBufferTypeInterrupt
	 */

#pragma mark template <uint8_t size> USBDescriptorString_t
	struct USBDescriptorStringHeader_t
	{
		USBDescriptorHeader_t Header; /**< Descriptor header, including type and size. */

		/* Initialization */

		USBDescriptorStringHeader_t(
							  const uint8_t size
							  )
		: Header(sizeof(USBDescriptorHeader_t) + size, kStringDescriptor)
		{
        };
	} ATTR_PACKED;

#pragma mark Setup_t
	struct Setup_t
	{
		uint8_t  _bmRequestType;
		uint8_t  _bRequest;
		uint8_t  _wValueL;
		uint8_t  _wValueH;
		uint16_t _wIndex;
		uint16_t _wLength;

		// internal enums

		// Possible values of _bRequest
		enum _requests_t {
			kGetStatus           = 0,
			kClearFeature        = 1,
			kSetFeature          = 3,

			kSetAddress          = 5,

			kGetDescriptor       = 6,
			kSetDescriptor       = 7,

			kGetConfiguration    = 8,
			kSetConfiguration    = 9,

			kGetInterface        = 10,
			kSetInterface        = 11
		};

		// Possible values of _bmRequestType
		enum _requestTypes_t {
			kRequestHostToDevice  = 0x00,
			kRequestDeviceToHost  = 0x80,
			kRequestDirectionMask = 0x80,

			kRequestStandard      = 0x00,
			kRequestClass         = 0x20,
			kRequestVendor        = 0x40,
			kRequestTypeMask      = 0x60,

			kRequestDevice        = 0x00,
			kRequestInterface     = 0x01,
			kRequestEndpoint      = 0x02,
			kRequestOther         = 0x03,
			kRequestRecipientMask = 0x1F
		};

		// Possible values of wValueL
		enum _setupValues_t {
			kSetupEndpointHalt       = 0x00,
			kSetupDeviceRemoteWakeup = 0x01,
			kSetupTestMode           = 0x02,

			// OTG-only options:
			kSetupBHNPEnable         = 0x03,
			kSetupAHNPSupport        = 0x04,
			kSetupAALTHNPSupport     = 0x05,
		};

		const bool isADeviceToHostRequest() const {
			return (_bmRequestType & kRequestDeviceToHost);
		};

		const bool isAStandardRequestType() const {
			return ((_bmRequestType & kRequestTypeMask) == kRequestStandard);
		};

		const bool isADeviceRequest() const {
			return ((_bmRequestType & kRequestRecipientMask) == kRequestDevice);
		}

		// Warning! Proper english making the "isA" into an "isAn"
		const bool isAnInterfaceRequest() const {
			return ((_bmRequestType & kRequestRecipientMask) == kRequestInterface);
		}

		// Warning! Proper english making the "isA" into an "isAn"
		const bool isAnEndpointRequest() const {
			return ((_bmRequestType & kRequestRecipientMask) == kRequestEndpoint);
		}

		const bool isAGetStatusRequest() const {
			return (_bRequest == kGetStatus);
		}

		const bool isAClearFeatureRequest() const {
			return (_bRequest == kClearFeature);
		}

		const bool isASetFeatureRequest() const {
			return (_bRequest == kSetFeature);
		}

		const bool isAGetDescriptorRequest() const {
			return (_bRequest == kGetDescriptor);
		}

		const bool isASetDescriptorRequest() const {
			return (_bRequest == kSetDescriptor);
		}

		const bool isAGetConfigurationRequest() const {
			return (_bRequest == kGetConfiguration);
		}

		const bool isASetConfigurationRequest() const {
			return (_bRequest == kSetConfiguration);
		}

		const bool isAGetInterfaceRequest() const {
			return (_bRequest == kGetInterface);
		}

		const bool isASetInterfaceRequest() const {
			return (_bRequest == kSetInterface);
		}

		const bool isASetAddressRequest() const {
			return (_bRequest == kSetAddress);
		}

		const bool isADeviceToHostClassInterfaceRequest() const {
			return (_bmRequestType == (kRequestDeviceToHost | kRequestClass | kRequestInterface));
		};
		
		const bool isAHostToDeviceClassInterfaceRequest() const {
			return (_bmRequestType == (kRequestHostToDevice | kRequestClass | kRequestInterface));
		};

		const bool requestIs(uint8_t testRequest) const {
			return _bRequest == testRequest;
		};

		const _setupValues_t featureToSetOrClear() {
			return (_setupValues_t) _wValueL;
		};
		
		const uint8_t valueLow() {
			return _wValueL;
		}

		const uint8_t valueHigh() {
			return _wValueH;
		}

		const uint16_t index() {
			return _wIndex;
		}

		const uint16_t length() {
			return _wLength;
		}
	} ATTR_PACKED; // Setup_t

}

#endif
// MOTATEUSBHELPERS_ONCE
