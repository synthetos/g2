/*
 utility/MotateUSBCDC.h - Library for the Motate system
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

#ifndef MOTATEUSBCDC_ONCE
#define MOTATEUSBCDC_ONCE

#include "utility/MotateUSBHelpers.h"
#include "Reset.h"

namespace Motate {

	/* ############################################ */
	/* #                                          # */
	/* #             USB CDC Interface            # */
	/* #                                          # */
	/* ############################################ */

	enum CDCDescriptorClassSubclassProtocol_t
	{
		kCDCClass               = 0x02, /*   Descriptor Class value indicating that the device or interface
										 *   belongs to the CDC class.
										 */
		kNoSpecificSubclass     = 0x00, /*   Descriptor Subclass value indicating that the device or interface
										 *   belongs to no specific subclass of the CDC class.
										 */
		kACMSubclass            = 0x02, /*   Descriptor Subclass value indicating that the device or interface
										 *   belongs to the Abstract Control Model CDC subclass.
										 */
		kATCommandProtocol      = 0x01, /*   Descriptor Protocol value indicating that the device or interface
										 *   belongs to the AT Command protocol of the CDC class.
										 */
		kNoSpecificProtocol     = 0x00, /*   Descriptor Protocol value indicating that the device or interface
										 *   belongs to no specific protocol of the CDC class.
										 */
#if 0
		// Duplicate from the higher level enums
		kVendorSpecificProtocol = 0xFF, /*   Descriptor Protocol value indicating that the device or interface
										 *   belongs to a vendor-specific protocol of the CDC class.
										 */
#endif
		kCDCDataClass           = 0x0A, /*   Descriptor Class value indicating that the device or interface
										 *   belongs to the CDC Data class.
										 */
		kNoDataSubclass         = 0x00, /*   Descriptor Subclass value indicating that the device or interface
										 *   belongs to no specific subclass of the CDC data class.
										 */
		kNoDataProtocol         = 0x00, /*   Descriptor Protocol value indicating that the device or interface
										 *   belongs to no specific protocol of the CDC data class.
										 */
	};

	/** Enum for the CDC class specific control requests that can be issued by the USB bus host. */
	enum CDCClassRequests_t
	{
		kSendEncapsulatedCommand = 0x00, /* CDC class-specific request to send an encapsulated command to the device. */
		kGetEncapsulatedResponse = 0x01, /* CDC class-specific request to retrieve an encapsulated command response from the device. */
		kSetLineEncoding         = 0x20, /* CDC class-specific request to set the current virtual serial port configuration settings. */
		kGetLineEncoding         = 0x21, /* CDC class-specific request to get the current virtual serial port configuration settings. */
		kSetControlLineState     = 0x22, /* CDC class-specific request to set the current virtual serial port handshake line states. */
		kSendBreak               = 0x23, /* CDC class-specific request to send a break to the receiver via the carrier channel. */
	};

	/** Enum for the CDC class specific notification requests that can be issued by a CDC device to a host. */
	enum CDCClassNotifications_t
	{
		kSerialState = 0x20, /*   Notification type constant for a change in the virtual serial port
							  *   handshake line states, for use with a \ref USB_Request_Header_t
							  *   notification structure when sent to the host via the CDC notification
							  *   endpoint.
							  */
	};

	/** Enum for the CDC class specific interface descriptor subtypes. */
	enum CDCDescriptorSubtypes_t
	{
		kCDCCSInterfaceHeader           = 0x00, /* CDC class-specific Header functional descriptor. */
		kCDCCSInterfaceCallManagement   = 0x01, /* CDC class-specific Call Management functional descriptor. */
		kCDCCSInterfaceACM              = 0x02, /* CDC class-specific Abstract Control Model functional descriptor. */
		kCDCCSInterfaceDirectLine       = 0x03, /* CDC class-specific Direct Line functional descriptor. */
		kCDCCSInterfaceTelephoneRinger  = 0x04, /* CDC class-specific Telephone Ringer functional descriptor. */
		kCDCCSInterfaceTelephoneCall    = 0x05, /* CDC class-specific Telephone Call functional descriptor. */
		kCDCCSInterfaceUnion            = 0x06, /* CDC class-specific Union functional descriptor. */
		kCDCCSInterfaceCountrySelection = 0x07, /* CDC class-specific Country Selection functional descriptor. */
		kCDCCSInterfaceTelephoneOpModes = 0x08, /* CDC class-specific Telephone Operation Modes functional descriptor. */
		kCDCCSInterfaceUSBTerminal      = 0x09, /* CDC class-specific USB Terminal functional descriptor. */
		kCDCCSInterfaceNetworkChannel   = 0x0A, /* CDC class-specific Network Channel functional descriptor. */
		kCDCCSInterfaceProtocolUnit     = 0x0B, /* CDC class-specific Protocol Unit functional descriptor. */
		kCDCCSInterfaceExtensionUnit    = 0x0C, /* CDC class-specific Extension Unit functional descriptor. */
		kCDCCSInterfaceMultiChannel     = 0x0D, /* CDC class-specific Multi-Channel Management functional descriptor. */
		kCDCCSInterfaceCAPI             = 0x0E, /* CDC class-specific Common ISDN API functional descriptor. */
		kCDCCSInterfaceEthernet         = 0x0F, /* CDC class-specific Ethernet functional descriptor. */
		kCDCCSInterfaceATM              = 0x10, /* CDC class-specific Asynchronous Transfer Mode functional descriptor. */
	};

	/** Enum for the possible line encoding formats of a virtual serial port. */
	enum CDCLineEncodingFormats_t
	{
		kCDCLineEncodingOneStopBit          = 0, /* Each frame contains one stop bit. */
		kCDCLineEncodingOneAndAHalfStopBits = 1, /* Each frame contains one and a half stop bits. */
		kCDCLineEncodingTwoStopBits         = 2, /* Each frame contains two stop bits. */
	};

	/** Enum for the possible line encoding parity settings of a virtual serial port. */
	enum CDCLineEncodingParity_t
	{
		kCDCParityNone  = 0, /* No parity bit mode on each frame. */
		kCDCParityOdd   = 1, /* Odd parity bit mode on each frame. */
		kCDCParityEven  = 2, /* Even parity bit mode on each frame. */
		kCDCParityMark  = 3, /* Mark parity bit mode on each frame. */
		kCDCParitySpace = 4, /* Space parity bit mode on each frame. */
	};

#pragma mark USBCDCDescriptorFunctionalHeader_t
	/** CDC class-specific Functional Header Descriptor (LUFA naming conventions).
	 *
	 *  Type define for a CDC class-specific functional header descriptor. This indicates to the host that the device
	 *  contains one or more CDC functional data descriptors, which give the CDC interface's capabilities and configuration.
	 *  See the CDC class specification for more details.
	 *
	 */

	struct USBCDCDescriptorFunctionalHeader_t
	{
		USBDescriptorHeader_t   Header; /*   Regular descriptor header containing the descriptor's type and length. */
		uint8_t                 Subtype; /*   Sub type value used to distinguish between CDC class-specific descriptors,
								          *   must be \ref CDC_DSUBTYPE_CSInterface_Header.
								          */
		uint16_t                CDCSpecificationBCD; /*   Version number of the CDC specification implemented by the device,
								                      *   encoded in BCD format.
								                      *
								                      *   \see \ref VERSION_BCD() utility macro.
								                      */
		USBCDCDescriptorFunctionalHeader_t(
										   uint16_t  _CDCSpecificationBCD = USBFloatToBCD(1.10)
										   )
		: Header(sizeof(USBCDCDescriptorFunctionalHeader_t), kCSInterfaceDescriptor),
		Subtype(kCDCCSInterfaceHeader),
		CDCSpecificationBCD(_CDCSpecificationBCD)
		{};
	} ATTR_PACKED;

#pragma mark USBCDCDescriptorFunctionalACM_t
	/*  CDC class-specific Functional ACM Descriptor (LUFA naming conventions).
	 *
	 *  Type define for a CDC class-specific functional ACM descriptor. This indicates to the host that the CDC interface
	 *  supports the CDC ACM subclass of the CDC specification. See the CDC class specification for more details.
	 *
	 */
	struct USBCDCDescriptorFunctionalACM_t
	{
		USBDescriptorHeader_t Header; /*   Regular descriptor header containing the descriptor's type and length. */
		uint8_t                 Subtype; /*   Sub type value used to distinguish between CDC class-specific descriptors,
										  *   must be \ref CDC_DSUBTYPE_CSInterface_ACM.
										  */
		uint8_t                 Capabilities; /*   Capabilities of the ACM interface, given as a bit mask. For most devices,
											   *   this should be set to a fixed value of 0x06 - for other capabilities, refer
											   *   to the CDC ACM specification.
											   */
		USBCDCDescriptorFunctionalACM_t(
										uint8_t _Capabilities = 0x06
										)
		: Header(sizeof(USBCDCDescriptorFunctionalACM_t), kCSInterfaceDescriptor),
		Subtype(kCDCCSInterfaceACM),
		Capabilities(_Capabilities)
		{};
	} ATTR_PACKED;

#pragma mark USBCDCDescriptorFunctionalUnion_t
	/*  CDC class-specific Functional Union Descriptor (LUFA naming conventions).
	 *
	 *  Type define for a CDC class-specific functional Union descriptor. This indicates to the host that specific
	 *  CDC control and data interfaces are related. See the CDC class specification for more details.
	 *
	 */
	struct USBCDCDescriptorFunctionalUnion_t
	{
		USBDescriptorHeader_t Header; /*   Regular descriptor header containing the descriptor's type and length. */
		uint8_t                 Subtype; /*   Sub type value used to distinguish between CDC class-specific descriptors,
										  *   must be \ref CDC_DSUBTYPE_CSInterface_Union.
										  */
		uint8_t                 MasterInterfaceNumber; /*   Interface number of the CDC Control interface. */
		uint8_t                 SlaveInterfaceNumber; /*   Interface number of the CDC Data interface. */
		USBCDCDescriptorFunctionalUnion_t(
										  uint8_t _MasterInterfaceNumber
										  )
		: Header(sizeof(USBCDCDescriptorFunctionalUnion_t), kCSInterfaceDescriptor),
		Subtype(kCDCCSInterfaceUnion),
		MasterInterfaceNumber(_MasterInterfaceNumber),
		SlaveInterfaceNumber(_MasterInterfaceNumber+1)
		{};
	} ATTR_PACKED;


#pragma mark USBCDC

	// Placeholder for use in end-code
	// IOW: USBDevice<USBCDC> usb;
	// Also, used as the base class for the resulting specialized USBMixin.
	struct USBCDC {
		static bool isNull() { return false; };
		static const uint8_t endpoints_used = 3;
	};

#pragma mark USBCDC_impl

	//Actual implementation of CDC
	template <typename usb_parent_type>
	struct USBSerial {
		usb_parent_type &usb;
		const uint8_t control_endpoint;
		const uint8_t read_endpoint;
		const uint8_t write_endpoint;

		struct _line_info_t
		{
			uint32_t	dwDTERate;
			uint8_t		bCharFormat;
			uint8_t 	bParityType;
			uint8_t 	bDataBits;

			_line_info_t() :
				dwDTERate(57600),
				bCharFormat(0x00),
				bParityType(0x00),
				bDataBits(0x08)
			{};
		} ATTR_PACKED;

		volatile uint8_t _line_state;

		volatile _line_info_t _line_info;

		USBSerial(usb_parent_type &usb_parent,
					const uint8_t new_endpoint_offset
					)
		: usb(usb_parent),
		control_endpoint(new_endpoint_offset),
		read_endpoint(new_endpoint_offset+1),
		write_endpoint(new_endpoint_offset+2),
		_line_state(0x00)
		{};

		int16_t readByte() {
			return usb.readByte(read_endpoint);
		};

		uint16_t read(const uint8_t *buffer, const uint16_t length) {
			int16_t total_read = 0;
			int16_t to_read = length;
			const uint8_t *read_ptr = buffer;

			// BLOCKING!!
			while (to_read > 0) {
				// Oddity of english: "to read" and "amount read" makes the same read.
				// So, we'll call it "amount_read".
				int16_t amount_read = usb.read(read_endpoint, read_ptr, length);

				total_read += amount_read;
				to_read -= amount_read;
				read_ptr += amount_read;
			};

			return total_read;
		};

		int32_t write(const uint8_t *data, const uint16_t length) {
			int16_t total_written = 0;
			int16_t written = 1; // start with a non-zero value
			const uint8_t *out_buffer = data;
			int16_t to_write = length;

			// BLOCKING!!
			do {
				written = usb.write(write_endpoint, out_buffer, length);

				if (written < 0) // ERROR!
					break;

				// TODO: Do this better... -Rob
				total_written += written;
				to_write -= written;
				out_buffer += written;
			} while (to_write);

			// HACK! Autoflush forced...
			if (total_written > 0)
				flush();

			return total_written;
		}

		void flush() {
			usb.flush(write_endpoint);
		}

		bool isConnected() {
			return _line_state & (0x01 << 1);
		}

		bool getDTR() {
			return _line_state & (0x01 << 0);
		}

		bool getRTS() {
			return _line_state & (0x01 << 1);
		}

		bool handleNonstandardRequest(Setup_t &setup) {
			if (setup.isADeviceToHostClassInterfaceRequest()) {
				if (setup.requestIs(kGetLineEncoding)) {
					usb.writeToControl(usb.master_control_endpoint, (uint8_t*)&_line_info, sizeof(_line_info));
					return true;
				}
			}

			if (setup.isAHostToDeviceClassInterfaceRequest()) {
				if (setup.requestIs(kSetLineEncoding)) {
					usb.readFromControl(usb.master_control_endpoint, (uint8_t*)&_line_info, sizeof(_line_info));
					return true;
				}

				if (setup.requestIs(kSetControlLineState)) {
					_line_state = setup.valueLow();

					// Auto-reset into the bootloader is triggered when the port, already open at 1200 bps, is closed.

					// Note that it may be reopened immediately at a different rate.
					// That will *NOT* cancel the reset.
					if (1200 == _line_info.dwDTERate)
					{
						// We check DTR state to determine if host port is open (bit 0 of lineState).
						if ((getDTR()) == false)
							initiateReset(250);
//						else
//							 cancelReset();
					}

					return true;
				}
			}

			return false;
		};

		// Stub in begin() and end()
		void begin(uint32_t baud_count) {};
		void end(void){};

		const EndpointBufferSettings_t getEndpointSettings(const uint8_t endpoint, const bool otherSpeed) {
			if (endpoint == control_endpoint)
			{
				const EndpointBufferSettings_t _buffer_speed = getBufferSizeFlags(Motate::getEndpointSize(control_endpoint, kEndpointTypeControl, otherSpeed));
				return kEndpointBufferInputToHost | _buffer_speed | kEndpointBufferBlocks1 | kEndpointBufferTypeInterrupt;
			}
			else if (endpoint == read_endpoint)
			{
				const EndpointBufferSettings_t _buffer_speed = getBufferSizeFlags(Motate::getEndpointSize(read_endpoint, kEndpointTypeBulk, otherSpeed));
				return kEndpointBufferOutputFromHost | _buffer_speed | kEndpointBufferBlocksUpTo2 | kEndpointBufferTypeBulk;
			}
			else if (endpoint == write_endpoint)
			{
				const EndpointBufferSettings_t _buffer_speed = getBufferSizeFlags(Motate::getEndpointSize(write_endpoint, kEndpointTypeBulk, otherSpeed));
				return kEndpointBufferInputToHost | _buffer_speed | kEndpointBufferBlocksUpTo2 | kEndpointBufferTypeBulk;
			}
			return kEndpointBufferNull;
		};

		uint16_t getEndpointSize(const uint8_t &endpoint, const bool otherSpeed) {
			if (endpoint == control_endpoint)
			{
				return Motate::getEndpointSize(control_endpoint, kEndpointTypeInterrupt, otherSpeed);
			}
			else if (endpoint == read_endpoint)
			{
				return Motate::getEndpointSize(read_endpoint, kEndpointTypeBulk, otherSpeed);
			}
			else if (endpoint == write_endpoint)
			{
				return Motate::getEndpointSize(write_endpoint, kEndpointTypeBulk, otherSpeed);
			}
			return 0;
		};

	};

#pragma mark USBMixin< USBCDC, usbIFB, usbIFC, 0 >

	template <typename usbIFB, typename usbIFC>
	struct USBMixin< USBCDC, usbIFB, usbIFC, 0 > : USBCDC {

		typedef USBDevice<USBCDC, usbIFB, usbIFC> usb_parent_type;
		typedef USBMixin< USBCDC, usbIFB, usbIFC, 0 > this_type;

		USBSerial< usb_parent_type > Serial;

		USBMixin< USBCDC, usbIFB, usbIFC, 0 > (usb_parent_type &usb_parent,
											   const uint8_t new_endpoint_offset
											   ) : Serial(usb_parent, new_endpoint_offset) {};

		static const EndpointBufferSettings_t getEndpointConfigFromMixin(const uint8_t endpoint, const bool other_speed) {
			return usb_parent_type::_singleton->this_type::Serial.getEndpointSettings(endpoint, other_speed);
		};
		static bool handleNonstandardRequestInMixin(Setup_t &setup) {
			return usb_parent_type::_singleton->this_type::Serial.handleNonstandardRequest(setup);
		};
		static uint16_t getEndpointSizeFromMixin(const uint8_t endpoint, const bool otherSpeed) {
			return usb_parent_type::_singleton->this_type::Serial.getEndpointSize(endpoint, otherSpeed);
		};
		static bool sendSpecialDescriptorOrConfig(Setup_t &setup) { return false; };
	};

#pragma mark USBMixin< usbIFA, USBCDC, usbIFC, 1 >
	template <typename usbIFA, typename usbIFC>
	struct USBMixin< usbIFA, USBCDC, usbIFC, 1 > : USBCDC {

		typedef USBDevice<usbIFA, USBCDC, usbIFC> usb_parent_type;
		typedef USBMixin< usbIFA, USBCDC, usbIFC, 1 > this_type;

		USBSerial< usb_parent_type > Serial;

		USBMixin< usbIFA, USBCDC, usbIFC, 1 > (usb_parent_type &usb_parent,
											   const uint8_t new_endpoint_offset
											   ) : Serial(usb_parent, new_endpoint_offset) {};

		static const EndpointBufferSettings_t getEndpointConfigFromMixin(uint8_t endpoint, const bool other_speed) {
			return usb_parent_type::_singleton->this_type::Serial.getEndpointSettings(endpoint, other_speed);
		};
		static bool handleNonstandardRequestInMixin(Setup_t &setup) {
			return usb_parent_type::_singleton->this_type::Serial.handleNonstandardRequest(setup);
		};
		static uint16_t getEndpointSizeFromMixin(const uint8_t endpoint, const bool otherSpeed) {
			return usb_parent_type::_singleton->this_type::Serial.getEndpointSize(endpoint, otherSpeed);
		};
		static bool sendSpecialDescriptorOrConfig(Setup_t &setup) { return false; };
	};

#pragma mark USBMixin< usbIFA, usbIFB, USBCDC, 2 >
	template <typename usbIFA, typename usbIFB>
	struct USBMixin< usbIFA, usbIFB, USBCDC, 2 > : USBCDC {

		typedef USBDevice<usbIFA, usbIFB, USBCDC> usb_parent_type;
		typedef USBMixin< usbIFA, usbIFB, USBCDC, 2 > this_type;

		USBSerial< usb_parent_type > Serial;

		USBMixin< usbIFA, usbIFB, USBCDC, 2 > (usb_parent_type &usb_parent,
											   const uint8_t new_endpoint_offset
											   ) : Serial(usb_parent, new_endpoint_offset) {};

		static const EndpointBufferSettings_t getEndpointConfigFromMixin(uint8_t endpoint, const bool other_speed) {
			return usb_parent_type::_singleton->this_type::Serial.getEndpointSettings(endpoint, other_speed);
		};
		static bool handleNonstandardRequestInMixin(Setup_t &setup) {
			return usb_parent_type::_singleton->this_type::Serial.handleNonstandardRequest(setup);
		};
		static uint16_t getEndpointSizeFromMixin(const uint8_t endpoint, const bool otherSpeed) {
			return usb_parent_type::_singleton->this_type::Serial.getEndpointSize(endpoint, otherSpeed);
		};
		static bool sendSpecialDescriptorOrConfig(Setup_t &setup) { return false; };
	};

#pragma mark USBDefaultDescriptor < USBCDC, USBNullInterface, USBNullInterface >

	// The descriptor for CDC has some odd rules, compared to other interfaces, since it's composite interface:
	//  1- If the ONLY interface is a CDC interface, then we explicity say as much in the device descriptor proper
	//  2- If there are any other interfaces (in any position), then we need to specify that we're using an Interface
	//     Associaton Descriptor (IAD). (Is this correct?? -Rob)

	// The configuration (below) has a very similar setup.

	// Case 1, we have one CDC interface and the other two are USBNullInterfaces
	template <  >
	struct USBDefaultDescriptor < USBCDC, USBNullInterface, USBNullInterface > : USBDescriptorDevice_t {
		USBDefaultDescriptor(const uint16_t vendorID, const uint16_t productID, const uint16_t productVersion) :
		USBDescriptorDevice_t(
							  /*    USBSpecificationBCD = */ USBFloatToBCD(1.1),
							  /*                  Class = */ kNoDeviceClass,
							  /*               SubClass = */ kNoSpecificSubclass,
							  /*               Protocol = */ kNoSpecificProtocol,

							  /*          Endpoint0Size = */ getEndpointSize(0, kEndpointTypeControl, false),

							  /*               VendorID = */ vendorID,
							  /*              ProductID = */ productID,
							  /*          ReleaseNumber = */ productVersion,

							  /*   ManufacturerStrIndex = */ kManufacturerStringId,
							  /*        ProductStrIndex = */ kProductStringId,
							  /*      SerialNumStrIndex = */ kSerialNumberId,

							  /* NumberOfConfigurations = */ 1
							  )
		{};
	};

#pragma mark USBCDCIADDescriptor

	// Case 2, we have one CDC interface and at least one other non-null interface
	// Since this is actually four different combinations of template, we make a base class and inherit from it.
	struct USBCDCIADDescriptor : USBDescriptorDevice_t {
		USBCDCIADDescriptor(const uint16_t vendorID, const uint16_t productID, const uint16_t productVersion) :
		USBDescriptorDevice_t(
							  /*    USBSpecificationBCD = */ USBFloatToBCD(1.1),
							  /*                  Class = */ kIADDeviceClass,
							  /*               SubClass = */ kIADDeviceSubclass,
							  /*               Protocol = */ kIADDeviceProtocol,

							  /*          Endpoint0Size = */ getEndpointSize(0, kEndpointTypeControl, false),

							  /*               VendorID = */ vendorID,
							  /*              ProductID = */ productID,
							  /*          ReleaseNumber = */ productVersion,

							  /*   ManufacturerStrIndex = */ kManufacturerStringId,
							  /*        ProductStrIndex = */ kProductStringId,
							  /*      SerialNumStrIndex = */ kSerialNumberId,

							  /* NumberOfConfigurations = */ 1
							  )
		{};
	};

	// CDC is the first interface...
	template < typename usbIFB, typename usbIFC >
	struct USBDefaultDescriptor < USBCDC, usbIFB, usbIFC > : USBCDCIADDescriptor {
		USBDefaultDescriptor(const uint16_t vendorID, const uint16_t productID, const uint16_t productVersion) :
		USBCDCIADDescriptor(vendorID, productID, productVersion)
		{};
	};

	// CDC is the second interface...
	template < typename usbIFA, typename usbIFC >
	struct USBDefaultDescriptor < usbIFA, USBCDC, usbIFC > : USBCDCIADDescriptor {
		USBDefaultDescriptor(const uint16_t vendorID, const uint16_t productID, const uint16_t productVersion) :
		USBCDCIADDescriptor(vendorID, productID, productVersion)
		{};
	};

	// CDC is the third interface...
	template < typename usbIFA, typename usbIFB >
	struct USBDefaultDescriptor < usbIFA, usbIFB, USBCDC > : USBCDCIADDescriptor {
		USBDefaultDescriptor(const uint16_t vendorID, const uint16_t productID, const uint16_t productVersion) :
		USBCDCIADDescriptor(vendorID, productID, productVersion)
		{};
	};

	// CDC is the first and second interface...
	template < typename usbIFC >
	struct USBDefaultDescriptor < USBCDC, USBCDC, usbIFC > : USBCDCIADDescriptor {
		USBDefaultDescriptor(const uint16_t vendorID, const uint16_t productID, const uint16_t productVersion) :
		USBCDCIADDescriptor(vendorID, productID, productVersion)
		{};
	};
	// CDC is the second and third interface...
	template < typename usbIFA >
	struct USBDefaultDescriptor < usbIFA, USBCDC, USBCDC > : USBCDCIADDescriptor {
		USBDefaultDescriptor(const uint16_t vendorID, const uint16_t productID, const uint16_t productVersion) :
		USBCDCIADDescriptor(vendorID, productID, productVersion)
		{};
	};


#pragma mark USBConfigMixin< USBCDC, ?, ? >

	// Define the CDC ConfigMixins

	// The configuration for CDC has similar rules to the descriptor, since it's composite interface:
	//  1- If the ONLY interface is a CDC interface, then we can just present the interfaces, and the descriptor will
	//     instruct the host that they need to be bound.
	//  2- If there are any other interfaces (in any position), then we need to insert an IAD before any CDC interface
	//     to inform the host to associate the two interfaces to one driver.

	
	// Case 1: CDC only
	template <  >
	struct USBConfigMixin< USBCDC, USBNullInterface, USBNullInterface, 0 >
	{
		static const uint8_t interfaces = 2;
		
		// CDC Control Interface
		const USBDescriptorInterface_t           CDC_CCI_Interface;
		const USBCDCDescriptorFunctionalHeader_t CDC_Functional_Header;
		const USBCDCDescriptorFunctionalACM_t    CDC_Functional_ACM;
		const USBCDCDescriptorFunctionalUnion_t  CDC_Functional_Union;
		const USBDescriptorEndpoint_t            CDC_NotificationEndpoint;

		// CDC Data Interface
		const USBDescriptorInterface_t CDC_DCI_Interface;
		const USBDescriptorEndpoint_t CDC_DataOutEndpoint;
		const USBDescriptorEndpoint_t CDC_DataInEndpoint;
		
		
		USBConfigMixin (const uint8_t _first_endpoint_number, const uint8_t _first_interface_number, const bool _other_speed)
		: CDC_CCI_Interface(
								   /* _InterfaceNumber   = */ _first_interface_number,
								   /* _AlternateSetting  = */ 0,
								   /* _TotalEndpoints    = */ 1,

								   /* _Class             = */ kCDCClass,
								   /* _SubClass          = */ kACMSubclass,
								   /* _Protocol          = */ kATCommandProtocol,

								   /* _InterfaceStrIndex = */ 0 // none
								   ),
		CDC_Functional_Header(),
		CDC_Functional_ACM(),
		CDC_Functional_Union(_first_interface_number),
		CDC_NotificationEndpoint(
								 /* _otherSpeed        = */ _other_speed,
								 /* _input             = */ true,
								 /* _EndpointAddress   = */ _first_endpoint_number,
								 /* _Attributes        = */ (kEndpointTypeInterrupt | kEndpointAttrNoSync | kEndpointUsageData),
								 /* _PollingIntervalMS = */ 0xFF,
								 /* _maxSize (optional) =*/ 64
								 ),

		
		CDC_DCI_Interface(
						  /* _InterfaceNumber   = */ _first_interface_number+1,
						  /* _AlternateSetting  = */ 0,
						  /* _TotalEndpoints    = */ 2,

						  /* _Class             = */ kCDCDataClass,
						  /* _SubClass          = */ kNoDataSubclass,
						  /* _Protocol          = */ kNoDataProtocol,

						  /* _InterfaceStrIndex = */ 0 // none
						 ),
		CDC_DataOutEndpoint(
							/* _otherSpeed        = */ _other_speed,
							/* _input             = */ false,
							/* _EndpointAddress   = */ _first_endpoint_number+1,
							/* _Attributes        = */ (kEndpointTypeBulk | kEndpointAttrNoSync | kEndpointUsageData),
							/* _PollingIntervalMS = */ 0x05
							),
		CDC_DataInEndpoint(
						   /* _otherSpeed        = */ _other_speed,
						   /* _input             = */ true,
						   /* _EndpointAddress   = */ _first_endpoint_number+2,
						   /* _Attributes        = */ (kEndpointTypeBulk | kEndpointAttrNoSync | kEndpointUsageData),
						   /* _PollingIntervalMS = */ 0x05
						   )
		{};
		
		static bool isNull() { return false; };
	};

	// Case 2, we have one CDC interface and at least one other non-null interface
	// Since this is actually four different combinations of template, we make a base class and inherit from it.
	struct USBConfigMixinMultiple_t
	{
		static const uint8_t interfaces = 2;

		const USBDescriptorInterfaceAssociation_t CDC_IAD;

		// CDC Control Interface
		const USBDescriptorInterface_t            CDC_CCI_Interface;
		const USBCDCDescriptorFunctionalHeader_t  CDC_Functional_Header;
		const USBCDCDescriptorFunctionalACM_t     CDC_Functional_ACM;
		const USBCDCDescriptorFunctionalUnion_t   CDC_Functional_Union;
		const USBDescriptorEndpoint_t             CDC_NotificationEndpoint;

		// CDC Data Interface
		const USBDescriptorInterface_t CDC_DCI_Interface;
		const USBDescriptorEndpoint_t CDC_DataOutEndpoint;
		const USBDescriptorEndpoint_t CDC_DataInEndpoint;

		USBConfigMixinMultiple_t (const uint8_t _first_endpoint_number, const uint8_t _first_interface_number, const bool _other_speed)
		: CDC_IAD (
				  /* _FirstInterfaceIndex = */ _first_interface_number,
				  /* _TotalInterfaces     = */ 2,
				  /* _Class               = */ kCDCClass,
				  /* _SubClass            = */ kACMSubclass,
				  /* _Protocol            = */ kATCommandProtocol,
				  /* _IADStrIndex         = */ 0 // none
				  ),
		CDC_CCI_Interface(
						 /* _InterfaceNumber   = */ _first_interface_number,
						 /* _AlternateSetting  = */ 0,
						 /* _TotalEndpoints    = */ 1,
                         
						 /* _Class             = */ kCDCClass,
						 /* _SubClass          = */ kACMSubclass,
						 /* _Protocol          = */ kATCommandProtocol,
                         
						 /* _InterfaceStrIndex = */ 0 // none
						 ),
		CDC_Functional_Header(),
		CDC_Functional_ACM(),
		CDC_Functional_Union(_first_interface_number),
		CDC_NotificationEndpoint(
								 /* _otherSpeed        = */ _other_speed,
								 /* _input             = */ true,
								 /* _EndpointAddress   = */ _first_endpoint_number,
								 /* _Attributes        = */ (kEndpointTypeInterrupt | kEndpointAttrNoSync | kEndpointUsageData),
								 /* _PollingIntervalMS = */ 0xFF,
								 /* _maxSize (optional) =*/ 64
								 ),

		CDC_DCI_Interface(
						 /* _InterfaceNumber   = */ _first_interface_number+1,
						 /* _AlternateSetting  = */ 0,
						 /* _TotalEndpoints    = */ 2,

						 /* _Class             = */ kCDCDataClass,
						 /* _SubClass          = */ kNoDataSubclass,
						 /* _Protocol          = */ kNoDataProtocol,

						 /* _InterfaceStrIndex = */ 0 // none
						 ),
		CDC_DataOutEndpoint(
							/* _otherSpeed        = */ _other_speed,
							/* _input             = */ false,
							/* _EndpointAddress   = */ _first_endpoint_number+1,
							/* _Attributes        = */ (kEndpointTypeBulk | kEndpointAttrNoSync | kEndpointUsageData),
							/* _PollingIntervalMS = */ 0x05
							),
		CDC_DataInEndpoint(
						   /* _otherSpeed        = */ _other_speed,
						   /* _input             = */ true,
						   /* _EndpointAddress   = */ _first_endpoint_number+2,
						   /* _Attributes        = */ (kEndpointTypeBulk | kEndpointAttrNoSync | kEndpointUsageData),
						   /* _PollingIntervalMS = */ 0x05
						   )
		{};

		static bool isNull() { return false; };
	};

	// CDC is the first interface...
	template < typename usbIFB, typename usbIFC >
	struct USBConfigMixin < USBCDC, usbIFB, usbIFC, 0 > : USBConfigMixinMultiple_t {
		USBConfigMixin(const uint8_t _first_endpoint_number, const uint8_t _first_interface_number, const bool _other_speed) :
		USBConfigMixinMultiple_t(_first_endpoint_number, _first_interface_number, _other_speed)
		{};
	};

	// CDC is the second interface...
	template < typename usbIFA, typename usbIFC >
	struct USBConfigMixin < usbIFA, USBCDC, usbIFC, 1 > : USBConfigMixinMultiple_t {
		USBConfigMixin(const uint8_t _first_endpoint_number, const uint8_t _first_interface_number, const bool _other_speed) :
		USBConfigMixinMultiple_t(_first_endpoint_number, _first_interface_number, _other_speed)
		{};
	};

	// CDC is the third interface...
	template < typename usbIFA, typename usbIFB >
	struct USBConfigMixin < usbIFA, usbIFB, USBCDC, 2 > : USBConfigMixinMultiple_t {
		USBConfigMixin(const uint8_t _first_endpoint_number, const uint8_t _first_interface_number, const bool _other_speed) :
		USBConfigMixinMultiple_t(_first_endpoint_number, _first_interface_number, _other_speed)
		{};
	};

	 // CDC is the first and second interface...
//	 template < typename usbIFC >
//	 struct USBConfigMixin < USBCDC, USBCDC, usbIFC, 0> : USBConfigMixinMultiple_t {
//		 USBConfigMixin(const uint8_t _first_endpoint_number, const uint8_t _first_interface_number, const bool _other_speed) :
//		 USBConfigMixinMultiple_t(_first_endpoint_number, _first_interface_number, _other_speed)
//		{};
//	 };
	// // CDC is the second and third interface...
	// template < typename usbIFA >
	// struct USBConfigMixin < usbIFA, USBCDC, USBCDC > : USBConfigMixinMultiple_t {
	// 	USBConfigMixin(const uint8_t _first_endpoint_number, const uint8_t _first_interface_number) :
	// 	USBConfigMixinMultiple_t(_first_endpoint_number, _first_interface_number)
	// 	{};
	// };

}

#endif
// MOTATEUSBCDC_ONCE
