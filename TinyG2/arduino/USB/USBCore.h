// Copyright (c) 2010, Peter Barrett
/*
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#ifndef __USBCORE_H__
#define __USBCORE_H__

//	Standard requests
#define GET_STATUS					0
#define CLEAR_FEATURE				1
#define SET_FEATURE					3
#define SET_ADDRESS					5
#define GET_DESCRIPTOR				6
#define SET_DESCRIPTOR				7
#define GET_CONFIGURATION			8
#define SET_CONFIGURATION			9
#define GET_INTERFACE				10
#define SET_INTERFACE				11


// bmRequestType
#define REQUEST_HOSTTODEVICE		0x00
#define REQUEST_DEVICETOHOST		0x80
#define REQUEST_DIRECTION			0x80

#define REQUEST_STANDARD			0x00
#define REQUEST_CLASS				0x20
#define REQUEST_VENDOR				0x40
#define REQUEST_TYPE				0x60

#define REQUEST_DEVICE				0x00
#define REQUEST_INTERFACE			0x01
#define REQUEST_ENDPOINT			0x02
#define REQUEST_OTHER				0x03
#define REQUEST_RECIPIENT			0x1F

#define REQUEST_DEVICETOHOST_CLASS_INTERFACE  (REQUEST_DEVICETOHOST + REQUEST_CLASS + REQUEST_INTERFACE)
#define REQUEST_HOSTTODEVICE_CLASS_INTERFACE  (REQUEST_HOSTTODEVICE + REQUEST_CLASS + REQUEST_INTERFACE)

//	Class requests

#define CDC_SET_LINE_CODING			0x20
#define CDC_GET_LINE_CODING			0x21
#define CDC_SET_CONTROL_LINE_STATE	0x22

#define MSC_RESET					0xFF
#define MSC_GET_MAX_LUN				0xFE

#define HID_GET_REPORT				0x01
#define HID_GET_IDLE				0x02
#define HID_GET_PROTOCOL			0x03
#define HID_SET_REPORT				0x09
#define HID_SET_IDLE				0x0A
#define HID_SET_PROTOCOL			0x0B

//	Descriptors

#define USB_DEVICE_DESC_SIZE 18
#define USB_CONFIGUARTION_DESC_SIZE 9
#define USB_INTERFACE_DESC_SIZE 9
#define USB_ENDPOINT_DESC_SIZE 7

#define USB_DEVICE_DESCRIPTOR_TYPE             1
#define USB_CONFIGURATION_DESCRIPTOR_TYPE      2
#define USB_STRING_DESCRIPTOR_TYPE             3
#define USB_INTERFACE_DESCRIPTOR_TYPE          4
#define USB_ENDPOINT_DESCRIPTOR_TYPE           5
#define USB_DEVICE_QUALIFIER                   6
#define USB_OTHER_SPEED_CONFIGURATION          7

#define USB_DEVICE_CLASS_COMMUNICATIONS        0x02
#define USB_DEVICE_CLASS_HUMAN_INTERFACE       0x03
#define USB_DEVICE_CLASS_STORAGE               0x08
#define USB_DEVICE_CLASS_VENDOR_SPECIFIC       0xFF

#define USB_CONFIG_POWERED_MASK                0x40
#define USB_CONFIG_BUS_POWERED                 0x80
#define USB_CONFIG_SELF_POWERED                0xC0
#define USB_CONFIG_REMOTE_WAKEUP               0x20

// bMaxPower in Configuration Descriptor
#define USB_CONFIG_POWER_MA(mA)                ((mA)/2)

// bEndpointAddress in Endpoint Descriptor
#define USB_ENDPOINT_DIRECTION_MASK            0x80
#define USB_ENDPOINT_OUT(addr)                 ((addr) | 0x00)
#define USB_ENDPOINT_IN(addr)                  ((addr) | 0x80)

#define USB_ENDPOINT_TYPE_MASK                 0x03
#define USB_ENDPOINT_TYPE_CONTROL              0x00
#define USB_ENDPOINT_TYPE_ISOCHRONOUS          0x01
#define USB_ENDPOINT_TYPE_BULK                 0x02
#define USB_ENDPOINT_TYPE_INTERRUPT            0x03

#define TOBYTES(x) ((x) & 0xFF),(((x) >> 8) & 0xFF)

#define CDC_V1_10                               0x0110
#define CDC_COMMUNICATION_INTERFACE_CLASS       0x02

#define CDC_CALL_MANAGEMENT                     0x01
#define CDC_ABSTRACT_CONTROL_MODEL              0x02
#define CDC_HEADER                              0x00
#define CDC_ABSTRACT_CONTROL_MANAGEMENT         0x02
#define CDC_UNION                               0x06
#define CDC_CS_INTERFACE                        0x24
#define CDC_CS_ENDPOINT                         0x25
#define CDC_DATA_INTERFACE_CLASS                0x0A

#define MSC_SUBCLASS_SCSI						0x06
#define MSC_PROTOCOL_BULK_ONLY					0x50

#define HID_HID_DESCRIPTOR_TYPE					0x21
#define HID_REPORT_DESCRIPTOR_TYPE				0x22
#define HID_PHYSICAL_DESCRIPTOR_TYPE			0x23

_Pragma("pack(1)")

//	Device
class DeviceDescriptor {
public:
	uint8_t  len;				// 18
	uint8_t  dtype;				// 1 USB_DEVICE_DESCRIPTOR_TYPE
	uint16_t usbVersion;		// 0x200
	uint8_t  deviceClass;
	uint8_t  deviceSubClass;
	uint8_t  deviceProtocol;
	uint8_t  packetSize0;		// Packet 0
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t deviceVersion;	// 0x100
	uint8_t  iManufacturer;
	uint8_t  iProduct;
	uint8_t  iSerialNumber;
	uint8_t  bNumConfigurations;

	DeviceDescriptor(
					 uint8_t _class,
					 uint8_t _subClass,
					 uint8_t _proto,
					 uint8_t _packetSize0,
					 uint16_t _vid,
					 uint16_t _pid,
					 uint16_t _version,
					 uint8_t _im,
					 uint8_t _ip,
					 uint8_t _is,
					 uint8_t _configs
					 )
	:
	len(18),
	dtype(1 /* USB_DEVICE_DESCRIPTOR_TYPE */),
	usbVersion(0x200 /* Little Endien */),
	deviceClass(_class),
	deviceSubClass(_subClass),
	deviceProtocol(_proto),
	packetSize0(_packetSize0),
	idVendor(_vid),
	idProduct(_pid),
	deviceVersion(_version),
	iManufacturer(_im),
	iProduct(_ip),
	iSerialNumber(_is),
	bNumConfigurations(_configs)
	{
	};
};

class DeviceQualifier {
public:
	uint8_t  len;				// 10
	uint8_t  dtype;				// 6 ???
	uint16_t usbVersion;		// 0x200
	uint8_t  deviceClass;
	uint8_t  deviceSubClass;
	uint8_t  deviceProtocol;
	uint8_t  packetSize0;		// Packet 0
	uint8_t  bNumConfigurations;

	DeviceQualifier(
					 uint8_t _class,
					 uint8_t _subClass,
					 uint8_t _proto,
					 uint8_t _packetSize0,
					 uint8_t _configs
					)
	:
	len(10),
	dtype(6 /* ??? */),
	usbVersion(0x200 /* Little Endien */),
	deviceClass(_class),
	deviceSubClass(_subClass),
	deviceProtocol(_proto),
	packetSize0(_packetSize0),
	bNumConfigurations(_configs)
	{ /* Nothing to do here */ };
};


//	Config
class ConfigDescriptor {
public:
	enum _cd_otherOrNot {
		kConfig = 0,
		kOtherConfig = 1,
	};
	
	uint8_t	len;			// 9
	uint8_t	dtype;			// 2 or 7 (other)
	uint16_t clen;			// total length
	uint8_t	numInterfaces;
	uint8_t	config;
	uint8_t	iconfig;
	uint8_t	attributes;
	uint8_t	maxPower;

	ConfigDescriptor(
					 _cd_otherOrNot _other,
					 uint16_t _totalLength,
					 uint8_t _interfaces
					 )
	:
	len(9),
	dtype(!_other ? 2 : 7),
	clen(_totalLength),
	numInterfaces(_interfaces),
	config(1),
	iconfig(0),
	attributes(USB_CONFIG_SELF_POWERED),
	maxPower(USB_CONFIG_POWER_MA(500))
	{ /* Nothing to do here */ };
};

//	String

//	Interface
#if 0
#define D_INTERFACE(_n,_numEndpoints,_class,_subClass,_protocol) \
{ 9, 4, _n, 0, _numEndpoints, _class,_subClass, _protocol, 0 }
#endif
class InterfaceDescriptor {
public:

	uint8_t len;		// 9
	uint8_t dtype;		// 4
	uint8_t number;
	uint8_t alternate;
	uint8_t numEndpoints;
	uint8_t interfaceClass;
	uint8_t interfaceSubClass;
	uint8_t protocol;
	uint8_t iInterface;


	InterfaceDescriptor(
						uint8_t _number,
						uint8_t _numEndpoints,
						uint8_t _interfaceClass,
						uint8_t _interfaceSubClass,
						uint8_t _protocol
						)
	:
	len(9),
	dtype(4),
	number(_number),
	alternate(0),
	numEndpoints(_numEndpoints),
	interfaceClass(_interfaceClass),
	interfaceSubClass(_interfaceSubClass),
	protocol(_protocol),
	iInterface(0)
	{ /* Nothing to do here */ };
};

//	Endpoint
#if 0
#define D_ENDPOINT(_addr,_attr,_packetSize, _interval) \
{ 7, 5, _addr,_attr,_packetSize, _interval }
#endif

class EndpointDescriptor {
public:

	uint8_t len;		// 7
	uint8_t dtype;		// 5
	uint8_t addr;
	uint8_t attr;
	uint16_t packetSize;
	uint8_t interval;

	EndpointDescriptor(
					   uint8_t _addr,
					   uint8_t _attr,
					   uint16_t _packetSize,
					   uint8_t _interval
					   )
	:
	len(7),
	dtype(5),
	addr(_addr),
	attr(_attr),
	packetSize(_packetSize),
	interval(_interval)
	{ /* Nothing to do here */ };
};

// Interface Association Descriptor
// Used to bind 2 interfaces together in CDC compostite device
#if 0
#define D_IAD(_firstInterface, _count, _class, _subClass, _protocol) \
{ 8, 11, _firstInterface, _count, _class, _subClass, _protocol, 0 }
#endif
class IADDescriptor {
public:

	uint8_t len;				// 8
	uint8_t dtype;				// 11
	uint8_t firstInterface;
	uint8_t interfaceCount;
	uint8_t functionClass;
	uint8_t funtionSubClass;
	uint8_t functionProtocol;
	uint8_t iInterface;

	IADDescriptor(
				  uint8_t _firstInterface,
				  uint8_t _interfaceCount,
				  uint8_t _functionClass,
				  uint8_t _funtionSubClass,
				  uint8_t _functionProtocol
				  )
	:
	len(8),
	dtype(11),
	firstInterface(_firstInterface),
	interfaceCount(_interfaceCount),
	functionClass(_functionClass),
	funtionSubClass(_funtionSubClass),
	functionProtocol(_functionProtocol),
	iInterface(0)
	{ /* Nothing to do here */ };
} ;

//	CDC CS interface descriptor
#if 0
#define D_CDCCS(_subtype,_d0,_d1)	{ 5, 0x24, _subtype, _d0, _d1 }
#define D_CDCCS4(_subtype,_d0)		{ 4, 0x24, _subtype, _d0 }
#endif
class CDCCSInterfaceDescriptor {
public:
	uint8_t len;		// 5
	uint8_t dtype;		// 0x24
	uint8_t subtype;
	uint8_t d0;
	uint8_t d1;

	CDCCSInterfaceDescriptor(
							 uint8_t _subtype,
							 uint8_t _d0,
							 uint8_t _d1
				  )
	:
	len(5),
	dtype(0x24),
	subtype(_subtype),
	d0(_d0),
	d1(_d1)
	{ /* Nothing to do here */ };
};

#if 0
//UNUSED?
class CDCCSInterfaceDescriptor4 {
public:

	uint8_t len;		// 4
	uint8_t dtype;		// 0x24
	uint8_t subtype;
	uint8_t d0;

	CDCCSInterfaceDescriptor4(
							 uint8_t _subtype,
							 uint8_t _d0
							 )
	:
	len(4),
	dtype(0x24),
	subtype(_subtype),
	d0(_d0)
	{ /* Nothing to do here */ };
};
#endif

class CMFunctionalDescriptor {
public:

    uint8_t	len;
    uint8_t	dtype;		// 0x24
    uint8_t	subtype;	// 1
    uint8_t	bmCapabilities;
    uint8_t	bDataInterface;

	CMFunctionalDescriptor(
							 uint8_t _subtype,
							 uint8_t _bmCapabilities,
							 uint8_t _bDataInterface
							 )
	:
	len(5),
	dtype(0x24),
	subtype(_subtype),
	bmCapabilities(_bmCapabilities),
	bDataInterface(_bDataInterface)
	{ /* Nothing to do here */ };
} ;

class ACMFunctionalDescriptor {
public:
    uint8_t	len;
    uint8_t 	dtype;		// 0x24
    uint8_t 	subtype;	// 1
    uint8_t 	bmCapabilities;
	
	ACMFunctionalDescriptor(
							  uint8_t _subtype,
							  uint8_t _bmCapabilities
							  )
	:
	len(4),
	dtype(0x24),
	subtype(_subtype),
	bmCapabilities(_bmCapabilities)
	{ /* Nothing to do here */ };
};

typedef struct
{
	//	IAD
	IADDescriptor				iad;	// Only needed on compound device

	//	Control
	InterfaceDescriptor			cif;
	CDCCSInterfaceDescriptor	header;
	CMFunctionalDescriptor		callManagement;			// Call Management
	ACMFunctionalDescriptor		controlManagement;		// ACM
	CDCCSInterfaceDescriptor	functionalDescriptor;	// CDC_UNION
	EndpointDescriptor			cifin;

	//	Data
	InterfaceDescriptor			dif;
	EndpointDescriptor			in;
	EndpointDescriptor			out;
} CDCDescriptor;

/* UNUSED?
typedef struct
{
	InterfaceDescriptor			msc;
	EndpointDescriptor			in;
	EndpointDescriptor			out;
} MSCDescriptor;
*/

#if 0
#define D_HIDREPORT(_descriptorLength) \
{ 9, 0x21, 0x1, 0x1, 0, 1, 0x22, _descriptorLength, 0 }
#endif
class HIDDescDescriptor
{
public:
	uint8_t len;			// 9
	uint8_t dtype;			// 0x21
	uint8_t addr;
	uint8_t	versionL;		// 0x101
	uint8_t	versionH;		// 0x101
	uint8_t	country;
	uint8_t	desctype;		// 0x22 report
	uint8_t	descLenL;
	uint8_t	descLenH;

	HIDDescDescriptor(
						   uint16_t	_descriptorLength
						   )
	:
	len(9),
	dtype(0x21),
	addr(0x1),
	versionL(0x1),
	versionH(0x0),
	country(0x1),
	desctype(0x22),
	descLenL(_descriptorLength & 0xFF),
	descLenH((_descriptorLength & 0xFF00) >> 8)
	{ /* Nothing to do here */ };
};

typedef struct
{
	InterfaceDescriptor		hid;
	HIDDescDescriptor		desc;
	EndpointDescriptor		in;
} HIDDescriptor;

_Pragma("pack()")

#if 0
#define D_DEVICE(_class,_subClass,_proto,_packetSize0,_vid,_pid,_version,_im,_ip,_is,_configs) \
	{ 18, 1, 0x200, _class,_subClass,_proto,_packetSize0,_vid,_pid,_version,_im,_ip,_is,_configs }

#define D_CONFIG(_totalLength,_interfaces) \
	{ 9, 2, _totalLength,_interfaces, 1, 0, USB_CONFIG_SELF_POWERED, USB_CONFIG_POWER_MA(500) }

#define D_OTHERCONFIG(_totalLength,_interfaces) \
	{ 9, 7, _totalLength,_interfaces, 1, 0, USB_CONFIG_SELF_POWERED, USB_CONFIG_POWER_MA(500) }

#define D_INTERFACE(_n,_numEndpoints,_class,_subClass,_protocol) \
	{ 9, 4, _n, 0, _numEndpoints, _class,_subClass, _protocol, 0 }

#define D_ENDPOINT(_addr,_attr,_packetSize, _interval) \
	{ 7, 5, _addr,_attr,_packetSize, _interval }

#define D_QUALIFIER(_class,_subClass,_proto,_packetSize0,_configs) \
	{ 10, 6, 0x200, _class,_subClass,_proto,_packetSize0,_configs }

#define D_IAD(_firstInterface, _count, _class, _subClass, _protocol) \
	{ 8, 11, _firstInterface, _count, _class, _subClass, _protocol, 0 }

#define D_HIDREPORT(_descriptorLength) \
	{ 9, 0x21, 0x1, 0x1, 0, 1, 0x22, _descriptorLength, 0 }

#define D_CDCCS(_subtype,_d0,_d1)	{ 5, 0x24, _subtype, _d0, _d1 }
#define D_CDCCS4(_subtype,_d0)		{ 4, 0x24, _subtype, _d0 }

#endif

#endif
