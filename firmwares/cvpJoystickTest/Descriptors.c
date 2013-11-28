/*
             LUFA Library
     Copyright (C) Dean Camera, 2010.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com
*/

/*
  Copyright 2010  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this 
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in 
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting 
  documentation, and that the name of the author not be used in 
  advertising or publicity pertaining to distribution of the 
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  USB Device Descriptors, for library use when in USB device mode. Descriptors are special 
 *  computer-readable structures which the host requests upon device enumeration, to determine
 *  the device's capabilities and functions.  
 */

#include "Descriptors.h"

/** HID class report descriptor. This is a special descriptor constructed with values from the
 *  USBIF HID class specification to describe the reports and capabilities of the HID device. This
 *  descriptor is parsed by the host and its contents used to determine what data (and in what encoding)
 *  the device will send, and what it may be sent back from the host. Refer to the HID specification for
 *  more details on HID report descriptors.
 */
const USB_Descriptor_HIDReport_Datatype_t PROGMEM Joystick1Report[] =
{
	0x05, 0x01,          /* Usage Page (Generic Desktop)                       */
	0x09, 0x04,          /* Usage (Joystick)                                   */

	0xa1, 0x01,          /* Collection (Application)                           */
	0x09, 0x01,          /*   Usage (Pointer)                                  */

	/* 8 axes, signed 16 bit resolution, the actualy resolution is still 10 bit range 0~65535  */
	0xa1, 0x00,          /*   Collection (Physical)                            */
	0x05, 0x01,          /*     Usage Page (Generic Desktop)                   */
	0x09, 0x30,          /*     Usage (X)                                      */
	0x09, 0x31,          /*     Usage (Y)                                      */
	0x09, 0x32,          /*     Usage (Analog1)                                */
	0x09, 0x33,          /*     Usage (Analog2)                                */
	0x09, 0x34,          /*     Usage (Analog3)                                */
	0x09, 0x35,          /*     Usage (Analog4)                                */
	0x09, 0x36,          /*     Usage (Analog5)                                */
	0x09, 0x37,          /*     Usage (Analog6)                                */
	0x27, 0xff, 0xff, 0x00, 0x00,    /*     Logical Maximum (65535)                        */
	0x47, 0xff, 0xff, 0x00, 0x00,    /*     Physical Maximum (65535)                        */
	0x75, 16,            /*     Report Size (16)                               */
	0x95, 8,             /*     Report Count (8)                               */
	0x81, 0x82,          /*     Input (Data, Variable, Absolute, Volatile)     */
	0xc0,                /*   End Collection                                   */

	/* 128 buttons, value 0=off, 1=on */
	0x05, 0x09,          /*   Usage Page (Button)                              */
	0x19, 1,             /*     Usage Minimum (Button 1)                       */
	0x29, 128,            /*     Usage Maximum (Button 128 )                      */
	0x15, 0x00,          /*   Logical Minimum (0)                              */
	0x25, 0x01,          /*   Logical Maximum (1)                              */
	0x35, 0x00,          /*   Physical Minimum (0)                              */
	0x45, 0x01,          /*   Physical Maximum (1)                              */
	0x75, 1,             /*   Report Size (1)                                  */
	0x95, 128,            /*   Report Count (128)                                */
	0x81, 0x02,          /*   Input (Data, Variable, Absolute)                 */
	0xc0                 /* End Collection                                     */
};

const USB_Descriptor_HIDReport_Datatype_t PROGMEM Joystick2Report[] =
{
	0x05, 0x01,          /* Usage Page (Generic Desktop)                       */
	0x09, 0x04,          /* Usage (Joystick)                                   */

	0xa1, 0x01,          /* Collection (Application)                           */

	0x05, 0x09,          /*   Usage Page (Button)                              */
	0x19, 1,             /*     Usage Minimum (Button 1)                       */
	0x29, 128,            /*     Usage Maximum (Button 128 )                      */
	0x15, 0x00,          /*   Logical Minimum (0)                              */
	0x25, 0x01,          /*   Logical Maximum (1)                              */
	0x35, 0x00,          /*   Physical Minimum (0)                              */
	0x45, 0x01,          /*   Physical Maximum (1)                              */
	0x75, 1,             /*   Report Size (1)                                  */
	0x95, 128,            /*   Report Count (128)                                */
	0x81, 0x02,          /*   Input (Data, Variable, Absolute)                 */
	0xc0                 /* End Collection                                     */
};

/** Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins.
 */
const USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},
		
	.USBSpecification       = VERSION_BCD(02.00),
	.Class                  = 0x00,
	.SubClass               = 0x00,
	.Protocol               = 0x00,
	
	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,
		
	.VendorID               = 0x03EB,
	.ProductID              = 0x2043,
	.ReleaseNumber          = 0x0003,
		
	.ManufacturerStrIndex   = 0x01,
	.ProductStrIndex        = 0x02,
	.SerialNumStrIndex      = 0x03,
		
	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/** Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device.
 */
const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
	.Config = 
	{
			.Header                 = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

			.TotalConfigurationSize = sizeof(USB_Descriptor_Configuration_t),
			.TotalInterfaces        = 2,
				
			.ConfigurationNumber    = 1,
			.ConfigurationStrIndex  = NO_DESCRIPTOR,
				
			.ConfigAttributes       = (USB_CONFIG_ATTR_BUSPOWERED),
			
			.MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
	},
		
	.HID_Interface1 = 
	{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = 0x00,
			.AlternateSetting       = 0x00,
			
			.TotalEndpoints         = 1,
				
			.Class                  = 0x03,
			.SubClass               = 0x00,
			.Protocol               = HID_CSCP_NonBootProtocol,
				
			.InterfaceStrIndex      = NO_DESCRIPTOR
	},

	.HID_Joystick1HID = 
	{
			.Header                 = {.Size = sizeof(USB_HID_Descriptor_HID_t), .Type = HID_DTYPE_HID},
			.HIDSpec                = VERSION_BCD(01.11),
			.CountryCode            = 0x00,
			.TotalReportDescriptors = 1,
			.HIDReportType          = HID_DTYPE_Report,
			.HIDReportLength        = sizeof(Joystick1Report)
	},

	.HID_J1_ReportINEndpoint = 
	{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			
			.EndpointAddress        = (ENDPOINT_DIR_IN | JOYSTICK1_EPNUM),
			.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = 32,
			.PollingIntervalMS      = 0x02
	},

	.HID_Interface2 = 
	{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = 0x01,
			.AlternateSetting       = 0x00,
			
			.TotalEndpoints         = 1,
				
			.Class                  = 0x03,
			.SubClass               = 0x00,
			.Protocol               = HID_CSCP_NonBootProtocol,
				
			.InterfaceStrIndex      = NO_DESCRIPTOR
	},

	.HID_Joystick2HID = 
	{
			.Header                 = {.Size = sizeof(USB_HID_Descriptor_HID_t), .Type = HID_DTYPE_HID},
			.HIDSpec                = VERSION_BCD(01.11),
			.CountryCode            = 0x00,
			.TotalReportDescriptors = 1,
			.HIDReportType          = HID_DTYPE_Report,
			.HIDReportLength        = sizeof(Joystick2Report)
	},

	.HID_J2_ReportINEndpoint = 
	{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			
			.EndpointAddress        = (ENDPOINT_DIR_IN | JOYSTICK2_EPNUM),
			.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = 32,
			.PollingIntervalMS      = 0x02
	},

};

/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
const USB_Descriptor_String_t PROGMEM LanguageString =
{
	.Header                 = {.Size = USB_STRING_LEN(1), .Type = DTYPE_String},
		
	.UnicodeString          = {LANGUAGE_ID_ENG}
};

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
const USB_Descriptor_String_t PROGMEM ManufacturerString =
{
	.Header                 = {.Size = USB_STRING_LEN(7), .Type = DTYPE_String},
		
	.UnicodeString          = L"Arduino"
};

/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
const USB_Descriptor_String_t PROGMEM ProductString =
{
	.Header                 = {.Size = USB_STRING_LEN(14), .Type = DTYPE_String},
		
	.UnicodeString          = L"C.V.P Joystick"
};

/* Serial Number */
const USB_Descriptor_String_t PROGMEM SerialNumberString =
{
	.Header                 = {.Size = USB_STRING_LEN(2), .Type = DTYPE_String},
		
	.UnicodeString          = L"C2"
};
/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	void*    Address = NULL;
	uint16_t Size    = NO_DESCRIPTOR;

	switch (DescriptorType)
	{
		case DTYPE_Device:
			Address = (void*)&DeviceDescriptor;
			Size    = sizeof(USB_Descriptor_Device_t);
			break;
		case DTYPE_Configuration: 
			Address = (void*)&ConfigurationDescriptor;
			Size    = sizeof(USB_Descriptor_Configuration_t);
			break;
		case DTYPE_String: 
			switch (DescriptorNumber)
			{
				case 0x00: 
					Address = (void*)&LanguageString;
					Size    = pgm_read_byte(&LanguageString.Header.Size);
					break;
				case 0x01: 
					Address = (void*)&ManufacturerString;
					Size    = pgm_read_byte(&ManufacturerString.Header.Size);
					break;
				case 0x02: 
					Address = (void*)&ProductString;
					Size    = pgm_read_byte(&ProductString.Header.Size);
					break;
				case 0x03:
					Address = (void*)&SerialNumberString;
					Size 	= pgm_read_byte(&SerialNumberString.Header.Size);
					break;
			}
			
			break;
		case HID_DTYPE_HID: 
			switch( wIndex )
			{
			case 0:
				Address = (void*)&ConfigurationDescriptor.HID_Joystick1HID;
				Size    = sizeof(USB_HID_Descriptor_HID_t);
				break;
			case 1:
				Address = (void*)&ConfigurationDescriptor.HID_Joystick2HID;
				Size    = sizeof(USB_HID_Descriptor_HID_t);
				break;
			}
			break;
		case HID_DTYPE_Report: 
			switch( wIndex  )
			{
			case 0:
				Address = (void*)&Joystick1Report;
				Size    = sizeof(Joystick1Report);
				break;
			case 1:
				Address = (void*)&Joystick2Report;
				Size    = sizeof(Joystick2Report);
				break;
			}
			break;
	}
	*DescriptorAddress = Address;
	return Size;
}
