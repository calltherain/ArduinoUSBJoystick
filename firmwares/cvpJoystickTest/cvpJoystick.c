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

/*-
 * Copyright (c) 2011 Darran Hunt (darran [at] hunt dot net dot nz)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file
 *
 *  Main source file for the Arduino-mouse project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "Arduino-joystick.h"

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
//uint8_t PrevJoystick1HIDReportBuffer[sizeof(USB_Joystick1Report_Data_t)];
//uint8_t PrevJoystick2HIDReportBuffer[sizeof(USB_Joystick2Report_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */

/** Circular buffer to hold data from the serial port before it is sent to the host. */
RingBuffer_t USARTtoUSB_Buffer;
const uint8_t BufferSize = sizeof(USB_Joystick1Report_Data_t ) + sizeof( USB_Joystick2Report_Data_t);
uint8_t USARTtoUSB_BufferStorage[ sizeof(USB_Joystick1Report_Data_t ) + sizeof( USB_Joystick2Report_Data_t) ];
uint8_t prevBufferData[ sizeof(USB_Joystick1Report_Data_t ) + sizeof( USB_Joystick2Report_Data_t)];


void Joystick1_Task(void);
void Joystick2_Task(void);


static uint16_t IdleCount, IdleRemaining;
static bool UsingReportProtocol = true;

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
	GlobalInterruptEnable();

	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_BufferStorage, sizeof( USARTtoUSB_BufferStorage) );

    	sei();

    	for (;;) {
		Joystick1_Task();
		Joystick2_Task();
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    /* Hardware Initialization */
    Serial_Init(115200, true);
    USB_Init();

    UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	UsingReportProtocol = true;
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
     Endpoint_ConfigureEndpoint(ENDPOINT_DIR_IN | JOYSTICK1_EPNUM, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);

     Endpoint_ConfigureEndpoint(ENDPOINT_DIR_IN | JOYSTICK2_EPNUM, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);

    USB_Device_EnableSOFEvents();
}

/** Event handler for the library USB Unhandled Control Request event. */
void EVENT_USB_Device_UnhandledControlRequest(void)
{
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	if ( IdleRemaining > 0)
		IdleRemaining--;	
}


void EVENT_USB_Device_ControlRequest(void)
{
        uint8_t* ReportData;
        uint8_t  ReportSize;

        /* Handle HID Class specific requests */

        switch (USB_ControlRequest.bRequest)
        {
        case HID_REQ_GetReport:
                if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
                {
                        Endpoint_ClearSETUP();

                        /* Determine if it is the joystick1 or 2 data that is being requested */
                        switch (USB_ControlRequest.wIndex)
        		{
        		case 0:
                                //ReportData = (uint8_t*)&joy1Report;
        			ReportData = USARTtoUSB_BufferStorage;
                                ReportSize = sizeof(USB_Joystick1Report_Data_t);
        			break;
        		case 1:
                                //ReportData = (uint8_t*)&joy2Report;
        			ReportData = USARTtoUSB_BufferStorage + sizeof(USB_Joystick1Report_Data_t);
                                ReportSize = sizeof(USB_Joystick2Report_Data_t);
        			break;
        		
        		default:
                                ReportData = USARTtoUSB_BufferStorage;
                                ReportSize = sizeof(USB_Joystick1Report_Data_t);
        			break;
        		}
                        /* Write the report data to the control endpoint */
                        Endpoint_Write_Control_Stream_LE(ReportData, ReportSize);
                        Endpoint_ClearOUT();
                }
                break;
	 case HID_REQ_SetIdle:
       		if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
       		{
       		        Endpoint_ClearSETUP();
       		        Endpoint_ClearStatusStage();

       		        /* Get idle period in MSB, must multiply by 4 to get the duration in milliseconds */
       		        IdleCount = ((USB_ControlRequest.wValue & 0xFF00) >> 6);
       		}
       		break;
 	case HID_REQ_GetIdle:
         	if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
         	{
         	        Endpoint_ClearSETUP();

         	        /* Write the current idle duration to the host, must be divided by 4 before sent to host */
         	        Endpoint_Write_8(IdleCount >> 2);

         	        Endpoint_ClearIN();
         	        Endpoint_ClearStatusStage();
         	}
         	break;
      }
}

void Joystick1_Task(void)
{
        /* Device must be connected and configured for the task to run */
        if (USB_DeviceState != DEVICE_STATE_Configured)
          return;

        /* Select the Mouse Report Endpoint */
        Endpoint_SelectEndpoint(ENDPOINT_DIR_IN | JOYSTICK1_EPNUM);

	bool IdleElapsed = false;
        /* Check if Mouse Endpoint Ready for Read/Write */
	if (IdleCount > 0 && IdleRemaining == 0 )
	{
		//The idle time has elapsed,reset the idleRemaining
		IdleRemaining = IdleCount;
		IdleElapsed = true;	
	
	}

        if (Endpoint_IsINReady() && ( memcmp( prevBufferData, USARTtoUSB_BufferStorage, sizeof(USB_Joystick1Report_Data_t)) != 0 || IdleElapsed ) ) 
        {
                /* Write Mouse Report Data */
                Endpoint_Write_Stream_LE( USARTtoUSB_BufferStorage, sizeof(USB_Joystick1Report_Data_t), NULL);

                /* Finalize the stream transfer to send the last packet */
                Endpoint_ClearIN();
		//save the current buffer data for comparing in next round
		memcpy( prevBufferData , USARTtoUSB_BufferStorage, sizeof ( USB_Joystick1Report_Data_t ) );
        }
}

void Joystick2_Task(void)
{
        /* Device must be connected and configured for the task to run */
        if (USB_DeviceState != DEVICE_STATE_Configured)
          return;


        /* Select the Joystick2 Report Endpoint */
        Endpoint_SelectEndpoint(ENDPOINT_DIR_IN | JOYSTICK2_EPNUM);

	bool IdleElapsed = false;
        /* Check if Mouse Endpoint Ready for Read/Write */
	if (IdleCount > 0 && IdleRemaining == 0 )
	{
		//The idle time has elapsed,reset the idleRemaining
		IdleRemaining = IdleCount;
		IdleElapsed = true;	
	
	}

        if (Endpoint_IsINReady() 
		&& (memcmp( prevBufferData + sizeof( USB_Joystick1Report_Data_t), USARTtoUSB_BufferStorage + sizeof(USB_Joystick1Report_Data_t),  sizeof ( USB_Joystick2Report_Data_t) ) != 0 || IdleElapsed ) 
	)
        {
                /* Write Joystick2 Report Data */
                Endpoint_Write_Stream_LE(USARTtoUSB_BufferStorage + sizeof ( USB_Joystick1Report_Data_t), sizeof(USB_Joystick2Report_Data_t), NULL);

                /* Finalize the stream transfer to send the last packet */
                Endpoint_ClearIN();
		//save the current buffer data for comparing in next round
		memcpy( prevBufferData +sizeof( USB_Joystick1Report_Data_t), USARTtoUSB_BufferStorage+sizeof( USB_Joystick1Report_Data_t), sizeof ( USB_Joystick2Report_Data_t ) );
        }
}



/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either REPORT_ITEM_TYPE_Out or REPORT_ITEM_TYPE_Feature
 *  \param[in] ReportData  Pointer to a buffer where the created report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
    /* Not used but must be present */
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
    uint8_t ReceivedByte = UDR1;

     /* we don't want to miss any received bytes from Serial Port, 
      *so just don't check the USB_DeviceState and don't check the RingBuffer is full or not, 
      *because RingBuffer will be overwrite from the begining and we don't care this situation.
      */
	RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}
