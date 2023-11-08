/*

Copyright (c) Future Technology Devices International 2014

THIS SOFTWARE IS PROVIDED BY FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

FTDI DRIVERS MAY BE USED ONLY IN CONJUNCTION WITH PRODUCTS BASED ON FTDI PARTS.

FTDI DRIVERS MAY BE DISTRIBUTED IN ANY FORM AS LONG AS LICENSE INFORMATION IS NOT MODIFIED.

IF A CUSTOM VENDOR ID AND/OR PRODUCT ID OR DESCRIPTION STRING ARE USED, IT IS THE
RESPONSIBILITY OF THE PRODUCT MANUFACTURER TO MAINTAIN ANY CHANGES AND SUBSEQUENT WHQL
RE-CERTIFICATION AS A RESULT OF MAKING THESE CHANGES.

Author : FTDI 

Project: libMPSSE
Module: SPI

Revision History:
0.1 - initial version
0.2 - 2011.07.08 - added FT_ReadGPIO, FT_WriteGPIO & SPI_ChangeCS
0.3 - 2011.10.25 - modified for supporting 64bit linux

*/
 
 

#ifndef LIBMPSSE_SPI_H
#define LIBMPSSE_SPI_H

#include "ftd2xx.h"


/******************************************************************************/
/*								Macro defines								  */
/******************************************************************************/

#ifdef _MSC_VER
#define FTDI_API 
#else
#define FTDI_API
#endif

/* Bit defination of the transferOptions parameter in SPI_Read, SPI_Write & SPI_Transfer  */

/* transferOptions-Bit0: If this bit is 0 then it means that the transfer size provided is in bytes */
#define	SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES			0x00000000
/* transferOptions-Bit0: If this bit is 1 then it means that the transfer size provided is in bytes */
#define	SPI_TRANSFER_OPTIONS_SIZE_IN_BITS			0x00000001
/* transferOptions-Bit1: if BIT1 is 1 then CHIP_SELECT line will be enables at start of transfer */
#define	SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE		0x00000002
/* transferOptions-Bit2: if BIT2 is 1 then CHIP_SELECT line will be disabled at end of transfer */
#define SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE		0x00000004




/* Bit defination of the Options member of configOptions structure*/
#define SPI_CONFIG_OPTION_MODE_MASK		0x00000003
#define SPI_CONFIG_OPTION_MODE0			0x00000000
#define SPI_CONFIG_OPTION_MODE1			0x00000001
#define SPI_CONFIG_OPTION_MODE2			0x00000002
#define SPI_CONFIG_OPTION_MODE3			0x00000003

#define SPI_CONFIG_OPTION_CS_MASK			0x0000001C		/*111 00*/
#define SPI_CONFIG_OPTION_CS_DBUS3		0x00000000		/*000 00*/
#define SPI_CONFIG_OPTION_CS_DBUS4		0x00000004		/*001 00*/
#define SPI_CONFIG_OPTION_CS_DBUS5		0x00000008		/*010 00*/
#define SPI_CONFIG_OPTION_CS_DBUS6		0x0000000C		/*011 00*/
#define SPI_CONFIG_OPTION_CS_DBUS7		0x00000010		/*100 00*/

#define SPI_CONFIG_OPTION_CS_ACTIVELOW	0x00000020


/******************************************************************************/
/*								Type defines								  */
/******************************************************************************/
#include <stdint.h>

typedef enum I2C_ClockRate_t{
	I2C_CLOCK_STANDARD_MODE = 100000,							// 100kb/sec
	I2C_CLOCK_FAST_MODE = 400000,								// 400kb/sec
	I2C_CLOCK_FAST_MODE_PLUS = 1000000, 						// 1000kb/sec
	I2C_CLOCK_HIGH_SPEED_MODE = 3400000 					    // 3.4Mb/sec
}I2C_CLOCKRATE;

typedef struct ChannelConfig_t
{
	uint32_t	ClockRate; 

	uint8_t 	LatencyTimer; 

	uint32_t	configOptions;	/*This member provides a way to enable/disable features
	specific to the protocol that are implemented in the chip
	BIT1-0=CPOL-CPHA:	00 - MODE0 - data captured on rising edge, propagated on falling
 						01 - MODE1 - data captured on falling edge, propagated on rising
 						10 - MODE2 - data captured on falling edge, propagated on rising
 						11 - MODE3 - data captured on rising edge, propagated on falling
	BIT4-BIT2: 000 - A/B/C/D_DBUS3=ChipSelect
			 : 001 - A/B/C/D_DBUS4=ChipSelect
 			 : 010 - A/B/C/D_DBUS5=ChipSelect
 			 : 011 - A/B/C/D_DBUS6=ChipSelect
 			 : 100 - A/B/C/D_DBUS7=ChipSelect
 	BIT5: ChipSelect is active high if this bit is 0
	BIT6 -BIT31		: Reserved
	*/

	uint32_t 	Pin;/*BIT7   -BIT0:   Initial direction of the pins	*/
					/*BIT15 -BIT8:   Initial values of the pins		*/
					/*BIT23 -BIT16: Final direction of the pins		*/
					/*BIT31 -BIT24: Final values of the pins		*/
	uint16_t	reserved;
}ChannelConfig;


/******************************************************************************/
/*								External variables							  */
/******************************************************************************/





/******************************************************************************/
/*								Function declarations						  */
/******************************************************************************/
FTDI_API FT_STATUS SPI_GetNumChannels(uint32_t *numChannels);
FTDI_API FT_STATUS SPI_GetChannelInfo(uint32_t index, 
	FT_DEVICE_LIST_INFO_NODE *chanInfo);
FTDI_API FT_STATUS SPI_OpenChannel(uint32_t index, FT_HANDLE *handle);
FTDI_API FT_STATUS SPI_InitChannel(FT_HANDLE handle, ChannelConfig *config);
FTDI_API FT_STATUS SPI_CloseChannel(FT_HANDLE handle);
FTDI_API FT_STATUS SPI_ToggleCS(FT_HANDLE handle,uint8_t high);
FTDI_API FT_STATUS SPI_Read(FT_HANDLE handle, uint8_t *buffer, 
	uint32_t sizeToTransfer, uint32_t *sizeTransfered, uint32_t options);
FTDI_API FT_STATUS SPI_Write(FT_HANDLE handle, uint8_t *buffer, 
	uint32_t sizeToTransfer, uint32_t *sizeTransfered, uint32_t options);
FTDI_API FT_STATUS SPI_ReadWrite(FT_HANDLE handle, uint8_t *inBuffer, 
	uint8_t *outBuffer, uint32_t sizeToTransfer, uint32_t *sizeTransferred, 
	uint32_t transferOptions);
FTDI_API FT_STATUS SPI_IsBusy(FT_HANDLE handle, bool *state);
FTDI_API void Init_libMPSSE(void);
FTDI_API void Cleanup_libMPSSE(void);
FTDI_API FT_STATUS SPI_ChangeCS(FT_HANDLE handle, uint32_t configOptions);
FTDI_API FT_STATUS FT_WriteGPIO(FT_HANDLE handle, uint8_t dir, uint8_t value);
FTDI_API FT_STATUS FT_ReadGPIO(FT_HANDLE handle,uint8_t *value);



/******************************************************************************/


#endif	/*LIBMPSSE_SPI_H*/

