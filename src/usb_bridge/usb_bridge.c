#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define FT800_PD_N 7

#include "ftd2xx.h"
#include "libmpsse_spi.h"

static FT_HANDLE handle;

FT_HANDLE GetFTDIHandle()
{
  return handle;
}

void HAL_Close(void)
{
  if (handle)
  {
    SPI_CloseChannel(handle);
    handle = NULL;
  }
}

void HAL_SPI_Enable(void)
{
  SPI_ToggleCS(handle, 1);
}

void HAL_SPI_Disable(void)
{
  SPI_ToggleCS(handle, 0);
}

uint8_t HAL_SPI_Write(uint8_t data)
{
  uint32_t SizeTransfered = 0;
  SPI_Write(handle, &data, 1, &SizeTransfered, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
  return 0;
}

uint8_t HAL_SPI_WriteByte(uint8_t data)
{
  uint32_t SizeTransfered;
  SPI_Write(handle, &data, 1, &SizeTransfered, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
  return 0;
}

uint8_t HAL_SPI_ReadByte(uint8_t data)
{
  uint8_t res;
  uint32_t SizeTransfered;
  SPI_Read(handle, &res, 1, &SizeTransfered, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
  return res;
}

void HAL_SPI_WriteBuffer(uint8_t *Buffer, uint32_t Length)
{
  uint32_t SizeTransfered;
  SPI_Write(handle, Buffer, Length, &SizeTransfered, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
}

void HAL_SPI_ReadBuffer(uint8_t *Buffer, uint32_t Length)
{
  HAL_SPI_WriteByte(0);
  uint32_t SizeTransfered;
  FT_STATUS res =
      SPI_Read(handle, Buffer, Length, &SizeTransfered, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
}

void HAL_Delay(uint32_t milliSeconds)
{
#ifdef _MSC_VER
  Sleep(milliSeconds);
#else
  usleep(milliSeconds * 1000);
#endif
}

int HAL_Eve_Reset_HW(void)
{
  uint32_t total_channels;
#ifndef _MSC_VER
  FT_SetVIDPID(0x1b3d, 0x0200);
#endif
  Init_libMPSSE();
  HAL_Close();
  FT_STATUS result = SPI_GetNumChannels(&total_channels);
  printf("channels found : %d\n", total_channels);
  const char *channel = getenv("SPICHANNEL");
  int ichan = 0;
  if (channel)
  {
    ichan = atoi(channel);
  }

  if (total_channels > ichan)
  {
    FT_DEVICE_LIST_INFO_NODE devList;
    SPI_GetChannelInfo(ichan, &devList);

    ChannelConfig channelConf; // channel configuration
    FT_STATUS status;
    /* configure the spi settings */
    channelConf.ClockRate = 12 * 1000 * 1000;
    channelConf.LatencyTimer = 2;
    channelConf.configOptions =
        SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
    channelConf.Pin = 0x00000000;

    /* Open the first available channel */
    SPI_OpenChannel(ichan, (FT_HANDLE *)&handle);
    status = SPI_InitChannel(handle, &channelConf);
    if (status == FT_OK)
    {
      printf("USB->SPI Bridge opened\n");
    }
    else
    {
      printf("Unable to open USB->SPI Bridge\n");
      return 0;
    }
  }
  else
  {
    printf("USB->SPI Bridge not found.");
    return 0;
  }

  // reset the EVE by toggling PD pin (GPIO 7 of the FT232H) 0 to 1
  FT_WriteGPIO(handle, (1 << FT800_PD_N) | 0x3B, (0 << FT800_PD_N) | 0x08); // PDN set to 0
  HAL_Delay(20);

  FT_WriteGPIO(handle, (1 << FT800_PD_N) | 0x3B, (1 << FT800_PD_N) | 0x08); // PDN set to 1
  HAL_Delay(20);
  return 1;
}
