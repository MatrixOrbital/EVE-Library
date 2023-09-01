/* Based on example by bjorn vaktaren at
 * https://gist.github.com/bjornvaktaren/d2461738ec44e3ad8b3bae4ce69445b4 */
#define WITH_FLUSH
#include <ftdi.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define BUS_SK 0x01 // ADBUS0, SPI data clock
#define BUS_DO 0x02 // ADBUS1, SPI data out
#define BUS_DI 0x04 // ADBUS2, SPI data in
#define BUS_CS 0x08 // ADBUS3, SPI chip select
#define BUS_L0 0x10 // ADBUS4, general-ourpose i/o, GPIOL0
#define BUS_L1 0x20 // ADBUS5, general-ourpose i/o, GPIOL1
#define BUS_L2 0x40 // ADBUS6, general-ourpose i/o, GPIOL2
#define BUS_L3 0x80 // ADBUS7, general-ourpose i/o, GPIOL3

#define FT800_RST BUS_L3
// Set these pins high
#define pinInitialState (BUS_CS | BUS_L0 | BUS_L1 | FT800_RST)
#define pinDirection (BUS_SK | BUS_DO | BUS_CS | BUS_L0 | BUS_L1 | FT800_RST)

struct ftdi_context *ftdi;

void HAL_Close(void)
{
  printf("Closing bridge\n");
  HAL_Delay(200);
#ifdef WITH_FLUSH
  ftdi_tcioflush(ftdi);
#endif
  if (ftdi)
  {
    ftdi_usb_close(ftdi);
  }
}

void HAL_RST_Enable(void)
{
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = SET_BITS_LOW;
  buf[icmd++] = pinInitialState & ~FT800_RST;
  buf[icmd++] = pinDirection;
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Enable write failed\n");
  }
}

void HAL_RST_Disable(void)
{
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = SET_BITS_LOW;
  buf[icmd++] = pinInitialState | FT800_RST;
  buf[icmd++] = pinDirection;
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Enable write failed\n");
  }
}

void HAL_SPI_Enable(void)
{
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = SET_BITS_LOW;
  buf[icmd++] = pinInitialState & ~BUS_CS;
  buf[icmd++] = pinDirection;
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Enable write failed\n");
  }
}

void HAL_SPI_Disable(void)
{
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = SET_BITS_LOW;
  buf[icmd++] = pinInitialState | BUS_CS;
  buf[icmd++] = pinDirection;
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Enable write failed\n");
  }
}

uint8_t HAL_SPI_Write(uint8_t data)
{
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
  buf[icmd++] = 0x00; // length low byte, 0x0000 ==> 1 byte
  buf[icmd++] = 0x00; // length high byte
  buf[icmd++] = data; // byte to send
#ifdef WITH_FLUSH
  ftdi_tcioflush(ftdi);
#endif
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Write failed\n");
  }
  return 0;
}

void HAL_SPI_WriteBuffer(uint8_t *Buffer, uint32_t Length)
{
  int icmd = 0;
  uint8_t *buf = malloc(Length + 16);
  buf[icmd++] = MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
  buf[icmd++] = (Length - 1) & 0xff;
  buf[icmd++] = ((Length - 1) >> 8) & 0xff; // length high byte
  memcpy(&buf[icmd], Buffer, Length);
  icmd += Length;
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Write failed\n");
  }
  free(buf);
}

void HAL_SPI_ReadBuffer(uint8_t *Buffer, uint32_t Length)
{
  HAL_SPI_Write(0);
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = MPSSE_WRITE_NEG | MPSSE_DO_READ;
  buf[icmd++] = (Length - 1) & 0xff;
  buf[icmd++] = ((Length - 1) >> 8) & 0xff; // length high byte
  buf[icmd++] = SEND_IMMEDIATE;
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Write failed\n");
  }
  uint8_t res;
  ftdi_read_data(ftdi, Buffer, Length);
}

void HAL_Delay(uint32_t milliSeconds)
{
  usleep(milliSeconds * 1000);
}

void HAL_Eve_Reset_HW(void)
{
  ftdi = ftdi_new();
  if (!ftdi)
  {
    printf("Failed to initialize USB bridge\n");
    exit(1);
  }

  int ftdi_status = ftdi_usb_open(ftdi, 0x1b3d, 0x200);
  if (ftdi_status != 0)
  {
    printf("Can't open USB bridge, error %s\n", ftdi_get_error_string(ftdi));
    exit(1);
  }
  printf("Bridge opened successfully!\n");
  ftdi_usb_reset(ftdi);
  ftdi_set_interface(ftdi, INTERFACE_ANY);
  ftdi_set_bitmode(ftdi, 0, 0);
  ftdi_set_bitmode(ftdi, 0, BITMODE_MPSSE);
  ftdi_tcioflush(ftdi);
  usleep(100000);

  unsigned int icmd = 0;
  unsigned char buf[256] = {0};
  buf[icmd++] = TCK_DIVISOR;     // opcode: set clk divisor
  buf[icmd++] = 0x05;            // argument: low bit. 6 MHz / (5+1) = 1 MHz
  buf[icmd++] = 0x00;            // argument: high bit.
  buf[icmd++] = DIS_ADAPTIVE;    // opcode: disable adaptive clocking
  buf[icmd++] = DIS_3_PHASE;     // opcode: disable 3-phase clocking
  buf[icmd++] = SET_BITS_LOW;    // opcode: set low bits (ADBUS[0-7])
  buf[icmd++] = pinInitialState; // argument: inital pin states
  buf[icmd++] = pinDirection;    // argument: pin direction
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("Bridge setup failed\n");
    ftdi_usb_close(ftdi);
    exit(1);
  }
  printf("Setup complete!\n");
  HAL_RST_Enable();
  HAL_Delay(20);
  HAL_RST_Disable();
  HAL_Delay(20);
}
