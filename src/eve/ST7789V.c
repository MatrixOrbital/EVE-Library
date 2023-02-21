// Bitbang SPI initialization for 2.4" display (ST7789V Controller)

#include "ST7789V.h"
#include "Eve2_81x.h"       // Header for this file with prototypes, defines, and typedefs
#include "MatrixEve2Conf.h" // Header for display selection
#include "hw_api.h"         // for spi abstraction
#include <stdbool.h>        // for true/false
#include <stdint.h>         // Find integer types like "uint8_t"
#include <stdio.h>

/********************************* SPI Bitbang
 * Implementation**************************************/

#define COMMAND 0
#define DATA 1
#define CS_ENABLE 0
#define CS_DISABLE 1

#define CS 0x02
#define SCL 0x04
#define SDA 0x08

void GPIOX_WriteBit(uint8_t data, bool state)
{
  if (state)
  {
    wr8(REG_GPIOX + RAM_REG, (rd8(REG_GPIOX + RAM_REG)) | data);
  }
  else
  {
    data = ~data;
    wr8(REG_GPIOX + RAM_REG, (rd8(REG_GPIOX + RAM_REG)) & (data));
  }
}

void MO_SPIBB_CS(uint8_t enable)
{
  wr16(REG_GPIOX_DIR + RAM_REG, 0x00f7); // set SDA GPIO0 as output

  uint32_t read;

  switch (enable)
  {
  case 0:
    wr8(REG_GPIOX + RAM_REG, (rd8(REG_GPIOX + RAM_REG)) & ~CS);
    break;
  case 1:
    wr8(REG_GPIOX + RAM_REG, (rd8(REG_GPIOX + RAM_REG)) | CS);
    break;
  }
}

void MO_SPIBB_Send(bool type, uint8_t data)
{
  unsigned char m = 0x80;
  uint8_t i, test;

  wr16(REG_GPIOX_DIR + RAM_REG, 0x80ff);
  wr16(REG_GPIOX + RAM_REG, 0x80f0);

  if (type == COMMAND)
  {

    GPIOX_WriteBit(SCL, 0);
    GPIOX_WriteBit(SDA, 0);
    GPIOX_WriteBit(SCL, 1);
  }
  else if (type == DATA)
  {

    GPIOX_WriteBit(SCL, 0);
    GPIOX_WriteBit(SDA, 1);
    GPIOX_WriteBit(SCL, 1);
  }

  for (i = 0; i < 8; i++)
  {
    GPIOX_WriteBit(SCL, 0);
    if (data & m)
    {
      GPIOX_WriteBit(SDA, 1);
    }
    else
    {
      GPIOX_WriteBit(SDA, 0);
    }
    GPIOX_WriteBit(SCL, 1);
    m = m >> 1;
  }
  GPIOX_WriteBit(SCL, 0);
}

#if 0
//Function which reads SPI (8-bit) data to ILI9341
void MO_SPIBB_Read(void)
{
	uint8_t i;
	uint32_t returnData = 0;
	uint32_t temp = 0;
	uint8_t Data = 0;
	wr16(REG_GPIOX_DIR + RAM_REG, 0xfff6);// set SDA GPIO0 as input
	HAL_Delay(10); 
	GPIOX_WriteBit(SCL, 1);
	HAL_Delay(10); 
	for (i = 0; i < 8; i++)
	{
		GPIOX_WriteBit(SCL, 0);
		HAL_Delay(100);
		GPIOX_WriteBit(SCL, 1);
		returnData = rd32(REG_GPIOX + RAM_REG);
		//printf("%d\n", returnData);
		Data |= (uint8_t)((returnData & 0x00000001) << (7-i));
		//printf("%d\n", returnData);
		HAL_Delay(100);
	}
	printf("%x\n", Data);	
	HAL_Delay(10); 
	GPIOX_WriteBit(SCL, 0);
	wr16(REG_GPIOX_DIR + RAM_REG, 0xfff7);// set SDA GPIO0 as output
}
#endif

void MO_ST7789V_init(void)
{
  printf("Start ST7789V SPI Initialization\n");

  wr16(REG_GPIOX_DIR + RAM_REG, (0x00FF));
  wr16(REG_GPIOX + RAM_REG, 0x00F7);

  uint32_t returnData;
  HAL_Delay(100); // 1000

  // the following is from AFY240320A0-2.8INTH data sheet, page 25
  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0x11);
  MO_SPIBB_CS(CS_DISABLE);
  HAL_Delay(120); // Delay 120ms

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0x36); // MADCTRL
  MO_SPIBB_Send(DATA, 0x00);    // was 0x80
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0x3a);
  MO_SPIBB_Send(DATA, 0x66);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xB0);
  MO_SPIBB_Send(DATA, 0x12); // <<-- RGB interface
  MO_SPIBB_Send(DATA, 0x00);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0x21);
  MO_SPIBB_CS(CS_DISABLE);
  /*
  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send( COMMAND, 0x2a);
  MO_SPIBB_Send( DATA, 0x00);
  MO_SPIBB_Send( DATA, 0x00);
  MO_SPIBB_Send( DATA, 0x00);
  MO_SPIBB_Send( DATA, 0xef);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send( COMMAND, 0x2b);
  MO_SPIBB_Send( DATA, 0x00);
  MO_SPIBB_Send( DATA, 0x00);
  MO_SPIBB_Send( DATA, 0x00);
  MO_SPIBB_Send( DATA, 0xef);
  MO_SPIBB_CS(CS_DISABLE);
  */
  //--------------------------------ST7789V Frame rate setting----------------------------------//
  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xb2);
  MO_SPIBB_Send(DATA, 0x0c);
  MO_SPIBB_Send(DATA, 0x0c);
  MO_SPIBB_Send(DATA, 0x00);
  MO_SPIBB_Send(DATA, 0x33);
  MO_SPIBB_Send(DATA, 0x33);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xb7);
  MO_SPIBB_Send(DATA, 0x35);
  MO_SPIBB_CS(CS_DISABLE);
  //---------------------------------ST7789V Power setting--------------------------------------//
  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xbb);
  MO_SPIBB_Send(DATA, 0x18); // 1F
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xc0);
  MO_SPIBB_Send(DATA, 0x2c);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xc2);
  MO_SPIBB_Send(DATA, 0x01);
  MO_SPIBB_Send(DATA, 0xFF);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xc3);
  MO_SPIBB_Send(DATA, 0x20); // 12
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xc4);
  MO_SPIBB_Send(DATA, 0x20);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xc6);
  MO_SPIBB_Send(DATA, 0x0f);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xd0);
  MO_SPIBB_Send(DATA, 0xa4);
  MO_SPIBB_Send(DATA, 0xa1);
  MO_SPIBB_CS(CS_DISABLE);
  //--------------------------------ST7789V gamma setting--------------------------------------//

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xe0);
  MO_SPIBB_Send(DATA, 0xd0);
  MO_SPIBB_Send(DATA, 0x08);
  MO_SPIBB_Send(DATA, 0x11);
  MO_SPIBB_Send(DATA, 0x08);
  MO_SPIBB_Send(DATA, 0x0c);
  MO_SPIBB_Send(DATA, 0x15);
  MO_SPIBB_Send(DATA, 0x39);
  MO_SPIBB_Send(DATA, 0x33);
  MO_SPIBB_Send(DATA, 0x50);
  MO_SPIBB_Send(DATA, 0x36);
  MO_SPIBB_Send(DATA, 0x13);
  MO_SPIBB_Send(DATA, 0x14);
  MO_SPIBB_Send(DATA, 0x29);
  MO_SPIBB_Send(DATA, 0x2d);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xe1);
  MO_SPIBB_Send(DATA, 0xd0);
  MO_SPIBB_Send(DATA, 0x08);
  MO_SPIBB_Send(DATA, 0x10);
  MO_SPIBB_Send(DATA, 0x08);
  MO_SPIBB_Send(DATA, 0x06);
  MO_SPIBB_Send(DATA, 0x06);
  MO_SPIBB_Send(DATA, 0x39);
  MO_SPIBB_Send(DATA, 0x44);
  MO_SPIBB_Send(DATA, 0x51);
  MO_SPIBB_Send(DATA, 0x0b);
  MO_SPIBB_Send(DATA, 0x16);
  MO_SPIBB_Send(DATA, 0x14);
  MO_SPIBB_Send(DATA, 0x2f);
  MO_SPIBB_Send(DATA, 0x31);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0x29);
  MO_SPIBB_CS(CS_DISABLE);

  printf("SPI Initialization Finished\n");
}
