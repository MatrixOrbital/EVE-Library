// EVE Processor Agnostic Library (Condensed)
//
// This library is for the FT812, FT813, BT815, BT816, BT817, BT818
//
// This "library" consists of the files "Eve2_81x.c" and "Eve2_81x.h".
//
// In persuit of the common goal of simplicity and understandability I find that I am unable to
// make function prototypes that match Bridgetek example code.  I draw the line between the
// EVE and all other hardware. The library is "clean" and includes no abstraction at all, unlike
// much of the example code on the Internet which is sort of application and abstraction mixed
// together in a confusing abuse of my eye-holes.
// My intent is to be as straight forward and understandable as possible, so while function
// names and parameter lists are different than Bridgetek code examples, they should be easily
// recognizable.  I have also made every attempt to reference Bridgetek documentation against
// the code to act as a translation to help in understanding.
//
// Notes on the operation of the EVE command processing engine - THE FIFO
//
// First be aware that the FTDI/Bridgetek documentation variously refers to you as "User", "MCU",
// "Host".
//
// The FIFO, like all FIFO's needs pointers to indicate the starting address of buffered data and
// the end address of buffered data.  There is wrapping involved, but the basic idea is clear.
// EVE takes data into it's FIFO using a fully defined write operation to a memory address - that
// is, you need to take care of the wrapping - to you, it is not a FIFO - it is a piece of memory.
// EVE keeps track of it's own read address location, but relies on you to write the address
// of the end of buffered data.
//
// So as commands are loaded into RAM - into the FIFO space - EVE will do nothing in response.
// EVE is happy to take your data and store it for you while it sits with it's read address and
// write address set to the same value.  Once the commands are loaded, the next available address
// is manually written (by you) to the register in which Eve stores the FIFO write pointer
// (REG_CMD_WRITE).
//
// Following this, EVE discovers that the addresses are different and begins processing commands
// while updating it's own read pointer until the read and write pointers are the same.
//
// Be aware that EVE stores only the offset into the "FIFO" as 16 bits, so any use of the offset
// requires adding the base address (RAM_CMD 0x308000) to the resultant 32 bit value.

#include "Eve2_81x.h"       // Header for this file with prototypes, defines, and typedefs
#include "MatrixEve2Conf.h" // Header for display selection
#include "ST7789V.h"        // For 2.4" and 2.8" displays
#include "hw_api.h"         // For SPI abstraction
#include <stdbool.h>        // For true/false
#include <stdint.h>         // Find integer types like "uint8_t"
#include <stdio.h>

#define WorkBuffSz 512
#define Log printf

// Global Variables
uint16_t FifoWriteLocation = 0;
char LogBuf[WorkBuffSz]; // The singular universal data array used for all things including logging

const uint8_t Touch70I_WG[] = {
    26,  255, 255, 255, 32,  32,  48,  0,   4,   0,   0,   0,   2,   0,   0,   0,   26,  255, 255,
    255, 0,   176, 48,  0,   4,   0,   0,   0,   82,  3,   0,   0,   34,  255, 255, 255, 0,   176,
    48,  0,   120, 218, 181, 83,  79,  104, 156, 85,  16,  159, 183, 111, 179, 154, 68,  214, 239,
    43,  165, 20,  201, 7,   251, 109, 54,  46,  237, 182, 80,  76,  68,  4,   133, 121, 73,  91,
    218, 132, 80,  180, 7,   115, 16,  250, 222, 183, 113, 255, 125, 43,  33,  120, 17,  137, 118,
    170, 32,  30,  62,  248, 240, 210, 122, 48,  197, 131, 171, 72,  192, 147, 65,  165, 7,   5,
    107, 144, 98,  201, 65,  40,  69,  8,   120, 40,  165, 167, 230, 146, 34,  168, 176, 206, 124,
    187, 74,  241, 36,  98,  120, 204, 155, 121, 243, 222, 252, 121, 51,  191, 121, 81,  3,   0,
    37,  13,  69,  105, 84,  117, 158, 11,  226, 15,  29,  80,  218, 244, 220, 186, 243, 153, 170,
    46,  177, 85,  150, 68,  95,  117, 34,  137, 28,  196, 242, 118, 240, 62,  136, 255, 146, 18,
    171, 155, 73,  99,  12,  147, 6,   123, 104, 121, 46,  109, 233, 86,  210, 96,  190, 2,   157,
    164, 49,  141, 105, 84,  68,  221, 122, 174, 164, 221, 243, 144, 218, 196, 22,  241, 56,  62,
    11,  190, 77,  35,  142, 222, 134, 14,  231, 209, 172, 146, 34,  207, 77,  219, 148, 179, 152,
    7,   191, 235, 185, 60,  38,  221, 2,   142, 211, 2,   46,  192, 57,  148, 88,  58,  242, 156,
    31,  103, 59,  71,  213, 43,  79,  179, 190, 128, 85,  182, 0,   10,  98,  223, 190, 94,  170,
    88,  69,  186, 190, 90,  58,  132, 75,  56,  129, 197, 108, 15,  226, 57,  108, 131, 118, 19,
    168, 93,  234, 138, 120, 144, 234, 40,  63,  58,  192, 92,  126, 177, 132, 135, 232, 37,  212,
    182, 140, 79,  12,  111, 14,  255, 125, 51,  193, 55,  169, 149, 204, 70,  9,   56,  39,  221,
    78,  235, 143, 177, 148, 54,  129, 214, 17,  26,  31,  149, 18,  11,  52,  70,  126, 247, 211,
    146, 228, 238, 219, 47,  74,  248, 110, 15,  79,  225, 231, 252, 91,  232, 108, 64,  234, 122,
    216, 131, 99,  116, 22,  106, 180, 136, 138, 142, 210, 180, 213, 238, 40,  91,  140, 225, 247,
    165, 30,  126, 203, 47,  20,  213, 72,  172, 107, 116, 22,  175, 101, 239, 32,  123, 55,  208,
    108, 195, 54,  123, 184, 129, 155, 248, 53,  211, 77,  248, 73,  246, 225, 233, 6,   108, 97,
    15,  175, 161, 248, 187, 199, 17,  107, 180, 137, 208, 189, 197, 22,  181, 44,  142, 220, 109,
    226, 45,  184, 72,  121, 250, 128, 251, 253, 43,  26,  212, 203, 183, 113, 155, 181, 190, 125,
    132, 116, 123, 151, 121, 97,  200, 71,  134, 60,  63,  228, 122, 200, 115, 67,  174, 50,  190,
    139, 119, 48,  12,  1,   18,  107, 88,  74,  163, 59,  88,  54,  15,  45,  165, 232, 119, 156,
    50,  64,  187, 144, 195, 93,  152, 50,  39,  212, 140, 41,  255, 99,  157, 80,  83,  230, 41,
    163, 221, 14,  148, 233, 60,  148, 152, 42,  76,  33,  247, 250, 164, 41,  226, 105, 51,  201,
    167, 28,  230, 176, 223, 135, 134, 111, 251, 253, 196, 114, 191, 35,  193, 164, 104, 214, 237,
    203, 166, 128, 121, 122, 6,   151, 204, 73,  117, 90,  157, 83,  103, 148, 216, 78,  209, 2,
    140, 144, 111, 161, 115, 193, 156, 194, 87,  195, 196, 102, 127, 90,  89,  11,  165, 131, 122,
    85,  47,  191, 169, 82,  39,  218, 26,  190, 29,  78,  195, 64,  42,  96,  197, 190, 101, 94,
    97,  116, 4,   241, 69,  245, 36,  251, 240, 237, 28,  234, 200, 183, 151, 195, 30,  66,  231,
    125, 182, 57,  198, 218, 227, 148, 116, 71,  153, 10,  76,  208, 16,  156, 113, 77,  149, 33,
    65,  64,  145, 174, 194, 39,  230, 203, 112, 13,  5,   177, 208, 245, 221, 107, 166, 234, 156,
    9,   98,  193, 239, 119, 74,  116, 121, 198, 86,  17,  215, 240, 7,   154, 31,  234, 37,  151,
    143, 205, 31,  160, 184, 66,  51,  230, 81,  210, 171, 105, 4,   116, 221, 108, 153, 29,  70,
    202, 117, 37,  178, 231, 182, 140, 50,  59,  48,  78,  139, 28,  225, 126, 40,  158, 128, 231,
    204, 99,  212, 64,  253, 110, 232, 187, 11,  102, 14,  101, 90,  70,  8,   154, 191, 132, 111,
    224, 30,  219, 141, 211, 85,  220, 11,  95,  48,  243, 230, 12,  72,  197, 244, 242, 17,  142,
    247, 56,  219, 142, 225, 111, 225, 17,  70,  152, 200, 80,  230, 122, 242, 63,  30,  40,  184,
    244, 192, 112, 223, 103, 57,  135, 118, 62,  167, 248, 52,  154, 203, 101, 59,  208, 21,  115,
    153, 54,  56,  159, 43,  102, 239, 210, 6,   220, 54,  50,  109, 178, 180, 245, 232, 71,  206,
    27,  168, 138, 139, 147, 131, 252, 103, 140, 94,  46,  240, 204, 105, 59,  99,  22,  232, 29,
    35,  8,   26,  72,  227, 44,  233, 168, 156, 221, 104, 119, 128, 107, 17,  196, 89,  102, 245,
    165, 178, 47,  211, 200, 154, 138, 149, 169, 235, 127, 35,  149, 213, 237, 32,  254, 204, 136,
    134, 43,  29,  137, 213, 127, 245, 255, 222, 62,  251, 255, 106, 159, 253, 223, 219, 103, 255,
    135, 39,  255, 127, 255, 21,  251, 112, 132, 32,  230, 41,  108, 233, 250, 226, 191, 138, 132,
    108, 47,  235, 103, 117, 112, 118, 98,  86,  49,  246, 206, 235, 63,  1,   55,  97,  247, 70,
    0,   0,   26,  255, 255, 255, 32,  32,  48,  0,   4,   0,   0,   0,   0,   0,   0,   0};

static uint32_t Width;
static uint32_t Height;
static uint32_t HOffset;
static uint32_t VOffset;
static uint8_t Touch;

uint32_t Display_Width()
{
  return Width;
}

uint32_t Display_Height()
{
  return Height;
}

uint8_t Display_Touch()
{
  return Touch;
}

uint32_t Display_HOffset()
{
  return HOffset;
}
uint32_t Display_VOffset()
{
  return VOffset;
}

// Call this function once at powerup to reset and initialize the EVE chip
int FT81x_Init(int display, int board, int touch)
{
  uint32_t Ready = false;
  int DWIDTH;
  int DHEIGHT;
  int PIXVOFFSET;
  int PIXHOFFSET;
  int HCYCLE;
  int HOFFSET;
  int HSYNC0;
  int HSYNC1;
  int VCYCLE;
  int VOFFSET;
  int VSYNC0;
  int VSYNC1;
  int PCLK;
  int SWIZZLE;
  int PCLK_POL;
  int HSIZE;
  int VSIZE;
  int CSPREAD;
  int DITHER;

  switch (display)
  {
  case DISPLAY_70:
    DWIDTH = 800;
    DHEIGHT = 480;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 928;
    HOFFSET = 88;
    HSYNC0 = 0;
    HSYNC1 = 48;
    VCYCLE = 525;
    VOFFSET = 32;
    VSYNC0 = 0;
    VSYNC1 = 3;
    PCLK = 2;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 800;
    VSIZE = 480;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_50:
    DWIDTH = 800;
    DHEIGHT = 480;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 928;
    HOFFSET = 88;
    HSYNC0 = 0;
    HSYNC1 = 48;
    VCYCLE = 525;
    VOFFSET = 32;
    VSYNC0 = 0;
    VSYNC1 = 3;
    PCLK = 2;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 800;
    VSIZE = 480;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_43:
    DWIDTH = 480;
    DHEIGHT = 272;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 548;
    HOFFSET = 43;
    HSYNC0 = 0;
    HSYNC1 = 41;
    VCYCLE = 292;
    VOFFSET = 12;
    VSYNC0 = 0;
    VSYNC1 = 10;
    PCLK = 5;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 480;
    VSIZE = 272;
    CSPREAD = 1;
    DITHER = 1;
    break;
  case DISPLAY_39:
    DWIDTH = 480;
    DHEIGHT = 128;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 524;
    HOFFSET = 17;
    HSYNC1 = 27;
    HSYNC0 = 0;
    VCYCLE = 288;
    VOFFSET = 12;
    VSYNC1 = 8;
    VSYNC0 = 7;
    PCLK = 5;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 480;
    VSIZE = 272;
    CSPREAD = 1;
    DITHER = 1;
    break;
  case DISPLAY_38:
    DWIDTH = 480;
    DHEIGHT = 116;
    PIXVOFFSET = 10;
    PIXHOFFSET = 0;
    HCYCLE = 524;
    HOFFSET = 43;
    HSYNC0 = 0;
    HSYNC1 = 41;
    VCYCLE = 292;
    VOFFSET = 12;
    VSYNC0 = 152;
    VSYNC1 = 10;
    PCLK = 5;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 480;
    VSIZE = 272;
    CSPREAD = 1;
    DITHER = 1;
    break;
  case DISPLAY_35:
    DWIDTH = 320;
    DHEIGHT = 240;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 408;
    HOFFSET = 68;
    HSYNC0 = 0;
    HSYNC1 = 10;
    VCYCLE = 262;
    VOFFSET = 18;
    VSYNC0 = 0;
    VSYNC1 = 2;
    PCLK = 8;
    SWIZZLE = 0;
    PCLK_POL = 0;
    HSIZE = 320;
    VSIZE = 240;
    CSPREAD = 1;
    DITHER = 1;
    break;
  case DISPLAY_29:
    DWIDTH = 320;
    DHEIGHT = 102;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 408;
    HOFFSET = 70;
    HSYNC0 = 0;
    HSYNC1 = 10;
    VCYCLE = 262;
    VOFFSET = 156;
    VSYNC0 = 0;
    VSYNC1 = 2;
    PCLK = 8;
    SWIZZLE = 0;
    PCLK_POL = 0;
    HSIZE = 320;
    VSIZE = 102;
    CSPREAD = 1;
    DITHER = 1;
    break;
  case DISPLAY_40:
    DWIDTH = 720;
    DHEIGHT = 720;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 812;
    HOFFSET = 91;
    HSYNC0 = 46;
    HSYNC1 = 48;
    VCYCLE = 756;
    VOFFSET = 35;
    VSYNC0 = 16;
    VSYNC1 = 18;
    PCLK = 2;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 720;
    VSIZE = 720;
    CSPREAD = 0;
    DITHER = 0;
    break;
  case DISPLAY_101:
    DWIDTH = 1280;
    DHEIGHT = 800;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 1440;
    HOFFSET = 158;
    HSYNC0 = 78;
    HSYNC1 = 80;
    VCYCLE = 823;
    VOFFSET = 22;
    VSYNC0 = 11;
    VSYNC1 = 12;
    PCLK = 1;
    SWIZZLE = 0;
    PCLK_POL = 0;
    HSIZE = 1280;
    VSIZE = 800;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_70I:
  case DISPLAY_70I_WG:
    DWIDTH = 1024;
    DHEIGHT = 600;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 1344;
    HOFFSET = 319;
    HSYNC0 = 12;
    HSYNC1 = 230;
    VCYCLE = 635;
    VOFFSET = 34;
    VSYNC0 = 12;
    VSYNC1 = 22;
    PCLK = 1;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 1024;
    VSIZE = 600;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_24:
    DWIDTH = 240;
    DHEIGHT = 320;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 298;
    HOFFSET = 57;
    HSYNC0 = 38;
    HSYNC1 = 48;
    VCYCLE = 336;
    VOFFSET = 15;
    VSYNC0 = 8;
    VSYNC1 = 8;
    PCLK = 6;
    SWIZZLE = 0;
    PCLK_POL = 0;
    HSIZE = 240;
    VSIZE = 320;
    CSPREAD = 1;
    DITHER = 1;
    break;
  default:
    printf("Unknown display type\n");
    return 0;
    break;
  }
  Width = DWIDTH;
  Height = DHEIGHT;
  HOffset = PIXHOFFSET;
  VOffset = PIXVOFFSET;
  Touch = touch;
  Eve_Reset(); // Hard reset of the EVE chip

  // Wakeup EVE
  if (board >= BOARD_EVE3)
  {
    HostCommand(HCMD_CLKEXT);
  }
  HostCommand(HCMD_ACTIVE);
  HAL_Delay(300);

  do
  {
    Ready = Cmd_READ_REG_ID();
  } while (!Ready);
  do
  {
    Ready = rd16(REG_CPU_RESET);
  } while (!Ready);

  //  Log("EVE now ACTIVE\n");         //

  Ready = rd32(REG_CHIP_ID);
  uint16_t ValH = Ready >> 16;
  uint16_t ValL = Ready & 0xFFFF;
  Log("Chip ID = 0x%04x%04x\n", ValH, ValL);

  if (display == DISPLAY_101)
  {
    wr32(REG_FREQUENCY + RAM_REG, 80000000); // Configure the system clock to 80MHz
  }
  else
  {
    wr32(REG_FREQUENCY + RAM_REG, 60000000); // Configure the system clock to 60MHz
  }
  // Before we go any further with EVE, it is a good idea to check to see if the EVE is wigging out
  // about something that happened before the last reset.  If EVE has just done a power cycle, this
  // would be unnecessary.
  if (rd16(REG_CMD_READ + RAM_REG) == 0xFFF)
  {
    // EVE is unhappy - needs a paddling.
    uint32_t Patch_Add = rd32(REG_COPRO_PATCH_PTR + RAM_REG);
    wr8(REG_CPU_RESET + RAM_REG, 1);
    wr16(REG_CMD_READ + RAM_REG, 0);
    wr16(REG_CMD_WRITE + RAM_REG, 0);
    wr16(REG_CMD_DL + RAM_REG, 0);
    wr8(REG_CPU_RESET + RAM_REG, 0);
    wr32(REG_COPRO_PATCH_PTR + RAM_REG, Patch_Add);
  }

  // Turn off screen output during startup
  wr16(REG_GPIOX + RAM_REG,
       rd16(REG_GPIOX + RAM_REG) &
           ~(1 << 15));       // Set REG_GPIOX bit 15 to 0 to turn off the LCD DISP signal
  wr8(REG_PCLK + RAM_REG, 0); // Pixel Clock Output disable

  if (display == DISPLAY_24)
    MO_ST7789V_init();

  // Load parameters of the physical screen to the EVE
  // All of these registers are 32 bits, but most bits are reserved, so only write what is actually
  // used
  wr16(REG_HCYCLE + RAM_REG, HCYCLE);    // Set H_Cycle to 548
  wr16(REG_HOFFSET + RAM_REG, HOFFSET);  // Set H_Offset to 43
  wr16(REG_HSYNC0 + RAM_REG, HSYNC0);    // Set H_SYNC_0 to 0
  wr16(REG_HSYNC1 + RAM_REG, HSYNC1);    // Set H_SYNC_1 to 41
  wr16(REG_VCYCLE + RAM_REG, VCYCLE);    // Set V_Cycle to 292
  wr16(REG_VOFFSET + RAM_REG, VOFFSET);  // Set V_OFFSET to 12
  wr16(REG_VSYNC0 + RAM_REG, VSYNC0);    // Set V_SYNC_0 to 0
  wr16(REG_VSYNC1 + RAM_REG, VSYNC1);    // Set V_SYNC_1 to 10
  wr8(REG_SWIZZLE + RAM_REG, SWIZZLE);   // Set SWIZZLE to 0
  wr8(REG_PCLK_POL + RAM_REG, PCLK_POL); // Set PCLK_POL to 1
  wr16(REG_HSIZE + RAM_REG, HSIZE);      // Set H_SIZE to 480
  wr16(REG_VSIZE + RAM_REG, VSIZE);      // Set V_SIZE to 272
  wr8(REG_CSPREAD + RAM_REG, CSPREAD); // Set CSPREAD to 1    (32 bit register - write only 8 bits)
  wr8(REG_DITHER + RAM_REG, DITHER);   // Set DITHER to 1     (32 bit register - write only 8 bits)

  /* Reset the touch engine, since it has sometimes issues starting up. */
  wr32(RAM_REG + REG_CPU_RESET, 1 << 1);
  HAL_Delay(10);
  wr32(RAM_REG + REG_CPU_RESET, 0);
  HAL_Delay(10);
  // Configure touch & audio
  if (touch == TOUCH_TPR)
  {
    wr16(REG_TOUCH_CONFIG + RAM_REG, 0x8381);
  }
  else if (touch == TOUCH_TPC)
  {
    if (display == DISPLAY_40)
      wr16(REG_TOUCH_CONFIG + RAM_REG, 0x480); // FT6336U touch controller
    else
      wr16(REG_TOUCH_CONFIG + RAM_REG, 0x5d0);
    if (board == BOARD_EVE2)
    {
      Cap_Touch_Upload();
    }
    if (board == BOARD_EVE4 && DISPLAY_70I_WG)
    {
      UploadTouchFirmware(Touch70I_WG, sizeof(Touch70I_WG));
    }
  }

  wr16(REG_TOUCH_RZTHRESH + RAM_REG, 1200); // Set touch resistance threshold
  wr8(REG_TOUCH_MODE + RAM_REG, 0x02);      // Set touch on: continous - this is default
  wr8(REG_TOUCH_ADC_MODE + RAM_REG, 0x01);  // Set ADC mode: differential - this is default
  wr8(REG_TOUCH_OVERSAMPLE + RAM_REG, 15);  // Set touch oversampling to max

  // wr16(REG_GPIOX_DIR + RAM_REG, 0x8000 | (1<<3));   // Set Disp GPIO Direction
  // wr16(REG_GPIOX + RAM_REG, 0x8000 | (1<<3));       // Enable Disp (if used)

  wr16(REG_GPIOX_DIR + RAM_REG, 0xffff); // Make GPIOs output
  if (display == DISPLAY_101)
    wr16(REG_GPIOX + RAM_REG,
         0x80f7); // Motor (GPIO 3, active high) is off, speaker (GPIO 2) is on
  else
    wr16(REG_GPIOX + RAM_REG, 0x80ff); // Motor (GPIO 3, active low) is off, speaker (GPIO 2) is on

  wr16(REG_PWM_HZ + RAM_REG, 0x00FA); // Backlight PWM frequency
  wr8(REG_PWM_DUTY + RAM_REG, 128);   // Backlight PWM duty (on)

  // write first display list (which is a clear and blank screen)
  wr32(RAM_DL + 0, CLEAR_COLOR_RGB(0, 0, 0));
  wr32(RAM_DL + 4, CLEAR(1, 1, 1));
  wr32(RAM_DL + 8, DISPLAY());
  wr8(REG_DLSWAP + RAM_REG, DLSWAP_FRAME); // Swap display lists
  wr8(REG_PCLK + RAM_REG, PCLK);           // After this display is visible on the TFT
  return 1;
}

// Reset EVE chip via the hardware PDN line
void Eve_Reset(void)
{
  HAL_Eve_Reset_HW();
}

// Upload Goodix Calibration file, ex GT911
void Cap_Touch_Upload(void)
{
// This makes the Arduino uno run out of space so sadly
// we cannot support this.
#if !defined(__AVR__)
#include "touch_cap_811.h"
  //---Goodix911 Configuration from AN336
  // Load the TOUCH_DATA_U8 or TOUCH_DATA_U32 array from file “touch_cap_811.h” via the FT81x
  // command buffer RAM_CMD
  uint8_t CTOUCH_CONFIG_DATA_G911[] = {TOUCH_DATA_U8};
  CoProWrCmdBuf(CTOUCH_CONFIG_DATA_G911, TOUCH_DATA_LEN);
  // Execute the commands till completion
  UpdateFIFO();
  Wait4CoProFIFOEmpty();
  // Hold the touch engine in reset(write REG_CPURESET = 2)
  wr8(REG_CPU_RESET + RAM_REG, 2);
  // Set GPIO3 output LOW
  wr8(REG_GPIOX_DIR + RAM_REG, (rd8(RAM_REG + REG_GPIOX_DIR) | 0x08)); // Set Disp GPIO Direction
  wr8(REG_GPIOX + RAM_REG, (rd8(RAM_REG + REG_GPIOX) | 0xF7));         // Clear GPIO
  // Wait more than 100us
  HAL_Delay(1);
  // Write REG_CPURESET=0
  wr8(REG_CPU_RESET + RAM_REG, 0);
  // Wait more than 55ms
  HAL_Delay(100);
  // Set GPIO3 to input (floating)
  wr8(REG_GPIOX_DIR + RAM_REG, (rd8(RAM_REG + REG_GPIOX_DIR) & 0xF7)); // Set Disp GPIO Direction
#endif
}

// *** Host Command - FT81X Embedded Video Engine Datasheet - 4.1.5
// ********************************************** Host Command is a function for changing hardware
// related parameters of the Eve chip.  The name is confusing. These are related to power modes and
// the like.  All defined parameters have HCMD_ prefix
void HostCommand(uint8_t HCMD)
{
  //  Log("Inside HostCommand\n");

  HAL_SPI_Enable();

  /*  HAL_SPI_Write(HCMD | 0x40); // In case the manual is making you believe that you just found
   * the bug you were looking for - no. */
  HAL_SPI_Write(HCMD);
  HAL_SPI_Write(0x00); // This second byte is set to 0 but if there is need for fancy, never used
                       // setups, then rewrite.
  HAL_SPI_Write(0x00);

  HAL_SPI_Disable();
}

// *** EVE API Reference Definitions
// ***************************************************************************** FT81X Embedded
// Video Engine Datasheet 1.3 - Section 4.1.4, page 16 These are all functions related to writing /
// reading data of various lengths with a memory address of 32 bits
// ***************************************************************************************************************
void wr32(uint32_t address, uint32_t parameter)
{
  HAL_SPI_Enable();
  uint8_t buffer[16];
  int idx = 0;

  buffer[idx++] =
      ((address >> 16) | 0x80); // RAM_REG = 0x302000 and high bit is set - result always 0xB0
  buffer[idx++] = (uint8_t)(address >> 8); // Next byte of the register address
  buffer[idx++] =
      (uint8_t)address; // Low byte of register address - usually just the 1 byte offset

  buffer[idx++] = (uint8_t)(parameter & 0xff); // Little endian (yes, it is most significant bit
                                               // first and least significant byte first)
  buffer[idx++] = (uint8_t)((parameter >> 8) & 0xff);
  buffer[idx++] = (uint8_t)((parameter >> 16) & 0xff);
  buffer[idx++] = (uint8_t)((parameter >> 24) & 0xff);
  HAL_SPI_WriteBuffer(buffer, idx);

  HAL_SPI_Disable();
}

void wr16(uint32_t address, uint16_t parameter)
{
  HAL_SPI_Enable();

  HAL_SPI_Write((uint8_t)((address >> 16) |
                          0x80)); // RAM_REG = 0x302000 and high bit is set - result always 0xB0
  HAL_SPI_Write((uint8_t)(address >> 8)); // Next byte of the register address
  HAL_SPI_Write((uint8_t)address); // Low byte of register address - usually just the 1 byte offset

  HAL_SPI_Write((uint8_t)(parameter & 0xff)); // Little endian (yes, it is most significant bit
                                              // first and least significant byte first)
  HAL_SPI_Write((uint8_t)(parameter >> 8));

  HAL_SPI_Disable();
}

void wr8(uint32_t address, uint8_t parameter)
{
  HAL_SPI_Enable();

  HAL_SPI_Write((uint8_t)((address >> 16) |
                          0x80)); // RAM_REG = 0x302000 and high bit is set - result always 0xB0
  HAL_SPI_Write((uint8_t)(address >> 8)); // Next byte of the register address
  HAL_SPI_Write(
      (uint8_t)(address)); // Low byte of register address - usually just the 1 byte offset

  HAL_SPI_Write(parameter);

  HAL_SPI_Disable();
}

uint32_t rd32(uint32_t address)
{
  uint8_t buf[4];
  int idx = 0;
  uint32_t Data32;

  HAL_SPI_Enable();

  buf[idx++] = (address >> 16) & 0x3F;
  buf[idx++] = (address >> 8) & 0xff;
  buf[idx++] = address & 0xff;
  HAL_SPI_WriteBuffer(buf, idx);
  HAL_SPI_ReadBuffer(buf, 4);

  HAL_SPI_Disable();

  Data32 = buf[0] + ((uint32_t)buf[1] << 8) + ((uint32_t)buf[2] << 16) + ((uint32_t)buf[3] << 24);
  return (Data32);
}
uint16_t rd16(uint32_t address)
{
  uint8_t buf[2] = {0, 0};

  HAL_SPI_Enable();

  HAL_SPI_Write((address >> 16) & 0x3F);
  HAL_SPI_Write((address >> 8) & 0xff);
  HAL_SPI_Write(address & 0xff);

  HAL_SPI_ReadBuffer(buf, 2);

  HAL_SPI_Disable();

  uint16_t Data16 = buf[0] + ((uint16_t)buf[1] << 8);
  return (Data16);
}

uint8_t rd8(uint32_t address)
{
  uint8_t buf[1];

  HAL_SPI_Enable();

  HAL_SPI_Write((address >> 16) & 0x3F);
  HAL_SPI_Write((address >> 8) & 0xff);
  HAL_SPI_Write(address & 0xff);

  HAL_SPI_ReadBuffer(buf, 1);

  HAL_SPI_Disable();

  return (buf[0]);
}

void rdN(uint32_t address, uint8_t *buffer, uint32_t size)
{

  HAL_SPI_Enable();

  HAL_SPI_Write((address >> 16) & 0x3F);
  HAL_SPI_Write((address >> 8) & 0xff);
  HAL_SPI_Write(address & 0xff);

  HAL_SPI_ReadBuffer(buffer, size);

  HAL_SPI_Disable();
}

// *** Send_Cmd() - this is like cmd() in (some) EVE docs - sends 32 bits but does not update the
// write pointer *** FT81x Series Programmers Guide Section 5.1.1 - Circular Buffer (AKA "the FIFO"
// and "Command buffer" and "Coprocessor") Don't miss section 5.3 - Interaction with RAM_DL
void Send_CMD(uint32_t data)
{
  wr32(FifoWriteLocation + RAM_CMD,
       data); // Write the command at the globally tracked "write pointer" for the FIFO

  FifoWriteLocation +=
      FT_CMD_SIZE; // Increment the Write Address by the size of a command - which we just sent
  FifoWriteLocation %= FT_CMD_FIFO_SIZE; // Wrap the address to the FIFO space
}

// UpdateFIFO - Cause the coprocessor to realize that it has work to do in the form of a
// differential between the read pointer and write pointer.  The coprocessor (FIFO or "Command
// buffer") does nothing until you tell it that the write position in the FIFO RAM has changed
void UpdateFIFO(void)
{
  wr16(REG_CMD_WRITE + RAM_REG,
       FifoWriteLocation); // We manually update the write position pointer
}

// Read the specific ID register and return TRUE if it is the expected 0x7C otherwise.
uint8_t Cmd_READ_REG_ID(void)
{
  uint8_t readData[2];

  HAL_SPI_Enable();
  HAL_SPI_Write(0x30); // Base address RAM_REG = 0x302000
  HAL_SPI_Write(0x20);
  HAL_SPI_Write(REG_ID);           // REG_ID offset = 0x00
  HAL_SPI_ReadBuffer(readData, 1); // There was a dummy read of the first byte in there
  HAL_SPI_Disable();

  if (readData[0] == 0x7C) // FT81x Datasheet section 5.1, Table 5-2. Return value always 0x7C
  {
    //    Log("\nGood ID: 0x%02x\n", readData[0]);
    return 1;
  }
  else
  {
    //    Log("0x%02x ", readData[0]);
    return 0;
  }
}

// **************************************** Coprocessor/GPU/FIFO/Command buffer Command Functions
// *************** These are discussed in FT81x Series Programmers Guide, starting around
// section 5.10 While display list commands can be sent to the coprocessor, these listed commands
// are specific to it.  They are mostly widgets like graphs, but also touch related functions like
// cmd_track() and memory operations. Essentially, these commands set up parameters for CoPro
// functions which expand "macros" using those parameters to then write a series of commands into
// the Display List to create all the primitives which make that widget.
// ***************************************************************************************************************

// ******************** Screen Object Creation Coprocessor Command Functions
// ******************************

// *** Draw Slider - FT81x Series Programmers Guide Section 5.38
// *************************************************
void Cmd_Slider(
    uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t options, uint16_t val, uint16_t range)
{
  Send_CMD(CMD_SLIDER);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)h << 16) | w);
  Send_CMD(((uint32_t)val << 16) | options);
  Send_CMD((uint32_t)range);
}

// *** Draw Spinner - FT81x Series Programmers Guide Section 5.54
// *************************************************
void Cmd_Spinner(uint16_t x, uint16_t y, uint16_t style, uint16_t scale)
{
  Send_CMD(CMD_SPINNER);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)scale << 16) | style);
}

// *** Draw Gauge - FT81x Series Programmers Guide Section 5.33
// **************************************************
void Cmd_Gauge(uint16_t x,
               uint16_t y,
               uint16_t r,
               uint16_t options,
               uint16_t major,
               uint16_t minor,
               uint16_t val,
               uint16_t range)
{
  Send_CMD(CMD_GAUGE);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)options << 16) | r);
  Send_CMD(((uint32_t)minor << 16) | major);
  Send_CMD(((uint32_t)range << 16) | val);
}

// *** Draw Dial - FT81x Series Programmers Guide Section 5.39
// ************************************************** This is much like a Gauge except for the
// helpful range parameter.  For some reason, all dials are 65535 around.
void Cmd_Dial(uint16_t x, uint16_t y, uint16_t r, uint16_t options, uint16_t val)
{
  Send_CMD(CMD_DIAL);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)options << 16) | r);
  Send_CMD((uint32_t)val);
}

// *** Make Track (for a slider) - FT81x Series Programmers Guide Section 5.62
// ************************************ Tag refers to the tag # previously assigned to the object
// that this track is tracking.
void Cmd_Track(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t tag)
{
  Send_CMD(CMD_TRACK);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)h << 16) | w);
  Send_CMD((uint32_t)tag);
}

// *** Draw Number - FT81x Series Programmers Guide Section 5.43
// *************************************************
void Cmd_Number(uint16_t x, uint16_t y, uint16_t font, uint16_t options, uint32_t num)
{
  Send_CMD(CMD_NUMBER);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)options << 16) | font);
  Send_CMD(num);
}

// *** Draw Smooth Color Gradient - FT81x Series Programmers Guide Section 5.34
// **********************************
void Cmd_Gradient(uint16_t x0, uint16_t y0, uint32_t rgb0, uint16_t x1, uint16_t y1, uint32_t rgb1)
{
  Send_CMD(CMD_GRADIENT);
  Send_CMD(((uint32_t)y0 << 16) | x0);
  Send_CMD(rgb0);
  Send_CMD(((uint32_t)y1 << 16) | x1);
  Send_CMD(rgb1);
}

// *** Draw Button - FT81x Series Programmers Guide Section 5.28
// **************************************************
void Cmd_Button(uint16_t x,
                uint16_t y,
                uint16_t w,
                uint16_t h,
                uint16_t font,
                uint16_t options,
                const char *str)
{
  uint16_t DataPtr, LoopCount, StrPtr;

  uint16_t length = (uint16_t)strlen(str);
  if (!length)
    return;

  uint32_t *data = (uint32_t *)calloc((length / 4) + 1, sizeof(uint32_t));

  StrPtr = 0;
  for (DataPtr = 0; DataPtr < (length / 4); DataPtr++, StrPtr += 4)
    data[DataPtr] = (uint32_t)str[StrPtr + 3] << 24 | (uint32_t)str[StrPtr + 2] << 16 |
                    (uint32_t)str[StrPtr + 1] << 8 | (uint32_t)str[StrPtr];

  for (LoopCount = 0; LoopCount < (length % 4); LoopCount++, StrPtr++)
    data[DataPtr] |= (uint32_t)str[StrPtr] << (LoopCount * 8);

  Send_CMD(CMD_BUTTON);
  Send_CMD(((uint32_t)y << 16) |
           x); // Put two 16 bit values together into one 32 bit value - do it little endian
  Send_CMD(((uint32_t)h << 16) | w);
  Send_CMD(((uint32_t)options << 16) | font);

  for (LoopCount = 0; LoopCount <= length / 4; LoopCount++)
  {
    Send_CMD(data[LoopCount]);
  }

  free(data);
}

// *** Draw Text - FT81x Series Programmers Guide Section 5.41
// ***************************************************
void Cmd_Text(uint16_t x, uint16_t y, uint16_t font, uint16_t options, const char *str)
{
  uint16_t DataPtr, LoopCount, StrPtr;

  uint16_t length = (uint16_t)strlen(str);
  if (!length)
    return;

  uint32_t *data = (uint32_t *)calloc(
      (length / 4) + 1, sizeof(uint32_t)); // Allocate memory for the string expansion

  StrPtr = 0;
  for (DataPtr = 0; DataPtr < (length / 4); ++DataPtr, StrPtr = StrPtr + 4)
    data[DataPtr] = (uint32_t)str[StrPtr + 3] << 24 | (uint32_t)str[StrPtr + 2] << 16 |
                    (uint32_t)str[StrPtr + 1] << 8 | (uint32_t)str[StrPtr];

  for (LoopCount = 0; LoopCount < (length % 4); ++LoopCount, ++StrPtr)
    data[DataPtr] |= (uint32_t)str[StrPtr] << (LoopCount * 8);

  // Set up the command
  Send_CMD(CMD_TEXT);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)options << 16) | font);

  // Send out the text
  for (LoopCount = 0; LoopCount <= length / 4; LoopCount++)
    Send_CMD(data[LoopCount]); // These text bytes get sucked up 4 at a time and fired at the FIFO

  free(data);
}

// ******************** Miscellaneous Operation Coprocessor Command Functions
// ******************************
void Cmd_SetFont2(uint32_t handle, uint32_t addr, uint32_t firstChar)
{
  Send_CMD(CMD_SETFONT2);
  Send_CMD(handle);
  Send_CMD(addr);
  Send_CMD(firstChar);
}
// *** Cmd_SetBitmap - generate DL commands for bitmap parms - FT81x Series Programmers Guide
// Section 5.65 *******
void Cmd_SetBitmap(uint32_t addr, uint16_t fmt, uint16_t width, uint16_t height)
{
  Send_CMD(CMD_SETBITMAP);
  Send_CMD(addr);
  uint32_t val = width;
  val = val << 16;
  val = val | fmt;
  Send_CMD(val);
  Send_CMD((uint32_t)height);
}

// *** Cmd_Memcpy - background copy a block of data - FT81x Series Programmers Guide Section 5.27
// ****************
void Cmd_Memcpy(uint32_t dest, uint32_t src, uint32_t num)
{
  Send_CMD(CMD_MEMCPY);
  Send_CMD(dest);
  Send_CMD(src);
  Send_CMD(num);
}

// *** Cmd_GetPtr - Get the last used address from CoPro operation - FT81x Series Programmers Guide
// Section 5.47 *
void Cmd_GetPtr(void)
{
  Send_CMD(CMD_GETPTR);
  Send_CMD(0);
}

// *** Set Highlight Gradient Color - FT81x Series Programmers Guide Section 5.32
// ********************************
void Cmd_GradientColor(uint32_t c)
{
  Send_CMD(CMD_GRADCOLOR);
  Send_CMD(c);
}

// *** Set Foreground color - FT81x Series Programmers Guide Section 5.30
// ************************************************
void Cmd_FGcolor(uint32_t c)
{
  Send_CMD(CMD_FGCOLOR);
  Send_CMD(c);
}

// *** Set Background color - FT81x Series Programmers Guide Section 5.31
// ************************************************
void Cmd_BGcolor(uint32_t c)
{
  Send_CMD(CMD_BGCOLOR);
  Send_CMD(c);
}

// *** Translate Matrix - FT81x Series Programmers Guide Section 5.51
// ********************************************
void Cmd_Translate(uint32_t tx, uint32_t ty)
{
  Send_CMD(CMD_TRANSLATE);
  Send_CMD(tx);
  Send_CMD(ty);
}

// *** Rotate Matrix - FT81x Series Programmers Guide Section 5.50
// ***********************************************
void Cmd_Rotate(uint32_t a)
{
  Send_CMD(CMD_ROTATE);
  Send_CMD(a);
}

// *** Rotate Screen - FT81x Series Programmers Guide Section 5.53
// ***********************************************
void Cmd_SetRotate(uint32_t rotation)
{
  Send_CMD(CMD_SETROTATE);
  Send_CMD(rotation);
}

// *** Scale Matrix - FT81x Series Programmers Guide Section 5.49
// ************************************************
void Cmd_Scale(uint32_t sx, uint32_t sy)
{
  Send_CMD(CMD_SCALE);
  Send_CMD(sx);
  Send_CMD(sy);
}

// *** Flash Fast - FT81x Series Programmers Guide Section x.xx
// ************************************************
void Cmd_Flash_Fast(void)
{
  Send_CMD(CMD_FLASHFAST);
  Send_CMD(0);
}

// *** Calibrate Touch Digitizer - FT81x Series Programmers Guide Section 5.52
// ***********************************
// * This business about "result" in the manual really seems to be simply leftover cruft of no
// purpose - send zero
void Cmd_Calibrate(uint32_t result)
{
  Send_CMD(CMD_CALIBRATE);
  Send_CMD(result);
}

// An interactive calibration screen is created and executed.
// New calibration values are written to the touch matrix registers of Eve.
void Calibrate_Manual(uint16_t Width, uint16_t Height, uint16_t V_Offset, uint16_t H_Offset)
{
  uint32_t displayX[3], displayY[3];
  uint32_t touchX[3], touchY[3];
  uint32_t touchValue = 0, storedValue = 0;
  int32_t tmp, k;
  int32_t TransMatrix[6];
  uint8_t count = 0;
  uint8_t pressed = 0;
  char num[2];

  // These values determine where your calibration points will be drawn on your display
  displayX[0] = (uint32_t)(Width * 0.15) + H_Offset;
  displayY[0] = (uint32_t)(Height * 0.15) + V_Offset;

  displayX[1] = (uint32_t)(Width * 0.85) + H_Offset;
  displayY[1] = (uint32_t)(Height / 2) + V_Offset;

  displayX[2] = (uint32_t)(Width / 2) + H_Offset;
  displayY[2] = (uint32_t)(Height * 0.85) + V_Offset;

  while (count < 3)
  {
    Send_CMD(CMD_DLSTART);
    Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    Send_CMD(CLEAR(1, 1, 1));

    // Draw Calibration Point on screen
    Send_CMD(COLOR_RGB(255, 0, 0));
    Send_CMD(POINT_SIZE(20 * 16));
    Send_CMD(BEGIN(POINTS));
    Send_CMD(VERTEX2F((uint32_t)(displayX[count]) * 16, (uint32_t)((displayY[count])) * 16));
    Send_CMD(END());
    Send_CMD(COLOR_RGB(255, 255, 255));
    Cmd_Text((Width / 2) + H_Offset, (Height / 3) + V_Offset, 27, OPT_CENTER, "Calibrating");
    Cmd_Text(
        (Width / 2) + H_Offset, (Height / 2) + V_Offset, 27, OPT_CENTER, "Please tap the dots");
    num[0] = count + 0x31;
    num[1] = 0; // null terminated string of one character
    Cmd_Text(displayX[count], displayY[count], 27, OPT_CENTER, num);

    Send_CMD(DISPLAY());
    Send_CMD(CMD_SWAP);
    UpdateFIFO();          // Trigger the coprocessor to start processing commands out of the FIFO
    Wait4CoProFIFOEmpty(); // Wait here until the coprocessor has read and executed every pending
                           // command.
    HAL_Delay(300);

    while (pressed == count)
    {
      touchValue = rd32(REG_TOUCH_DIRECT_XY + RAM_REG); // Read for any new touch tag inputs
      if (!(touchValue & 0x80000000))
      {
        touchX[count] = (touchValue >> 16) & 0x03FF; // Raw Touchscreen Y coordinate
        touchY[count] = touchValue & 0x03FF;         // Raw Touchscreen Y coordinate

        // Log("\ndisplay x[%d]: %ld display y[%d]: %ld\n", count, displayX[count], count,
        // displayY[count]); Log("touch x[%d]: %ld touch y[%d]: %ld\n", count, touchX[count], count,
        // touchY[count]);

        count++;
      }
    }
    pressed = count;
  }

  k = ((touchX[0] - touchX[2]) * (touchY[1] - touchY[2])) -
      ((touchX[1] - touchX[2]) * (touchY[0] - touchY[2]));

  tmp = (((displayX[0] - displayX[2]) * (touchY[1] - touchY[2])) -
         ((displayX[1] - displayX[2]) * (touchY[0] - touchY[2])));
  TransMatrix[0] = ((int64_t)tmp << 16) / k;

  tmp = (((touchX[0] - touchX[2]) * (displayX[1] - displayX[2])) -
         ((displayX[0] - displayX[2]) * (touchX[1] - touchX[2])));
  TransMatrix[1] = ((int64_t)tmp << 16) / k;

  tmp = ((touchY[0] * (((touchX[2] * displayX[1]) - (touchX[1] * displayX[2])))) +
         (touchY[1] * (((touchX[0] * displayX[2]) - (touchX[2] * displayX[0])))) +
         (touchY[2] * (((touchX[1] * displayX[0]) - (touchX[0] * displayX[1])))));
  TransMatrix[2] = ((int64_t)tmp << 16) / k;

  tmp = (((displayY[0] - displayY[2]) * (touchY[1] - touchY[2])) -
         ((displayY[1] - displayY[2]) * (touchY[0] - touchY[2])));
  TransMatrix[3] = ((int64_t)tmp << 16) / k;

  tmp = (((touchX[0] - touchX[2]) * (displayY[1] - displayY[2])) -
         ((displayY[0] - displayY[2]) * (touchX[1] - touchX[2])));
  TransMatrix[4] = ((int64_t)tmp << 16) / k;

  tmp = ((touchY[0] * (((touchX[2] * displayY[1]) - (touchX[1] * displayY[2])))) +
         (touchY[1] * (((touchX[0] * displayY[2]) - (touchX[2] * displayY[0])))) +
         (touchY[2] * (((touchX[1] * displayY[0]) - (touchX[0] * displayY[1])))));
  TransMatrix[5] = ((int64_t)tmp << 16) / k;

  count = 0;
  do
  {
    wr32(REG_TOUCH_TRANSFORM_A + RAM_REG + (count * 4),
         TransMatrix[count]); // Write to Eve config registers

    //    uint16_t ValH = TransMatrix[count] >> 16;
    //    uint16_t ValL = TransMatrix[count] & 0xFFFF;
    //    Log("TM%d: 0x%04x %04x\n", count, ValH, ValL);

    count++;
  } while (count < 6);
}
// ***************************************************************************************************************
// *** Animation functions
// ***************************************************************************************
// ***************************************************************************************************************

void Cmd_AnimStart(int32_t ch, uint32_t aoptr, uint32_t loop)
{
  Send_CMD(CMD_ANIMSTART);
  Send_CMD(ch);
  Send_CMD(aoptr);
  Send_CMD(loop);
}

void Cmd_AnimStop(int32_t ch)
{
  Send_CMD(CMD_ANIMSTOP);
  Send_CMD(ch);
}

void Cmd_AnimXY(int32_t ch, int16_t x, int16_t y)
{
  Send_CMD(CMD_ANIMXY);
  Send_CMD(ch);
  Send_CMD(((uint32_t)y << 16) | x);
}

void Cmd_AnimDraw(int32_t ch)
{
  Send_CMD(CMD_ANIMDRAW);
  Send_CMD(ch);
}

void Cmd_AnimDrawFrame(int16_t x, int16_t y, uint32_t aoptr, uint32_t frame)
{
  Send_CMD(CMD_ANIMFRAME);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(aoptr);
  Send_CMD(frame);
}

// ***************************************************************************************************************
// *** Utility and helper functions
// ******************************************************************************
// ***************************************************************************************************************

// Find the space available in the GPU AKA coprocessor AKA command buffer AKA FIFO
uint16_t CoProFIFO_FreeSpace(void)
{
  uint16_t cmdBufferDiff, cmdBufferRd, cmdBufferWr, retval;

  cmdBufferRd = rd16(REG_CMD_READ + RAM_REG);
  cmdBufferWr = rd16(REG_CMD_WRITE + RAM_REG);

  cmdBufferDiff = (cmdBufferWr - cmdBufferRd) % FT_CMD_FIFO_SIZE; // FT81x Programmers Guide 5.1.1
  retval = (FT_CMD_FIFO_SIZE - 4) - cmdBufferDiff;
  return (retval);
}

// Sit and wait until there are the specified number of bytes free in the <GPU/Coprocessor>
// incoming FIFO
void Wait4CoProFIFO(uint32_t room)
{
  uint16_t getfreespace;

  do
  {
    getfreespace = CoProFIFO_FreeSpace();
  } while (getfreespace < room);
}

// Sit and wait until the CoPro FIFO is empty
// Detect operational errors and print the error and stop.
void Wait4CoProFIFOEmpty(void)
{
  uint16_t ReadReg;
  uint8_t ErrChar;
  uint8_t buffy[2];
  do
  {
    ReadReg = rd16(REG_CMD_READ + RAM_REG);
    if (ReadReg == 0xFFF)
    {
      // This is a error which would require sophistication to fix and continue but we fake it
      // somewhat unsuccessfully
      Log("\n");
      uint8_t Offset = 0;
      do
      {
        // Get the error character and display it
        ErrChar = rd8(RAM_ERR_REPORT + Offset);
        Offset++;
        sprintf(buffy, "%c", ErrChar);
        Log(buffy);
      } while ((ErrChar != 0) &&
               (Offset < 128)); // When the last stuffed character was null, we are done
      Log("\n");

      // EVE is unhappy - needs a paddling.
      uint32_t Patch_Add = rd32(REG_COPRO_PATCH_PTR + RAM_REG);
      wr8(REG_CPU_RESET + RAM_REG, 1);
      wr8(REG_CMD_READ + RAM_REG, 0);
      wr8(REG_CMD_WRITE + RAM_REG, 0);
      wr8(REG_CMD_DL + RAM_REG, 0);
      wr8(REG_CPU_RESET + RAM_REG, 0);
      wr32(REG_COPRO_PATCH_PTR + RAM_REG, Patch_Add);
      HAL_Delay(250); // We already saw one error message and we don't need to see then 1000 times
                      // a second
    }
  } while (ReadReg != rd16(REG_CMD_WRITE + RAM_REG));
}

// Every CoPro transaction starts with enabling the SPI and sending an address
void StartCoProTransfer(uint32_t address, uint8_t reading)
{
  HAL_SPI_Enable();
  if (reading)
  {
    HAL_SPI_Write(address >> 16);
    HAL_SPI_Write(address >> 8);
    HAL_SPI_Write(address);
    HAL_SPI_Write(0);
  }
  else
  {
    HAL_SPI_Write((address >> 16) | 0x80);
    HAL_SPI_Write(address >> 8);
    HAL_SPI_Write(address);
  }
}

// *** CoProWrCmdBuf() - Transfer a buffer into the CoPro FIFO as part of an ongoing command
// operation ***********
void CoProWrCmdBuf(const uint8_t *buff, uint32_t count)
{
  uint32_t TransferSize = 0;
  int32_t Remaining = count; // Signed

  do
  {
    // Here is the situation:  You have up to about a megabyte of data to transfer into the FIFO
    // Your buffer is LogBuf - limited to 64 bytes (or some other value, but always limited).
    // You need to go around in loops taking 64 bytes at a time until all the data is gone.
    //
    // Most interactions with the FIFO are started and finished in one operation in an obvious
    // fashion, but here it is important to understand the difference between EVE RAM registers and
    // Eve FIFO.  Even though you are in the middle of a FIFO operation and filling the FIFO is an
    // ongoing task, you are still free to write and read non-FIFO registers on the EVE chip.
    //
    // Since the FIFO is 4K in size, but the RAM_G space is 1M in size, you can not, obviously,
    // send all the possible RAM_G data through the FIFO in one step.  Also, since the EVE is not
    // capable of updating it's own FIFO pointer as data is written, you will need to
    // intermittently tell EVE to go process some FIFO in order to make room in the FIFO for more
    // RAM_G data.

    Wait4CoProFIFO(
        WorkBuffSz); // It is reasonable to wait for a small space instead of firing data piecemeal

    if (Remaining > WorkBuffSz)  // Remaining data exceeds the size of our buffer
      TransferSize = WorkBuffSz; // So set the transfer size to that of our buffer
    else
    {
      TransferSize = Remaining;                  // Set size to this last dribble of data
      TransferSize = (TransferSize + 3) & 0xFFC; // 4 byte alignment
    }

    StartCoProTransfer(FifoWriteLocation + RAM_CMD,
                       false); // Base address of the Command Buffer plus our offset into it -
                               // Start SPI transaction

    HAL_SPI_WriteBuffer((uint8_t *)buff,
                        TransferSize); // Write the little bit for which we found space
    buff += TransferSize;              // Move the working data read pointer to the next fresh data

    FifoWriteLocation = (FifoWriteLocation + TransferSize) % FT_CMD_FIFO_SIZE;
    HAL_SPI_Disable(); // End SPI transaction with the FIFO

    wr16(REG_CMD_WRITE + RAM_REG, FifoWriteLocation); // Manually update the write position pointer
                                                      // to initiate processing of the FIFO
    Remaining -= TransferSize;                        // reduce what we want by what we sent

  } while (Remaining > 0); // Keep going as long as we still want more
}

// Write a block of data into EVE RAM space a byte at a time.
// Return the last written address + 1 (The next available RAM address)
uint32_t WriteBlockRAM(uint32_t Add, const uint8_t *buff, uint32_t count)
{
  uint32_t index;
  uint32_t WriteAddress =
      Add; // I want to return the value instead of modifying the variable in place

  for (index = 0; index < count; index++)
  {
    wr8(WriteAddress++, buff[index]);
  }
  return (WriteAddress);
}

// CalcCoef - Support function for manual screen calibration function
int32_t CalcCoef(int32_t Q, int32_t K)
{
  int8_t sn = 0;

  if (Q < 0) // We need to work with positive values
  {
    Q *= -1; // So here we make them positive
    sn++;    // and remember that fact
  }

  if (K < 0)
  {
    K *= -1;
    sn++; // 1 + 1 = 2 = 0b00000010
  }

  uint32_t I = ((uint32_t)Q / (uint32_t)K) << 16; // Get the integer part and shift it by 16
  uint32_t R = Q % K;                             // Get the remainder of a/k;
  R = R << 14;                                    // Shift by 14
  R = R / K;                                      // Divide
  R = R << 2;                                     // Make up for the missing bits
  int32_t returnValue = I + R;                    // Combine them

  if (sn & 0x01)       // If the result is supposed to be negative
    returnValue *= -1; // then return it to that state.

  return (returnValue);
}

bool FlashAttach(void)
{
  Send_CMD(CMD_FLASHATTACH);
  UpdateFIFO();          // Trigger the coprocessor to start processing commands out of the FIFO
  Wait4CoProFIFOEmpty(); // Wait here until the coprocessor has read and executed every pending
                         // command.

  uint8_t FlashStatus = rd8(REG_FLASH_STATUS + RAM_REG);
  if (FlashStatus != FLASH_STATUS_BASIC)
  {
    return false;
  }
  return true;
}

bool FlashDetach(void)
{
  Send_CMD(CMD_FLASHDETACH);
  UpdateFIFO();          // Trigger the coprocessor to start processing commands out of the FIFO
  Wait4CoProFIFOEmpty(); // Wait here until the coprocessor has read and executed every pending
                         // command.

  uint8_t FlashStatus = rd8(REG_FLASH_STATUS + RAM_REG);
  if (FlashStatus != FLASH_STATUS_DETACHED)
  {
    return false;
  }
  return true;
}

bool FlashFast(void)
{
  Cmd_Flash_Fast();
  UpdateFIFO();          // Trigger the coprocessor to start processing commands out of the FIFO
  Wait4CoProFIFOEmpty(); // Wait here until the coprocessor has read and executed every pending
                         // command.

  uint8_t FlashStatus = rd8(REG_FLASH_STATUS + RAM_REG);
  if (FlashStatus != FLASH_STATUS_FULL)
  {
    return false;
  }
  return true;
}

bool FlashErase(void)
{
  Send_CMD(CMD_FLASHERASE);
  UpdateFIFO();          // Trigger the coprocessor to start processing commands out of the FIFO
  Wait4CoProFIFOEmpty(); // Wait here until the coprocessor has read and executed every pending
                         // command.
  return true;
}

void UploadTouchFirmware(const uint8_t *firmware, size_t length)
{
  CoProWrCmdBuf(firmware, length);
  UpdateFIFO();
  Wait4CoProFIFOEmpty();
  wr8(REG_CPU_RESET + RAM_REG, 2);
  wr8(REG_GPIOX_DIR + RAM_REG, (rd8(RAM_REG + REG_GPIOX_DIR) | 0x08)); // Set Disp GPIO Direction
  wr8(REG_GPIOX + RAM_REG, (rd8(RAM_REG + REG_GPIOX) | 0xF7));         // Clear GPIO
  HAL_Delay(1);
  wr8(REG_CPU_RESET + RAM_REG, 0);
  HAL_Delay(100);
  wr8(REG_GPIOX_DIR + RAM_REG, (rd8(RAM_REG + REG_GPIOX_DIR) & 0xF7)); // Set Disp GPIO Direction
}

#if defined(EVE_MO_INTERNAL_BUILD)
void EVE_SPI_Enable(void)
{
  HAL_SPI_Enable();
}

void EVE_SPI_Disable(void)
{
  HAL_SPI_Disable();
}

uint8_t EVE_SPI_Write(uint8_t data)
{
  return HAL_SPI_Write(data);
}

void EVE_SPI_WriteBuffer(uint8_t *Buffer, uint32_t Length)
{
  HAL_SPI_WriteBuffer(Buffer, Length);
}
#endif
