#ifdef _MSC_VER
#include <conio.h>
#endif
#include "MONOSPACE821BT_64_ASTC.glyph.h"
#include "MONOSPACE821BT_64_ASTC.xfont.h"
#include "eve.h"
#include "hw_api.h"

void MakeScreen_HelloWorld()
{
  // Start a new display list
  Send_CMD(CMD_DLSTART);
  // Setup VERTEX2F to take pixel coordinates
  Send_CMD(VERTEXFORMAT(0));
  // Set the clear screen color
  Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
  // Clear the screen
  Send_CMD(CLEAR(1, 1, 1));
  Send_CMD(COLOR_RGB(255, 255, 255));
  // Select the custom font for font 1, the xfont data is written at RAM_G
  Cmd_SetFont2(1, RAM_G, 0);
  Cmd_Text(Display_Width() / 2,
           Display_VOffset() + (Display_Height() / 2),
           1,
           OPT_CENTER,
           "MONOSPACE\n821BT_64");
  // End the display list
  Send_CMD(DISPLAY());
  // Swap commands into RAM
  Send_CMD(CMD_SWAP);
  // Trigger the CoProcessor to start processing the FIFO
  UpdateFIFO();
}

int main()
{
  if (EVE_Init(DEMO_DISPLAY, DEMO_BOARD, DEMO_TOUCH) <= 1)
  {
    printf("ERROR: Eve not detected.\n");
    return -1;
  }

  StartCoProTransfer(RAM_G, 0);
  HAL_SPI_WriteBuffer((uint8_t *)&MONOSPACE821BT_64_ASTC_xfont,
                      sizeof(MONOSPACE821BT_64_ASTC_xfont));
  HAL_SPI_Disable();

  StartCoProTransfer(RAM_G + 4096, 0);
  HAL_SPI_WriteBuffer((uint8_t *)&MONOSPACE821BT_64_ASTC_glyph,
                      sizeof(MONOSPACE821BT_64_ASTC_glyph));
  HAL_SPI_Disable();

  MakeScreen_HelloWorld();
  HAL_Close();
}
