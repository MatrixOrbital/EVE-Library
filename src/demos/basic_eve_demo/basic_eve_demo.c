#ifdef _MSC_VER
#include <conio.h>
#endif
#include "eve.h"
#include "hw_api.h"

// MakeScreen_MatrixOrbital draws a blue dot in the center screen, along
// with the text "MATRIX ORBITAL"
void MakeScreen_MatrixOrbital(uint8_t DotSize)
{
  // Start a new display list
  Send_CMD(CMD_DLSTART);
  // Setup VERTEX2F to take pixel coordinates
  Send_CMD(VERTEXFORMAT(0));
  // Set the clear screen color
  Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
  // Clear the screen
  Send_CMD(CLEAR(1, 1, 1));
  // change color to blue
  Send_CMD(COLOR_RGB(26, 26, 192));
  // set point size to DotSize pixels. Points = (pixels x 16)
  Send_CMD(POINT_SIZE(DotSize * 16));
  // start drawing a point
  Send_CMD(BEGIN(POINTS));
  // Tag the blue dot with a touch ID of 1
  Send_CMD(TAG(1));
  // place blue point in the center of the screen, this is offset by Display_VOffset() since
  // some displays like the 38 have to be driven at a higher resolution than what is visible
  // on the physical display, Display_VOffset() will return the first visible line on the display
  Send_CMD(VERTEX2F(Display_Width() / 2, Display_VOffset() + (Display_Height() / 2)));
  // end drawing point
  Send_CMD(END());
  // Change color to white for text
  Send_CMD(COLOR_RGB(255, 255, 255));
  // Write text in the center of the screen
  Cmd_Text(Display_Width() / 2,
           Display_VOffset() + (Display_Height() / 2),
           30,
           OPT_CENTER,
           " MATRIX         ORBITAL");
  // End the display list
  Send_CMD(DISPLAY());
  // Swap commands into RAM
  Send_CMD(CMD_SWAP);
  // Trigger the CoProcessor to start processing the FIFO
  UpdateFIFO();
}

// A calibration screen for the touch digitizer
void Calibrate(void)
{
  Calibrate_Manual(Display_Width(), Display_Height(), Display_VOffset(), Display_HOffset());
}

// A Clear screen function
void ClearScreen(void)
{
  Send_CMD(CMD_DLSTART);
  Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
  Send_CMD(CLEAR(1, 1, 1));
  Send_CMD(DISPLAY());
  Send_CMD(CMD_SWAP);
  UpdateFIFO();          // Trigger the CoProcessor to start processing commands out of the FIFO
  Wait4CoProFIFOEmpty(); // wait here until the coprocessor has read and executed every pending
                         // command.
  HAL_Delay(10);
}

int main()
{
  if (EVE_Init(DEMO_DISPLAY, DEMO_BOARD, DEMO_TOUCH) <= 1)
  {
    printf("ERROR: Eve not detected.\n");
    return -1;
  }

  ClearScreen(); // Clear any remnants in the RAM

  if (Display_Touch() == TOUCH_TPR)
  {
    Calibrate();
  }

  MakeScreen_MatrixOrbital(30); // Draw the Matrix Orbital Screen
  uint8_t pressed = 0;

  while (1)
  {
#ifdef _MSC_VER
    if (_kbhit())
      break;
#endif
    uint8_t Tag = rd8(REG_TOUCH_TAG + RAM_REG); // Check for touches
    switch (Tag)
    {
    case 1:
      if (!pressed)
      {
        MakeScreen_MatrixOrbital(120); // Blue dot is 120 when not touched
        pressed = 1;
      }
      break;
    default:
      if (pressed)
      {
        pressed = 0;
        MakeScreen_MatrixOrbital(30); // Blue dot size is 30 when not touched
      }
      break;
    }
  }
  HAL_Close();
}
