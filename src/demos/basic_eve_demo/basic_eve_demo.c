#ifdef _MSC_VER
#include <conio.h>
#endif
#include "Eve2_81x.h"
#include "MatrixEve2Conf.h"
#include "ST7789V.h"
#include "hw_api.h"

/****************************************************************************
 *                             IMPORTANT NOTICE                              *
 * Please make sure to configure the display timing values and define what   *
 * touch variant you are using in the MatrixEve2Conf.h file.                 *
 *                                                                           *
 ****************************************************************************/

// MakeScreen_MatrixOrbital draws a blue dot in the center screen, along
// with the text "MATRIX ORBITAL"
void MakeScreen_MatrixOrbital(uint8_t DotSize)
{
  Send_CMD(CMD_DLSTART);              // Start a new display list
  Send_CMD(VERTEXFORMAT(0));          // setup VERTEX2F to take pixel coordinates
  Send_CMD(CLEAR_COLOR_RGB(0, 0, 0)); // Determine the clear screen color
  Send_CMD(CLEAR(1, 1, 1));           // Clear the screen and the curren display list
  Send_CMD(COLOR_RGB(26, 26, 192));   // change colour to blue
  Send_CMD(POINT_SIZE(DotSize * 16)); // set point size to DotSize pixels. Points = (pixels x 16)
  Send_CMD(BEGIN(POINTS));            // start drawing point
  Send_CMD(TAG(1));                   // Tag the blue dot with a touch ID
  Send_CMD(VERTEX2F(Display_Width() / 2, Display_Height() / 2)); // place blue point
  Send_CMD(END());                                               // end drawing point
  Send_CMD(COLOR_RGB(255, 255, 255));                            // Change color to white for text
  Cmd_Text(Display_Width() / 2,
           Display_Height() / 2,
           30,
           OPT_CENTER,
           " MATRIX         ORBITAL"); // Write text in the center of the screen
  Send_CMD(DISPLAY());                 // End the display list
  Send_CMD(CMD_SWAP);                  // Swap commands into RAM
  UpdateFIFO();                        // Trigger the CoProcessor to start processing the FIFO
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
  FT81x_Init(DEMO_DISPLAY,
             DEMO_BOARD,
             DEMO_TOUCH); // Initialize the EVE graphics controller. Make sure to define which
                          // display you are using in the MatrixEveConf.h
  ClearScreen(); // Clear any remnants in the RAM

  // If you are using a touch screen, make sure to define what
  // variant you are using in the MatrixEveConf.h file
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
