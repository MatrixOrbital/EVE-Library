#ifndef __ST7789V_H
#define __ST7789V_H

/* For interal builds a dll version of this code is supported */
#if defined(EVE_MO_INTERNAL_BUILD)
#include "eve_export.h"
#else
#define EVE_EXPORT
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// =====================================================================================
// Required Functions - Hardware driver or otherwise environment specific. Abstracted  |
// and found in hw_api.h.                                                              |
// This library requires base support functions for SPI, delays, and hardware pin      |
// control.                                                                            |
// =====================================================================================

#ifdef __cplusplus
extern "C"
{
#endif

  void EVE_EXPORT GPIOX_WriteBit(uint8_t data, bool state);
  void EVE_EXPORT MO_SPIBB_CS(uint8_t enable);
  void EVE_EXPORT MO_SPIBB_Read(void);
  void EVE_EXPORT MO_SPIBB_Send(bool type, uint8_t data);
  void EVE_EXPORT MO_ST7789V_init(void);

#ifdef __cplusplus
}
#endif

#endif
