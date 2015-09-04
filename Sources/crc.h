/******************************************************************************
 *                       Copyright (c) 2008, Jabil Circuit
 *
 * This source code and any compilation or derivative thereof is the sole
 * property of Jabil Circuit and is provided pursuant to a Software License
 * Agreement.  This code is the proprietary information of Jabil Circuit and
 * is confidential in nature.  Its use and dissemination by any party other
 * than Jabil Circuit is strictly limited by the confidential information
 * provisions of the Software License Agreement referenced above.
 *
 ******************************************************************************
 *
 * Project      : WaterOptimizer Irrigation System (WOIS)
 * Organization : WaterOptimizer, LLC
 * Module       : crc.c
 * Description  : This file defines the crc calculation utility
 *
 *****************************************************************************/

#ifndef __crc_H
#define __crc_H

/* MODULE crc */

#include "global.h"

/******************************************************************************
 *
 *  CRC IMPLEMENTATION VALUES
 *
 *****************************************************************************/

/* Initial value (the CRC preset value) to start the CRC calculation */
#define CRC_INIT_VALUE      0xFFFF

/* CRC-CCITT polynomial x^16 + x^12 + x^5 + 1  (MSB first) */
#define CRC_POLYNOMIAL      0x1021


/******************************************************************************
 *
 *  FUNCTION PROTOTYPES
 *
 *****************************************************************************/

/* CRC calculation utility */
uint16_t crc16(const void *pBuf, uint16_t length);

/* CRC calculation helper function */
void crcByte(uint8_t inChar, uint16_t *crc);


/* END crc */

#endif
