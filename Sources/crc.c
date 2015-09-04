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
 * Description  : This file implements the crc calculation utility
 *
 *****************************************************************************/

/* Used for building in Windows environment. */
#include "stdafx.h"

#include "global.h"
#include "crc.h"


/******************************************************************************
 *
 * FUNCTION NAME
 *      crc16
 *
 * PURPOSE
 *      This routine calculates the 16-bit CRC checksum for a block of data.
 *
 * PARAMETERS
 *      IN  pBuf        pointer to data
 *      IN  length      length of data
 *
 * RETURN VALUE
 *      This routine returns the calculated 16-bit CRC checksum.
 *
 * NOTES
 *      The CRC is calculated using the CRC-CCITT 16-bit CRC algorithm
 *      (ISO/IEC 13239).  The polynomial is x^16 + x^12 + x^5 + 1, or
 *      hexadecimal 0x1021 (MSB first).  The initial value for the CRC
 *      (the CRC preset value) is 0xFFFF.
 *
 *****************************************************************************/
uint16_t crc16(const void *pBuf, uint16_t length)
{
    const uint8_t *pByte = (const uint8_t *)pBuf;
    uint16_t crc;
    int i;

    crc = CRC_INIT_VALUE;
    for (i = 0; i < length; i++)
    {
        crcByte(pByte[i], &crc);
    }

    return crc;
}


/******************************************************************************
 *
 * FUNCTION NAME
 *      crcByte
 *
 * PURPOSE
 *      This function is helper routine that calculates the CRC of a single
 *      data byte.
 *
 * PARAMETERS
 *      IN      inByte  a data byte to calculate the CRC for
 *      IN/OUT  crc     pointer to CRC value to update with the new CRC value
 *
 * RETURN VALUE
 *      None
 *
 * NOTES
 *      The CRC is calculated using the CRC-CCITT 16-bit CRC algorithm
 *      (ISO/IEC 13239).  The polynomial is x^16 + x^12 + x^5 + 1, or
 *      hexadecimal 0x1021 (MSB first).
 *
 *****************************************************************************/
void crcByte(uint8_t inByte, uint16_t *crc)
{
    int i;
    *crc = *crc ^ ((uint16_t)inByte << 8);

    for (i = 0; i < 8; i++)
    {
        *crc = (*crc & 0x8000) ?
               ((*crc << 1) ^ CRC_POLYNOMIAL) :
               (*crc << 1);
    }
}
