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
 * Module       : platform.c
 * Description  : This file implements temporary stubs used during stage 1 & 2
 *                development to support the dual platform environment.
 *
 *****************************************************************************/

/* MODULE platform */


/* Used for building in Windows environment. */
#include "stdafx.h"

#include <ctype.h>

#include "global.h"
#include "platform.h"
#include "system.h"




#ifdef WIN32

/****** Stubs needed to run on Stage1 (Windows) platform. ********************/

/*
**  Byte-swap routines needed for configuration image compatibility between
**  Coldfire platform and Windows platform.
*/

uint16_t htons(uint16_t hostshort)
{
    uint16_t netshort = (hostshort >> 8) | (hostshort << 8);
    return netshort;
}

uint16_t ntohs(uint16_t netshort)
{
    uint16_t hostshort = (netshort >> 8) | (netshort << 8);
    return hostshort;
}

uint32_t htonl(uint32_t hostlong)
{
    uint32_t netlong;
    uint16_t hosthigh = (uint16_t)(hostlong >> 16);
    uint16_t hostlow = (uint16_t)(hostlong & 0x0000FFFF);
    uint16_t nethigh = htons(hostlow);
    uint16_t netlow = htons(hosthigh);
    netlong = (nethigh << 16) | netlow;
    return netlong;
}

uint32_t ntohl(uint32_t netlong)
{
    uint32_t hostlong;
    uint16_t nethigh = (uint16_t)(netlong >> 16);
    uint16_t netlow = (uint16_t)(netlong & 0x0000FFFF);
    uint16_t hosthigh = ntohs(netlow);
    uint16_t hostlow = ntohs(nethigh);
    hostlong = (hosthigh << 16) | hostlow;
    return hostlong;
}

uint64_t htonll(uint64_t hostlonglong)
{
    return ntohll(hostlonglong);
}

uint64_t ntohll(uint64_t netlonglong)
{
    return ((uint64_t)ntohl((uint32_t)(netlonglong & 0xFFFFFFFF)) << 32 |
            ntohl((uint32_t)(netlonglong >> 32)));
}


/****** End of Stage1 stubs. *************************************************/

#else /* !WIN32 */

/****** Stubs needed to run on Stage2 (pre-EVT hardware) platform. ***********/

#include "hwExpOut.h"


/*
 * Debug output functions.
 */

void debugPutc(char ch)
{
    if (ch == '\n')
    {
        debugPutc('\r');
    }
    while (hwExpOut_SendChar(ch) != ERR_OK)
    {
        ;
    }
}

void debugPuts(const char *pStr)
{
    while (*pStr != '\0')
    {
        debugPutc(*pStr++);
    }
}

/****** End of Stage2 stubs. *************************************************/

#endif /* !WIN32 */


/****** Stubs common to Stage1 & Stage2 platforms. ***************************/


void debugHexWrite(const uint8_t *pBuf, int length, bool_t withAscii)
{
    char field[10];

    for (int i = 0; i < length; i++)
    {
        sprintf(field, "%02X ", pBuf[i]);
        debugWrite(field);
        /* Make sure watchdog doesn't timeout on long debug message writes. */
        sysExecutionExtend();
    }
    debugWrite("\n");
    /* Now output same line in ASCII. */
    if (withAscii)
    {
        for (int i = 0; i < length; i++)
        {
            sprintf(field, " %c ", isprint(pBuf[i]) ? pBuf[i] : '.');
            debugWrite(field);
        }
        debugWrite("\n");
    }
}


/* END platform */
