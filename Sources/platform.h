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
 * Module       : platform.h
 * Description  : This file declares temporary stubs used during stage 1 & 2
 *                development to support the dual platform environment.
 *
 *****************************************************************************/

#ifndef __platform_H
#define __platform_H

/* MODULE platform */


/******************************************************************************
//
//  Only used in Stage 1 Prototype.
//
******************************************************************************/
#ifdef WIN32

#define sprintf wsprintfA
#define strcpy strcpy_s

void debugWrite(const char *pData);

#endif  /* WIN32 */


/******************************************************************************
 *
 *  Only used in Stage 2 Prototype.
 *
 *****************************************************************************/
#ifndef WIN32

#define ENABLE_BBU  1           /* include BBU function */

/*
 * Debug functions.
 */
void debugPutc(char ch);
void debugPuts(const char *pStr);
#define debugWrite(pData)       debugPuts(pData)

#endif  /* !WIN32 */


/******************************************************************************
 *
 *  Platform functions common to Stage1 and Stage2 platforms.
 *
 *****************************************************************************/

/* MACROS */

/* Make uint16_t from 2 uint8_t bytes, a=MSB */
#define U8TOU16(a, b)           ((uint16_t)(((0U + a) << 8) | (0U + b)))

/* Make uint32_t from 4 uint8_t bytes, a=MSB */
#define U8TOU32(a, b, c, d)     (((0UL + a) << 24) | ((0UL + b) << 16) | ((0UL + c) << 8) | (0UL + d))

/* Make uint64_t from 8 uint8_t bytes, a=MSB */
#define U8TOU64(a, b, c, d, e, f, g, h)     ((uint64_t)(U8TOU32(a, b, c, d)) << 32 | (U8TOU32(e, f, g, h)))


#ifdef WIN32
/*  Windows platform needs byte-swap functions for network byte order. */
uint16_t htons(uint16_t hostshort);
uint16_t ntohs(uint16_t netshort);
uint32_t htonl(uint32_t hostlong);
uint32_t ntohl(uint32_t netlong);
uint64_t htonll(uint64_t hostlonglong);
uint64_t ntohll(uint64_t netlonglong);
#else
/* Coldfire platform has native byte order same as network byte order. */
#define htons(x)            (x)
#define ntohs(x)            (x)
#define htonl(x)            (x)
#define ntohl(x)            (x)
#define htonll(x)           (x)
#define ntohll(x)           (x)
#endif

/* Used for enablin softkeys in Stage 1 Windows Prototype. */
void ledSet(uint8_t bitVec);

/*
 * Debug functions.
 */
void debugHexWrite(const uint8_t *pBuf, int length, bool_t withAscii);


/* END platform */

#endif
