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
 * Module       : drv.c
 * Description  : This file defines global driver data and implements generic
 *                driver support functions.
 *
 *****************************************************************************/

/* MODULE drv */


#include "global.h"

#include "drv.h"


/* Soft copies of write-only latch values */
uint8_t drvLatch1 = 0x00;
uint8_t drvLatch2 = 0x00;
uint8_t drvLatch3 = 0x00;


#if 0
/******************************************************************************
 *
 *  drvIlmSet
 *
 *  DESCRIPTION:
 *      This driver utility function sets a new interrupt level mask and
 *      returns the previous level.  It it used to raise the interrupt mask
 *      level and to restore a previous level.
 *
 *      Typical usage is:
 *          volatile uint8_t ilm;
 *          ...
 *          ilm = drvIlmSet(7);     // enter critical region
 *          ...
 *          (void)drvIlmSet(ilm);   // exit critical region
 *
 *  PARAMETERS:
 *      level (in) - ColdFire interrupt mask level to set/restore (0..7)
 *
 *  RETURNS:
 *      previous ColdFire interrupt mask level (0..7)
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
asm uint8_t drvIlmSet(uint8_t level)
{
    fralloc +

    move.w  SR, D0
    lsr.l   #8, D0
    and.l   #7, D0

    move.w  level, D1
    and.l   #7, D1
    lsl.l   #8, D1

    move.w  SR, D2
    and.l   #0xF8FF, D2
    or.l    D1, D2
    move.w  D2, SR

    frfree

    rts
}
#endif


/* END drv */
