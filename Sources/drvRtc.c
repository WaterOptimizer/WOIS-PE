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
 * Module       : drvRtc.c
 * Description  : This file implements the Real-Time-Clock driver, providing
 *                a 1Hz time reference.
 *
 *****************************************************************************/

/* MODULE drvRtc */

#include "global.h"
#include "hwRtc.h"

#include "drvRtc.h"


static uint32_t drvRtcTicks = 0;        /* system uptime in seconds */
static uint32_t drvMSTicks = 0;         /* system uptime in milliseconds */


/******************************************************************************
 *
 *  drvRtcIsr
 *
 *  DESCRIPTION:
 *      This driver internal function is invoked by a 1 second timer ISR to
 *      perform real-time-clock timekeeping functions.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is called from a timer ISR.
 *
 *****************************************************************************/
void drvRtcIsr(void)
{
    drvRtcTicks++;
}


/******************************************************************************
 *
 *  drvRtcAccel
 *
 *  DESCRIPTION:
 *      This driver API function selects between normal and accelerated RTC
 *      rate.  The accelerated rate (32 Hz) is intended only for product
 *      development use.
 *
 *  PARAMETERS:
 *      isFast (in) - TRUE = accelerated rate, FALSE = normal 1 Hz rate
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvRtcAccel(bool_t isFast)
{
    if (isFast)
    {
        hwRtc_SetPeriodMode(hwRtc_Pm_32Hz);
    }
    else
    {
        hwRtc_SetPeriodMode(hwRtc_Pm_1Hz);
    }
}


/******************************************************************************
 *
 *  drvRtcGet
 *
 *  DESCRIPTION:
 *      This driver API function returns the RTC tick count, which is the count
 *      of seconds since the system started (initial power-up or reset).
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      count of one-second ticks since system start-up
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
uint32_t drvRtcGet(void)
{
    return drvRtcTicks;
}


/******************************************************************************
 *
 *  drvMSIsr
 *
 *  DESCRIPTION:
 *      This driver internal function is invoked by a fast timer ISR to
 *      provide a high-speed time reference.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is called from the 2ms timer ISR.
 *
 *****************************************************************************/
void drvMSIsr(void)
{
    drvMSTicks += 2;
}


/******************************************************************************
 *
 *  drvMSGet
 *
 *  DESCRIPTION:
 *      This driver API function returns the fast clock tick count, which is
 *      the count of milliseconds since the system started (initial power-up
 *      or reset).
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      count of millisecond ticks since system start-up
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *      This does not yield true millisecond timing resolution as it is
 *      incremented at a slower rate.  Also, it is NOT synchronized to the
 *      one tick/second RTC time base.
 *
 *****************************************************************************/
uint32_t drvMSGet(void)
{
    return drvMSTicks;
}


/* END drvRtc */
