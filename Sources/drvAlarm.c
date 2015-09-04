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
 * Module       : drvAlarm.c
 * Description  : This file implements the alarm output driver.
 *
 *****************************************************************************/

/* MODULE drvAlarm */


#include "global.h"
#include "drv.h"
#include "hwBusData.h"
#include "hwBusLatch3.h"

#include "drvAlarm.h"


/******************************************************************************
 *
 *  drvAlarm
 *
 *  DESCRIPTION:
 *      This driver API function activates/deactivates the alarm output.
 *
 *  PARAMETERS:
 *      state (in) - TRUE = activated, FALSE = deactivated
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvAlarm(bool state)
{
    EnterCritical();                    /* save and disable interrupts */

    if (state)
    {
        drvLatch3 |= 0x02;
    }
    else
    {
        drvLatch3 &= ~0x02;
    }
    hwBusData_PutVal(drvLatch3);
    hwBusLatch3_SetVal();
    hwBusLatch3_ClrVal();

    ExitCritical();                     /* restore interrupts */
}


/* END drvAlarm */
