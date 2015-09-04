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
 * Module       : drvLed.c
 * Description  : This file implements the LED driver, including support for
 *                the function switch LEDs and the three status/expansion LEDs.
 *
 *****************************************************************************/

/* MODULE drvLed */

#include "global.h"
#include "drv.h"
#include "hwBusData.h"
#include "hwBusLatch3.h"
#include "hwLed.h"

#include "drvLed.h"


/******************************************************************************
 *
 *  drvLedFunc
 *
 *  DESCRIPTION:
 *      This driver API function sets the state of the function switch LEDs.
 *
 *  PARAMETERS:
 *      func (in) - Value (0..15) of function LED to activate.
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvLedFunc(uint8_t func)
{
    EnterCritical();                    /* save and disable interrupts */

    drvLatch3 = (uint8_t)((drvLatch3 & ~0xF0) | ((func & 0x0F) << 4));
    hwBusData_PutVal(drvLatch3);
    hwBusLatch3_SetVal();
    hwBusLatch3_ClrVal();

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvLedFault
 *
 *  DESCRIPTION:
 *      This driver API function sets the state of the fault LED.
 *
 *  PARAMETERS:
 *      state (in) - TRUE = on, FALSE = off
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvLedFault(bool_t state)
{
    EnterCritical();                    /* save and disable interrupts */

    if (state)
    {
        hwLed_SetBit(0);
    }
    else
    {
        hwLed_ClrBit(0);
    }

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvLedError
 *
 *  DESCRIPTION:
 *      This driver API function sets the state of the error LED.
 *
 *  PARAMETERS:
 *      state (in) - TRUE = on, FALSE = off
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvLedError(bool_t state)
{
    EnterCritical();                    /* save and disable interrupts */

    if (state)
    {
        hwLed_SetBit(1);
    }
    else
    {
        hwLed_ClrBit(1);
    }

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvLedDebug
 *
 *  DESCRIPTION:
 *      This driver API function sets the state of the debug LED.
 *
 *  PARAMETERS:
 *      state (in) - TRUE = on, FALSE = off
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvLedDebug(bool_t state)
{
    EnterCritical();                    /* save and disable interrupts */

    if (state)
    {
        hwLed_SetBit(2);
    }
    else
    {
        hwLed_ClrBit(2);
    }

    ExitCritical();                     /* restore interrupts */
}


/* END drvLed */
