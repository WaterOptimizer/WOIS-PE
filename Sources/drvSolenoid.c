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
 * Module       : drvSolenoid.c
 * Description  : This file implements the solenoid driver.
 *
 *****************************************************************************/

/* MODULE drvSolenoid */

#include "global.h"
#include "drv.h"
#include "hwBusData.h"
#include "hwBusLatch1.h"
#include "hwBusLatch2.h"
#include "hwBusLatch3.h"
#include "hwPower24vacControl.h"
#include "system.h"

#include "drvSolenoid.h"


static uint16_t drvSolenoidActual = 0;
static uint16_t drvSolenoidTarget = 0;


/******************************************************************************
 *
 *  drvSolenoidSet
 *
 *  DESCRIPTION:
 *      This driver API function sets/clears the target solenoid state for a
 *      specified zone or master valve solenoid.  Zone solenoids are numbered
 *      1..12.  A value of 0 represents the master valve.  Application logic
 *      should insure that no more than one zone solenoid is activated at any
 *      one time, 
 *      the master valve solenoid is on whenever running an irrigation program
 *
 *  PARAMETERS:
 *      zone (in)  - Value (0..12, master + 12 solenoids) of solenoid to activate/deactivate.
 *      state (in) - TRUE = activated, FALSE = deactivated
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context, but it is not reentrant.
 *
 *****************************************************************************/
void drvSolenoidSet(uint8_t zone, bool_t state)
{
    //everything is already off since;
    // a. there can only be one solenoid on at a time
    // b. the master solenoid should be on unless specifically commanded to turn off
    drvSolenoidTarget = 0x0001;

    if(TRUE == state) 
    {
		      drvSolenoidTarget |= (1 << zone);
    }
    else
    {
        //everything is already off nothing to do;
        
        if(zone == 0)
		    {
		      //master off...everything off
		      drvSolenoidTarget = 0;
		    }
    }
}


/******************************************************************************
 *
 *  drvSolenoidGet
 *
 *  DESCRIPTION:
 *      This driver API function reports the target solenoid state for a
 *      specified zone or master valve solenoid.  Zone solenoids are numbered
 *      1..12.  A value of 0 represents the master valve.
 *
 *  PARAMETERS:
 *      zone (in) - Value (0..12) of solenoid to activate/deactivate.
 *
 *  RETURNS:
 *      state - TRUE = activated, FALSE = deactivated
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
bool_t drvSolenoidGet(uint8_t zone)
{
    return ((drvSolenoidTarget & (1 << zone)) != 0);
}


/******************************************************************************
 *
 *  drvSolenoidIsr
 *
 *  DESCRIPTION:
 *      This internal driver function updates the actual state of the solenoid
 *      outputs to match the application-specified target state.  At most one
 *      solenoid output will be changed on each call.  Turning outputs off is
 *      favored over turning outputs on.  It is called periodically from a
 *      20mS timer interrupt.  This mechanism limits the electrical current
 *      demands due to solenoid switching.  It also allows time for one zone's
 *      TRIAC to turn off (at the next 24VAC zero-crossing) before turning on
 *      another zone's TRIAC.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is invoked from a timer ISR.
 *
 *****************************************************************************/
void drvSolenoidIsr(void)
{
    uint16_t off = (drvSolenoidActual ^ drvSolenoidTarget) & ~drvSolenoidTarget;
    uint16_t on  = (drvSolenoidActual ^ drvSolenoidTarget) & drvSolenoidTarget;

    /* turn-off events take priority over turn-on events */
    if (off != 0)
    {
        /* at least one solenoid to turn off - select lowest numbered one */
        for (int i = 0; i < 13; i++)
        {
            if ((off & (1 << i)) != 0)
            {
                drvSolenoidActual &= ~(1 << i);
                break;
            }
        }
    }
    else if (on != 0)
    {
        /* at least one solenoid to turn on - select lowest numbered one */
        for (int i = 0; i < 13; i++)
        {
            if ((on & (1 << i)) != 0)
            {
                drvSolenoidActual |= (1 << i);
                break;
            }
        }
    }
    else
    {
        return;
    }

    /* update solenoid output latches and 24VAC power control */

    EnterCritical();                    /* save and disable interrupts */

    drvLatch1 = (uint8_t)(drvSolenoidActual >> 1);
    hwBusData_PutVal(drvLatch1);
    hwBusLatch1_SetVal();
    hwBusLatch1_ClrVal();

    drvLatch2 = (uint8_t)((drvLatch2 & ~0x0F) | ((drvSolenoidActual >> 9) & 0x0F));
    hwBusData_PutVal(drvLatch2);
    hwBusLatch2_SetVal();
    hwBusLatch2_ClrVal();

    drvLatch3 = (uint8_t)((drvLatch3 & ~0x01) | (drvSolenoidActual & 0x01));
    hwBusData_PutVal(drvLatch3);
    hwBusLatch3_SetVal();
    hwBusLatch3_ClrVal();

    /* enable 24VAC power if and only if any solenoid is activated */
    hwPower24vacControl_PutVal((drvSolenoidActual != 0) ? 1 : 0);

    ExitCritical();                     /* restore interrupts */
}


/* END drvSolenoid */
