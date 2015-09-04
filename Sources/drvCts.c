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
 * Module       : drvCts.c
 * Description  : This file implements internal driver support for CTS signal
 *                management.
 *
 *****************************************************************************/

/* MODULE drvCts */


#include "global.h"
#include "drvRadio.h"
#include "hwCts.h"

#include "drvCts.h"


/******************************************************************************
 *
 *  drvCtsInit
 *
 *  DESCRIPTION:
 *      This internal driver function initializes the CTS edge-detect interrupt
 *      support.  The interrupt is enabled but all CTS signals are ignored.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is invoked only from the system driver start logic.
 *
 *****************************************************************************/
void drvCtsInit(void)
{
    hwCts_Init();
    KBI2PE = 0x00;                      /* disable all pins at start */
    KBI2SC_KBIE = 1;                    /* and enable the interrupt */
}


/******************************************************************************
 *
 *  drvCtsISR
 *
 *  DESCRIPTION:
 *      This internal driver function is the interrupt service routine that
 *      processes CTS signal transition interrupts.  It is invoked directly by
 *      the hardware via the interrupt vector table.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is an actual ISR, invoked via the interrupt vector table.
 *
 *****************************************************************************/
ISR(drvCtsISR)
{
    uint8_t active = PTDD & 0x62;

    KBI2PE = active;                    /* disable any active (low) pins */
    KBI2SC_KBACK = 0x01;

    if ((active & 0x02) == 0)           /* check pin PTD1 (radio CTS) */
    {
        drvRadioOnCtsAsserted();
    }
    if ((active & 0x20) == 0)           /* check pin PTD5 (Exp-In CTS) */
    {
        /* INSERT ADDITIONAL CODE HERE TO SUPPORT UART2/EXP-IN CTS SIGNAL */
    }
    if ((active & 0x40) == 0)           /* check pin PTD6 (Exp-Out CTS) */
    {
        /* INSERT ADDITIONAL CODE HERE TO SUPPORT UART1/EXP-OUT CTS SIGNAL */
    }
}


/******************************************************************************
 *
 *  drvCtsGet
 *
 *  DESCRIPTION:
 *      This internal driver function returns the current value of the
 *      specified CTS signal.
 *
 *  PARAMETERS:
 *      signal (in) - CTS signal to read
 *
 *  RETURNS:
 *      TRUE if CTS asserted; FALSE if CTS deasserted
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
bool drvCtsGet(drvCtsSignal_t signal)
{
    uint8_t value = PTDD;

    switch (signal)
    {
        case DRV_CTS_XBEE:
            return !PTDD_PTDD1;         /* radio CTS signal on pin PTD1 */
        case DRV_CTS_UART2:
            return !PTDD_PTDD5;         /* Exp-In CTS signal on pin PTD5 */
        case DRV_CTS_UART1:
            return !PTDD_PTDD6;         /* Exp-Out CTS signal on pin PTD6 */
        default:
            return FALSE;
    }
}


/******************************************************************************
 *
 *  drvCtsIntrEnable
 *
 *  DESCRIPTION:
 *      This internal driver function enables edge-detection interrupts for the
 *      specified CTS signal.  This interrupt is used to detect when a CTS
 *      signal transitions from FALSE to TRUE, unblocking our transmitter.
 *
 *  PARAMETERS:
 *      signal (in) - CTS signal to enable
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvCtsIntrEnable(drvCtsSignal_t signal)
{
    EnterCritical();                    /* save and disable interrupts */

    switch (signal)
    {
        case DRV_CTS_XBEE:
            KBI2PE |= (1 << 1);         /* radio CTS signal on pin PTD1 */
            break;
        case DRV_CTS_UART2:
            KBI2PE |= (1 << 5);         /* Exp-In CTS signal on pin PTD5 */
            break;
        case DRV_CTS_UART1:
            KBI2PE |= (1 << 6);         /* Exp-Out CTS signal on pin PTD6 */
            break;
    }

    ExitCritical();                     /* restore interrupts */
}


/* END drvCts */
