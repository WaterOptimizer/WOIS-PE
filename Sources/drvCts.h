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
 * Module       : drvCts.h
 * Description  : This file declares the internal driver interface for CTS
 *                signal management.
 *
 *****************************************************************************/

#ifndef __drvCts_H
#define __drvCts_H

/* MODULE drvCts */


/*
 * Internal Driver Interfaces
 */

/* CTS signals */
typedef enum
{
    DRV_CTS_XBEE,
    DRV_CTS_UART2,
    DRV_CTS_UART1,
} drvCtsSignal_t;

void drvCtsInit(void);
bool drvCtsGet(drvCtsSignal_t signal);
void drvCtsIntrEnable(drvCtsSignal_t signal);


/* END drvCts */

#endif
