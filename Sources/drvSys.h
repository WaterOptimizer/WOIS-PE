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
 * Module       : drvSys.h
 * Description  : This file implements the interface to the driver for power
 *                management, including support for mains power fail detection,
 *                backup battery monitoring, and entering/exiting low-power
 *                mode.
 *
 *****************************************************************************/

#ifndef __drvSys_H
#define __drvSys_H

/* MODULE drvSys */


/*
 * Application Interface
 */

#define drvSysMainsOk()                 drvSys12vdcOk()

bool_t  drvSys12vdcOk(void);
bool_t  drvSys24vacOk(void);

void    drvSysShutdown(void);
void    drvSysRestart(void);
void    drvSysStop(void);

void    drvSysWatchDogClear(void);

uint8_t drvSysResetReason(void);

void drvProcessorReboot(void);
/*
 * Internal Driver Interfaces
 */
void drvSysStart(void);                 /* called once by PE startup code */


/* END drvSys */

#endif
