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
 * Module       : drvRtc.h
 * Description  : This file declares the interface to the Real-Time-Clock
 *                driver, providing a 1Hz time reference.
 *
 *****************************************************************************/

#ifndef __drvRtc_H
#define __drvRtc_H

/* MODULE drvRtc */


/*
 * Application Interface
 */

void     drvRtcAccel(bool_t isFast);
uint32_t drvRtcGet(void);
uint32_t drvMSGet(void);


/*
 * Internal Driver Interfaces
 */
void drvRtcIsr(void);
void drvMSIsr(void);


/* END drvRtc */

#endif
