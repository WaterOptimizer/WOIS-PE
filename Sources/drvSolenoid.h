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
 * Module       : drvSolenoid.h
 * Description  : This file declares the interface to the solenoid driver.
 *
 *****************************************************************************/

#ifndef __drvSolenoid_H
#define __drvSolenoid_H

/* MODULE drvSolenoid */


/*
 * Application Interface
 */

void   drvSolenoidSet(uint8_t zone, bool_t state);
bool_t drvSolenoidGet(uint8_t zone);


/*
 * Internal Driver Interfaces
 */
void drvSolenoidIsr(void);


/* END drvSolenoid */

#endif
