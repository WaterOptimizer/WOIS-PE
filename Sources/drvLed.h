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
 * Module       : drvLed.h
 * Description  : This file declares the interface to the LED driver,
 *                including support for the function switch LEDs and the three
 *                status/expansion LEDs.
 *
 *****************************************************************************/

#ifndef __drvLed_H
#define __drvLed_H

/* MODULE drvLed */


/*
 * Application Interface
 */

void drvLedFunc(uint8_t func);
void drvLedFault(bool_t state);
void drvLedError(bool_t state);
void drvLedDebug(bool_t state);


/* END drvLed */

#endif
