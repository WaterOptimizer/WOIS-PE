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
 * Module       : drvLcd.h
 * Description  : This file declares the interface to the LCD driver.
 *
 *****************************************************************************/

#ifndef __drvLcd_H
#define __drvLcd_H

/* MODULE drvLcd */


/*
 * Application Interface
 */

#define DRV_LCD_CURSOR_RC(row,col)     (((row) * 40) + (col))
#define DRV_LCD_CURSOR_OFF              -1      /* disable cursor */

void drvLcdWrite(const char *pData, uint8_t cursor);


/*
 * Internal Driver Interfaces
 */
void drvLcdRestart(void);


/* END drvLcd */

#endif
