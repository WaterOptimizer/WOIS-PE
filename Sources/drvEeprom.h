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
 * Module       : drvEeprom.h
 * Description  : This file declares the interface to the EEPROM driver.
 *
 *****************************************************************************/

#ifndef __drvEeprom_H
#define __drvEeprom_H

/* MODULE drvEeprom */


/*
 * Application Interface
 */

#define DRV_EEPROM_PAGE_SIZE    64

bool_t drvEepromRead(uint32_t offset, void *pBuf, uint32_t nbytes);
bool_t drvEepromWrite(const void *pBuf, uint32_t offset, uint32_t nbytes);


/*
 * Internal Driver Interfaces
 */
void drvEepromReset(void);


/* END drvEeprom */

#endif
