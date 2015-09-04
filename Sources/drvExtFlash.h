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
 * Module       : drvExtFlash.h
 * Description  : This file declares the interface to the SPI flash driver.
 *
 *****************************************************************************/

#ifndef __drvExtFlash_H
#define __drvExtFlash_H

/* MODULE drvExtFlash */


#include "platform.h"


/* ST Micro M25P40 flash device size parameters */
#define DRV_EXTFLASH_PAGE_SIZE      256
#define DRV_EXTFLASH_SECTOR_SIZE    (256 * DRV_EXTFLASH_PAGE_SIZE)
#define DRV_EXTFLASH_DEVICE_SIZE    (8 * DRV_EXTFLASH_SECTOR_SIZE)


/*
 * Application Interface
 */

bool_t drvExtFlashRead(uint32_t offset, void *pBuf, uint32_t nbytes);
bool_t drvExtFlashWrite(const void *pBuf, uint32_t offset, uint32_t nbytes);
bool_t drvExtFlashErase(uint32_t offset);
bool_t drvExtFlashBusy(void);


/* END drvExtFlash */

#endif
