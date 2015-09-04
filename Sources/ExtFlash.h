/******************************************************************************
 *                       Copyright (c) 2008, King Engineering
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
 * Module       : ExtFlash.h
 * Description  : This file declares the interface to the External Flash
 *
 *****************************************************************************/

#ifndef __extFlash_H
#define __extFlash_H

/* MODULE ExtFlash */
#define MAX_FW_IMAGE_SIZE  124000

//flash 512KB sector addresses
#define EXT_FLASH_SEC_SIZE  0x10000
#define EXT_FLASH_SEC_0     0x00000
#define EXT_FLASH_SEC_1     0x10000
#define EXT_FLASH_SEC_2     0x20000
#define EXT_FLASH_SEC_3     0x30000
#define EXT_FLASH_SEC_4     0x40000
#define EXT_FLASH_SEC_5     0x50000
#define EXT_FLASH_SEC_6     0x60000
#define EXT_FLASH_SEC_7     0x70000

#define FW_NEW_INFO_SPI_ADDR          0x00000    // spi flash address for new firmware info 
#define FW_NEW_CODE_SPI_ADDR          0x10000    // spi flash address for a firmware version
#define FW_FACTORY_INFO_SPI_ADDR      0x40000    // spi flash address for factory info
#define FW_FACTORY_CODE_SPI_ADDR      0x50000    // spi flash address for factory version

/*
 * Application Interface
 */
bool_t extFlashBufferWrite(const void *pBuf, uint32_t offset, uint32_t len);
bool_t extFlashBufferRead(uint32_t offset, void *pBuf, uint32_t len);
bool_t extFlashFWErase(uint32_t offset);

/* END extFlash */

#endif
